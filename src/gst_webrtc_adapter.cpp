#include "webrtc_gst_cpp/gst_webrtc_adapter.h"
#include "webrtc_gst_cpp/signaling_client.h"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/sdp.h>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <memory>
#include <functional>

using json = nlohmann::json;

struct GstWebRTCAdapter::Impl {
  std::string signaling_url;
  int width;
  int height;
  int fps;

  GMainLoop* loop = nullptr;
  GMainContext* context = nullptr;
  GstElement* pipeline = nullptr;
  GstElement* appsrc = nullptr;
  GstElement* pay0 = nullptr;    // rtpvp8pay
  GstElement* webrtc = nullptr;

  std::unique_ptr<SignalingClient> signaling;
  std::thread glib_thread;
  std::mutex mutex;

  // Simple running PTS based on fps
  GstClockTime running_pts = 0;

  // Negotiation/ICE state
  bool remote_desc_set = false;
  struct PendingCand { int mline; std::string cand; };
  std::vector<PendingCand> pending_cands;

  Impl(const std::string& url, int w, int h, int f)
    : signaling_url(url), width(w), height(h), fps(f) {}
};

// Post helper to run code on GLib main context
static gboolean invoke_trampoline(gpointer data) {
  std::unique_ptr<std::function<void()>> fn(reinterpret_cast<std::function<void()>*>(data));
  (*fn)();
  return G_SOURCE_REMOVE;
}
static void post_main(GstWebRTCAdapter::Impl* self, std::function<void()> fn) {
  if (!self || !self->context) return;
  g_main_context_invoke(self->context, invoke_trampoline, new std::function<void()>(std::move(fn)));
}

// Helpers
static void add_ice_candidate_safe(GstElement* webrtc, int mline, const std::string& cand) {
  if (!webrtc) return;
  if (cand.empty()) return;
  // Use 2-parameter signature for wider compatibility:
  // g_signal_emit_by_name(webrtc, "add-ice-candidate", guint mlineindex, gchar* candidate);
  g_signal_emit_by_name(webrtc, "add-ice-candidate", (guint)mline, cand.c_str());
}

static void flush_pending_candidates(GstWebRTCAdapter::Impl* self) {
  if (!self || !self->webrtc) return;
  for (const auto& pc : self->pending_cands) {
    add_ice_candidate_safe(self->webrtc, pc.mline, pc.cand);
  }
  self->pending_cands.clear();
}

static void set_remote_description_async(GstWebRTCAdapter::Impl* self,
                                         const std::string& sdp_text,
                                         GstWebRTCSDPType type,
                                         GstPromiseChangeFunc done_cb) {
  if (!self || !self->webrtc) return;
  if (sdp_text.empty()) {
    std::cerr << "[webrtc] Remote SDP is empty\n";
    return;
  }
  GstSDPMessage *sdp = nullptr;
  if (gst_sdp_message_new(&sdp) != GST_SDP_OK) {
    std::cerr << "[webrtc] Failed to alloc SDP message\n";
    return;
  }
  const GstSDPResult r = gst_sdp_message_parse_buffer(
      reinterpret_cast<const guint8*>(sdp_text.data()),
      sdp_text.size(), sdp);
  if (r != GST_SDP_OK) {
    std::cerr << "[webrtc] Failed to parse SDP text, code=" << r << "\n";
    gst_sdp_message_free(sdp);
    return;
  }
  GstWebRTCSessionDescription *desc = gst_webrtc_session_description_new(type, sdp);
  GstPromise *p = gst_promise_new_with_change_func(done_cb, self, nullptr);
  std::cout << "[webrtc] set-remote-description (type=" << (type == GST_WEBRTC_SDP_TYPE_OFFER ? "offer" : "answer") << ")\n";
  g_signal_emit_by_name(self->webrtc, "set-remote-description", desc, p);
  gst_webrtc_session_description_free(desc);
}

static void on_set_local_desc_done(GstPromise* promise, gpointer /*user_data*/) {
  gst_promise_unref(promise);
}

static void on_answer_created(GstPromise *promise, gpointer user_data) {
  auto *self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  if (!self || !self->webrtc || !self->signaling) {
    gst_promise_unref(promise);
    return;
  }
  const GstStructure *reply = gst_promise_get_reply(promise);
  GstWebRTCSessionDescription *answer = nullptr;
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, NULL);
  gst_promise_unref(promise);
  if (!answer) {
    std::cerr << "[webrtc] Failed to get generated answer\n";
    return;
  }

  // Set local description
  GstPromise *p = gst_promise_new_with_change_func(on_set_local_desc_done, self, nullptr);
  g_signal_emit_by_name(self->webrtc, "set-local-description", answer, p);

  // Serialize SDP and send
  gchar *sdp_text = gst_sdp_message_as_text(answer->sdp);
  if (!sdp_text) {
    gst_webrtc_session_description_free(answer);
    std::cerr << "[webrtc] Failed to serialize SDP answer\n";
    return;
  }
  json j = { {"type", "answer"}, {"sdp", std::string(sdp_text)} };
  std::cout << "[signaling] send answer (" << strlen(sdp_text) << " bytes)\n";
  self->signaling->Send(j.dump());
  g_free(sdp_text);
  gst_webrtc_session_description_free(answer);
}

static void on_set_remote_desc_done(GstPromise *promise, gpointer user_data) {
  auto *self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  gst_promise_unref(promise);
  if (!self || !self->webrtc) return;

  self->remote_desc_set = true;
  std::cout << "[webrtc] remote description set → flushing " << self->pending_cands.size() << " pending ICE candidates\n";
  flush_pending_candidates(self);

  // Create answer (Answerer mode)
  GstPromise *p = gst_promise_new_with_change_func(on_answer_created, self, nullptr);
  std::cout << "[webrtc] create-answer\n";
  g_signal_emit_by_name(self->webrtc, "create-answer", nullptr, p);
}

static void on_ice_candidate(GstElement * /*webrtc*/, guint mlineindex, gchar *candidate, gpointer user_data) {
  auto *self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  if (!self || !self->signaling) return;

  json j = {
    {"type", "candidate"},
    {"candidate", std::string(candidate ? candidate : "")},
    {"sdpMLineIndex", static_cast<int>(mlineindex)}
  };
  // Optional: std::cout << "[signaling] send local ICE cand mline=" << (int)mlineindex << "\n";
  self->signaling->Send(j.dump());
}

// Debug helper to list pad templates
static void debug_list_pad_templates(GstElement* elem, const char* name) {
  if (!elem) return;
  GstElementFactory* f = gst_element_get_factory(elem);
  if (!f) return;
  std::cerr << "[debug] Pad templates for " << (name ? name : gst_element_get_name(elem)) << ":\n";
  const GList* templates = gst_element_factory_get_static_pad_templates(f);
  for (const GList* it = templates; it; it = it->next) {
    GstStaticPadTemplate* t = (GstStaticPadTemplate*)it->data;
    std::cerr << "  - " << t->name_template << " ("
              << (t->direction == GST_PAD_SINK ? "SINK" : "SRC") << "), presence="
              << (t->presence == GST_PAD_ALWAYS ? "ALWAYS" :
                  t->presence == GST_PAD_SOMETIMES ? "SOMETIMES" : "REQUEST")
              << "\n";
  }
}

GstWebRTCAdapter::GstWebRTCAdapter(const std::string& signaling_url, int width, int height, int fps)
  : impl_(new Impl(signaling_url, width, height, fps)) {}

GstWebRTCAdapter::~GstWebRTCAdapter() {
  if (!impl_) return;
  if (impl_->pipeline) {
    gst_element_set_state(impl_->pipeline, GST_STATE_NULL);
    gst_object_unref(impl_->pipeline);
    impl_->pipeline = nullptr;
  }
  if (impl_->loop) {
    g_main_loop_quit(impl_->loop);
    if (impl_->glib_thread.joinable()) impl_->glib_thread.join();
    g_main_loop_unref(impl_->loop);
    impl_->loop = nullptr;
  }
}

bool GstWebRTCAdapter::Start() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  gst_init(nullptr, nullptr);
  impl_->loop = g_main_loop_new(nullptr, FALSE);
  impl_->context = g_main_loop_get_context(impl_->loop);

  // Build pipeline to rtpvp8pay; we add webrtcbin programmatically to link request pad properly
  std::string pipeline_str =
    "appsrc name=src is-live=true block=true format=time "
    "caps=video/x-raw,format=BGR,width=" + std::to_string(impl_->width) +
    ",height=" + std::to_string(impl_->height) + ",framerate=" + std::to_string(impl_->fps) + "/1 "
    "! videoconvert ! queue "
    "! vp8enc deadline=1 cpu-used=5 error-resilient=1 keyframe-max-dist=60 "
    "! rtpvp8pay pt=96 name=pay0";

  GError *error = nullptr;
  impl_->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
  if (!impl_->pipeline) {
    std::cerr << "Failed to create pipeline: " << (error ? error->message : "unknown") << std::endl;
    if (error) g_error_free(error);
    return false;
  }

  impl_->appsrc = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "src");
  impl_->pay0   = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "pay0");
  if (!impl_->appsrc || !impl_->pay0) {
    std::cerr << "Failed to get appsrc or pay0 element from pipeline\n";
    return false;
  }

  // Create and add webrtcbin
  impl_->webrtc = gst_element_factory_make("webrtcbin", "webrtc");
  if (!impl_->webrtc) {
    std::cerr << "Failed to create webrtcbin element (is gstreamer-webrtc installed?)\n";
    return false;
  }
  gst_bin_add(GST_BIN(impl_->pipeline), impl_->webrtc);

  // Optional STUN (LAN thường không cần)
  // g_object_set(impl_->webrtc, "stun-server", "stun://stun.l.google.com:19302", nullptr);

  // Link pay0:src -> webrtcbin:sink_0 (correct request pad for webrtcbin)
  GstPad* pay_src = gst_element_get_static_pad(impl_->pay0, "src");
  // GstPad* webrtc_sink = gst_element_get_request_pad(impl_->webrtc, "sink_0");

  // Fix 1:
  GstPad* webrtc_sink = gst_element_get_request_pad(impl_->webrtc, "sink_%u");

  if (!pay_src || !webrtc_sink) {
    if (!pay_src) std::cerr << "Failed to get pay0 src pad\n";
    if (!webrtc_sink) {
      std::cerr << "Failed to request webrtcbin sink_0 pad\n";
      debug_list_pad_templates(impl_->webrtc, "webrtcbin");
    }
    if (pay_src) gst_object_unref(pay_src);
    if (webrtc_sink) gst_object_unref(webrtc_sink);
    std::cerr << "Failed to get pads for linking pay0 -> webrtcbin\n";
    return false;
  }
  if (gst_pad_link(pay_src, webrtc_sink) != GST_PAD_LINK_OK) {
    gst_object_unref(pay_src);
    gst_object_unref(webrtc_sink);
    std::cerr << "Failed to link pay0 src to webrtcbin sink_0\n";
    return false;
  }
  gst_object_unref(pay_src);
  gst_object_unref(webrtc_sink);

  // ICE callback (will be emitted on the GStreamer thread)
  g_signal_connect(impl_->webrtc, "on-ice-candidate", G_CALLBACK(on_ice_candidate), impl_.get());

  // Explicit caps for appsrc (redundant to caps= above, but safe)
  GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                      "format", G_TYPE_STRING, "BGR",
                                      "width", G_TYPE_INT, impl_->width,
                                      "height", G_TYPE_INT, impl_->height,
                                      "framerate", GST_TYPE_FRACTION, impl_->fps, 1,
                                      NULL);
  g_object_set(impl_->appsrc,
               "caps", caps,
               "stream-type", 0,
               "format", GST_FORMAT_TIME,
               "is-live", TRUE,
               "block", TRUE,
               NULL);
  gst_caps_unref(caps);

  // Start pipeline
  if (gst_element_set_state(impl_->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to set pipeline to PLAYING\n";
    return false;
  }

  // Start GLib main loop
  impl_->glib_thread = std::thread([this]() {
    g_main_loop_run(impl_->loop);
  });

  // Setup signaling
  impl_->signaling = std::make_unique<SignalingClient>(impl_->signaling_url);
  impl_->signaling->SetOnOpen([this]() {
    std::cout << "[signaling] connected (answerer mode)\n";
  });
  impl_->signaling->SetOnMessage([this](const std::string& raw) {
    // WebSocket++ ASIO thread → post into GLib main
    post_main(impl_.get(), [this, raw]() {
      try {
        auto j = json::parse(raw);
        const std::string type = j.value("type", "");
        if (type == "offer") {
          const std::string sdp = j.value("sdp", "");
          std::cout << "[signaling] offer received (" << sdp.size() << " bytes) → applying remote desc\n";
          set_remote_description_async(impl_.get(), sdp, GST_WEBRTC_SDP_TYPE_OFFER, on_set_remote_desc_done);
        } else if (type == "candidate") {
          const std::string cand = j.value("candidate", "");
          const int mline = j.value("sdpMLineIndex", 0);
          if (!impl_->remote_desc_set) {
            std::cout << "[signaling] buffer ICE cand (mline=" << mline << ") until remote desc set\n";
            impl_->pending_cands.push_back({mline, cand});
          } else {
            // Apply immediately (we are on GLib main thread here)
            add_ice_candidate_safe(impl_->webrtc, mline, cand);
          }
        } else {
          // ignore others
        }
      } catch (const std::exception& e) {
        std::cerr << "[signaling] JSON parse failed: " << e.what() << "\n";
      }
    });
  });
  impl_->signaling->SetOnClose([]() {
    std::cout << "[signaling] closed\n";
  });
  if (!impl_->signaling->Connect()) {
    std::cerr << "Failed to connect signaling at: " << impl_->signaling_url << "\n";
    return false;
  }

  return true;
}

void GstWebRTCAdapter::PushFrame(const cv::Mat& bgr_frame) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->appsrc) return;
  if (bgr_frame.empty()) return;

  cv::Mat frame;
  if (bgr_frame.cols != impl_->width || bgr_frame.rows != impl_->height) {
    cv::resize(bgr_frame, frame, cv::Size(impl_->width, impl_->height));
  } else {
    frame = bgr_frame;
  }

  const gsize size = static_cast<gsize>(frame.total() * frame.elemSize());
  GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
  gst_buffer_fill(buffer, 0, frame.data, size);

  // Monotonic PTS/DURATION
  GstClockTime duration = gst_util_uint64_scale_int(GST_SECOND, 1, impl_->fps);
  GST_BUFFER_PTS(buffer) = impl_->running_pts;
  GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DURATION(buffer) = duration;
  impl_->running_pts += duration;

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(impl_->appsrc), buffer);
  if (ret != GST_FLOW_OK) {
    // Downstream not ready; drop with no spam
  }
}

void GstWebRTCAdapter::OnSignalingMessage(const std::string& /*msg*/) {
  // Unused: we now route all messages via SetOnMessage -> post_main
}
