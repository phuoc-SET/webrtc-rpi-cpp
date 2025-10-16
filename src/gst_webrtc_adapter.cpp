#include "webrtc_gst_cpp/gst_webrtc_adapter.h"
#include "webrtc_gst_cpp/signaling_client.h"

// Silence unstable API warning (also added in CMake)
#ifndef GST_USE_UNSTABLE_API
#define GST_USE_UNSTABLE_API
#endif

#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/app/gstappsrc.h>
#include <gst/sdp/gstsdpmessage.h>

#include <nlohmann/json.hpp>
#include <thread>
#include <iostream>
#include <cstring>
#include <vector>

using json = nlohmann::json;

// -------- Safe JSON helpers --------
static std::string json_get_string_or(const json& j, const char* key, const std::string& defval) {
  try {
    if (!j.contains(key) || j.at(key).is_null()) return defval;
    if (j.at(key).is_string()) return j.at(key).get<std::string>();
    // Allow numbers to be coerced to string
    if (j.at(key).is_number_integer()) return std::to_string(j.at(key).get<long long>());
    if (j.at(key).is_number_unsigned()) return std::to_string(j.at(key).get<unsigned long long>());
    if (j.at(key).is_number_float()) return std::to_string(j.at(key).get<double>());
  } catch (...) {}
  return defval;
}
static int json_get_int_or(const json& j, const char* key, int defval) {
  try {
    if (!j.contains(key) || j.at(key).is_null()) return defval;
    if (j.at(key).is_number_integer()) return j.at(key).get<int>();
    if (j.at(key).is_string()) return std::stoi(j.at(key).get<std::string>());
  } catch (...) {}
  return defval;
}

struct GstWebRTCAdapter::Impl {
  std::string signaling_url;
  int width;
  int height;
  int fps;

  GMainLoop* loop = nullptr;
  GstElement* pipeline = nullptr;
  GstElement* appsrc = nullptr;
  GstElement* webrtc = nullptr;

  std::unique_ptr<SignalingClient> signaling;
  std::thread glib_thread;
  std::mutex mutex;

  // Simple running PTS based on fps
  GstClockTime running_pts = 0;

  // Negotiation/ICE state
  bool remote_desc_set = false;
  struct PendingCand { std::string mid; int mline; std::string cand; };
  std::vector<PendingCand> pending_cands;

  Impl(const std::string& url, int w, int h, int f)
    : signaling_url(url), width(w), height(h), fps(f) {}
};

// Forward decls (ANSWERER mode: we do not create local offer)
static void on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data);
static void on_answer_created(GstPromise *promise, gpointer user_data);
static void on_set_remote_desc_done(GstPromise *promise, gpointer user_data);
static gboolean create_answer_idle_cb(gpointer data);

// Helpers
static void add_ice_candidate_safe(GstElement* webrtc, const std::string& mid, int mline, const std::string& cand) {
  if (!webrtc) return;
  if (cand.empty()) return;
  const char* mid_c = (mid.empty() ? "0" : mid.c_str()); // never pass NULL to GLib
  g_signal_emit_by_name(webrtc, "add-ice-candidate", mid_c, mline, cand.c_str());
}

static void flush_pending_candidates(GstWebRTCAdapter::Impl* self) {
  if (!self || !self->webrtc) return;
  for (const auto& pc : self->pending_cands) {
    add_ice_candidate_safe(self->webrtc, pc.mid, pc.mline, pc.cand);
  }
  self->pending_cands.clear();
}

static void set_remote_description_async(GstWebRTCAdapter::Impl* self,
                                         const std::string& sdp_text,
                                         GstWebRTCSDPType type,
                                         GstPromiseChangeFunc done_cb) {
  if (!self || !self->webrtc) return;
  if (sdp_text.empty()) {
    std::cerr << "Remote SDP is empty\n";
    return;
  }
  GstSDPMessage *sdp = nullptr;
  if (gst_sdp_message_new(&sdp) != GST_SDP_OK) {
    std::cerr << "Failed to alloc SDP message\n";
    return;
  }
  const GstSDPResult r = gst_sdp_message_parse_buffer(
      reinterpret_cast<const guint8*>(sdp_text.data()),
      sdp_text.size(), sdp);
  if (r != GST_SDP_OK) {
    std::cerr << "Failed to parse SDP text, code=" << r << "\n";
    gst_sdp_message_free(sdp);
    return;
  }
  GstWebRTCSessionDescription *desc = gst_webrtc_session_description_new(type, sdp);
  GstPromise *p = gst_promise_new_with_change_func(done_cb, self, nullptr);
  g_signal_emit_by_name(self->webrtc, "set-remote-description", desc, p);
  gst_webrtc_session_description_free(desc);
}

// Callbacks
static void on_ice_candidate(GstElement * /*webrtc*/, guint mlineindex, gchar *candidate, gpointer user_data) {
  auto *self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  if (!candidate || !*candidate) {
    // Ignore null/empty candidates to avoid GLib warnings
    return;
  }
  json j;
  j["type"] = "candidate";
  j["candidate"] = std::string(candidate);
  j["sdpMLineIndex"] = static_cast<int>(mlineindex);
  j["sdpMid"] = "0"; // single m-line, default "0"
  if (self->signaling) self->signaling->Send(j.dump());
}

static void on_answer_created(GstPromise *promise, gpointer user_data) {
  auto *self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  const GstStructure *reply = gst_promise_get_reply(promise);
  if (!reply) {
    std::cerr << "Answer promise has no reply structure\n";
    gst_promise_unref(promise);
    return;
  }

  GstWebRTCSessionDescription *answer = nullptr;
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
  if (!answer) {
    std::cerr << "Failed to get answer from promise\n";
    gst_promise_unref(promise);
    return;
  }

  // Set local description
  g_signal_emit_by_name(self->webrtc, "set-local-description", answer, nullptr);

  // Send SDP answer
  gchar *sdp_str = gst_sdp_message_as_text(answer->sdp);
  json j;
  j["type"] = "answer";
  j["sdp"] = std::string(sdp_str ? sdp_str : "");
  if (sdp_str) g_free(sdp_str);

  if (self->signaling) self->signaling->Send(j.dump());

  gst_webrtc_session_description_free(answer);
  gst_promise_unref(promise);
}

static gboolean create_answer_idle_cb(gpointer data) {
  auto* self = static_cast<GstWebRTCAdapter::Impl*>(data);
  if (!self || !self->webrtc) return G_SOURCE_REMOVE;
  // Now remote description is set; create the answer
  GstPromise *promise = gst_promise_new_with_change_func(on_answer_created, self, nullptr);
  g_signal_emit_by_name(self->webrtc, "create-answer", nullptr, promise);
  return G_SOURCE_REMOVE;
}

static void on_set_remote_desc_done(GstPromise *promise, gpointer user_data) {
  auto *self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  // Remote desc is set; flush buffered ICE and schedule answer creation
  self->remote_desc_set = true;
  flush_pending_candidates(self);
  gst_promise_unref(promise);
  // Defer create-answer on main loop to ensure state settled
  g_idle_add(create_answer_idle_cb, self);
}

// Public API
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

  // appsrc -> videoconvert -> vp8enc -> rtpvp8pay -> caps -> webrtcbin
  std::string pipeline_str =
    "appsrc name=src is-live=true block=true format=time "
    "caps=video/x-raw,format=BGR,width=" + std::to_string(impl_->width) +
    ",height=" + std::to_string(impl_->height) + ",framerate=" + std::to_string(impl_->fps) + "/1 "
    "! videoconvert ! queue ! vp8enc deadline=1 cpu-used=5 error-resilient=1 keyframe-max-dist=60 "
    "! rtpvp8pay pt=96 "
    "! application/x-rtp,media=video,encoding-name=VP8,payload=96 "
    "! webrtcbin name=webrtc";

  GError *error = nullptr;
  impl_->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
  if (!impl_->pipeline) {
    std::cerr << "Failed to create pipeline: " << (error ? error->message : "unknown") << std::endl;
    if (error) g_error_free(error);
    return false;
  }

  impl_->appsrc = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "src");
  impl_->webrtc = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "webrtc");

  if (!impl_->appsrc || !impl_->webrtc) {
    std::cerr << "Failed to get appsrc or webrtc element from pipeline\n";
    return false;
  }

  // In ANSWERER mode: do NOT create a local offer; only respond to remote offer.
  // LAN thường không cần STUN; nếu bật, đảm bảo chuỗi hợp lệ "stun://host:port".
  // g_object_set(impl_->webrtc, "stun-server", "stun://stun.l.google.com:19302", nullptr);

  // Only ICE callback
  g_signal_connect(impl_->webrtc, "on-ice-candidate", G_CALLBACK(on_ice_candidate), impl_.get());

  gst_element_set_state(impl_->pipeline, GST_STATE_PLAYING);

  // Start GLib main loop
  impl_->glib_thread = std::thread([this]() {
    g_main_loop_run(impl_->loop);
  });

  // Setup signaling
  impl_->signaling = std::make_unique<SignalingClient>(impl_->signaling_url);
  impl_->signaling->SetOnOpen([this]() {
    std::cout << "[signaling] connected (answerer mode)\n";
  });
  impl_->signaling->SetOnMessage([this](const std::string& msg) {
    this->OnSignalingMessage(msg);
  });
  impl_->signaling->SetOnClose([]() {
    std::cout << "[signaling] closed\n";
  });

  if (!impl_->signaling->Connect()) {
    std::cerr << "Failed to connect signaling server\n";
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

  // Simple increasing PTS with fps (monotonic)
  GstClockTime duration = gst_util_uint64_scale_int(GST_SECOND, 1, impl_->fps);
  GST_BUFFER_PTS(buffer) = impl_->running_pts;
  GST_BUFFER_DTS(buffer) = impl_->running_pts;
  GST_BUFFER_DURATION(buffer) = duration;
  impl_->running_pts += duration;

  GstFlowReturn ret = GST_FLOW_OK;
  g_signal_emit_by_name(impl_->appsrc, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);
  if (ret != GST_FLOW_OK) {
    std::cerr << "appsrc push-buffer returned " << ret << std::endl;
  }
}

void GstWebRTCAdapter::OnSignalingMessage(const std::string& msg) {
  try {
    json j = json::parse(msg);
    const std::string type = json_get_string_or(j, "type", "");

    if (type == "offer") {
      const std::string sdp = json_get_string_or(j, "sdp", "");
      // Asynchronously set remote description; when done -> create and send answer
      set_remote_description_async(impl_.get(), sdp, GST_WEBRTC_SDP_TYPE_OFFER, on_set_remote_desc_done);

    } else if (type == "candidate") {
      const std::string cand = json_get_string_or(j, "candidate", "");
      if (cand.empty()) return; // avoid passing NULL/empty to GLib
      const int mline = json_get_int_or(j, "sdpMLineIndex", 0);
      std::string mid = json_get_string_or(j, "sdpMid", "0");
      if (mid.empty()) mid = "0"; // never pass NULL/empty

      if (impl_->remote_desc_set) {
        add_ice_candidate_safe(impl_->webrtc, mid, mline, cand);
      } else {
        // Buffer until remote description is applied
        impl_->pending_cands.push_back({mid, mline, cand});
      }

    } else if (type == "answer") {
      // Not expected in strict answerer mode, but handle gracefully if ever used as caller
      const std::string sdp = json_get_string_or(j, "sdp", "");
      set_remote_description_async(impl_.get(), sdp, GST_WEBRTC_SDP_TYPE_ANSWER, on_set_remote_desc_done);

    } else {
      // ignore (e.g., "ready")
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse signaling message: " << e.what() << std::endl;
  }
}