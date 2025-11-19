#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

using json = nlohmann::json;
using websocket_client = websocketpp::client<websocketpp::config::asio_client>;

enum class Role { Offer, Answer };

struct AppConfig {
  std::string signaling_url = "ws://localhost:8080";
  std::string stun_server = "stun:stun.l.google.com:19302";
  int width = 640;
  int height = 480;
  int fps = 30;
  std::string encoder = "vp8"; 		// h264 | h264-sw | vp8
  std::string source = "libcamera"; // auto | libcamera | v4l2
  Role role = Role::Answer;     	// default Answer for Node client
};

struct AppState {
  GMainLoop* loop = nullptr;
  GMainContext* context = nullptr;
  GstElement* pipeline = nullptr;
  GstElement* webrtcbin = nullptr;

  websocket_client ws_client;
  websocketpp::connection_hdl ws_hdl;
  std::atomic<bool> ws_open{false};
  std::mutex ws_mu;

  AppConfig cfg;
};

static void post_to_main(AppState* app, std::function<void()> fn) {
  if (!app || !app->context) return;
  auto trampoline = [](gpointer data) -> gboolean {
    std::unique_ptr<std::function<void()>> fn_ptr(static_cast<std::function<void()>*>(data));
    (*fn_ptr)();
    return G_SOURCE_REMOVE;
  };
  g_main_context_invoke(app->context, trampoline, new std::function<void()>(std::move(fn)));
}

static void ws_send(AppState* app, const json& j) {
  std::lock_guard<std::mutex> lk(app->ws_mu);
  if (!app->ws_open.load()) return;
  websocketpp::lib::error_code ec;
  app->ws_client.send(app->ws_hdl, j.dump(), websocketpp::frame::opcode::text, ec);
  if (ec) std::cerr << "WS send error: " << ec.message() << std::endl;
}

static const char* ice_state_str(GstWebRTCICEConnectionState s) {
  switch (s) {
    case GST_WEBRTC_ICE_CONNECTION_STATE_NEW: return "NEW";
    case GST_WEBRTC_ICE_CONNECTION_STATE_CHECKING: return "CHECKING";
    case GST_WEBRTC_ICE_CONNECTION_STATE_CONNECTED: return "CONNECTED";
    case GST_WEBRTC_ICE_CONNECTION_STATE_COMPLETED: return "COMPLETED";
    case GST_WEBRTC_ICE_CONNECTION_STATE_FAILED: return "FAILED";
    case GST_WEBRTC_ICE_CONNECTION_STATE_DISCONNECTED: return "DISCONNECTED";
    case GST_WEBRTC_ICE_CONNECTION_STATE_CLOSED: return "CLOSED";
    default: return "UNKNOWN";
  }
}
static const char* pc_state_str(GstWebRTCPeerConnectionState s) {
  switch (s) {
    case GST_WEBRTC_PEER_CONNECTION_STATE_NEW: return "NEW";
    case GST_WEBRTC_PEER_CONNECTION_STATE_CONNECTING: return "CONNECTING";
    case GST_WEBRTC_PEER_CONNECTION_STATE_CONNECTED: return "CONNECTED";
    case GST_WEBRTC_PEER_CONNECTION_STATE_DISCONNECTED: return "DISCONNECTED";
    case GST_WEBRTC_PEER_CONNECTION_STATE_FAILED: return "FAILED";
    case GST_WEBRTC_PEER_CONNECTION_STATE_CLOSED: return "CLOSED";
    default: return "UNKNOWN";
  }
}
static const char* sig_state_str(GstWebRTCSignalingState s) {
  switch (s) {
    case GST_WEBRTC_SIGNALING_STATE_STABLE: return "STABLE";
    case GST_WEBRTC_SIGNALING_STATE_HAVE_LOCAL_OFFER: return "HAVE_LOCAL_OFFER";
    case GST_WEBRTC_SIGNALING_STATE_HAVE_REMOTE_OFFER: return "HAVE_REMOTE_OFFER";
    case GST_WEBRTC_SIGNALING_STATE_HAVE_LOCAL_PRANSWER: return "HAVE_LOCAL_PRANSWER";
    case GST_WEBRTC_SIGNALING_STATE_HAVE_REMOTE_PRANSWER: return "HAVE_REMOTE_PRANSWER";
    case GST_WEBRTC_SIGNALING_STATE_CLOSED: return "CLOSED";
    default: return "UNKNOWN";
  }
}

static gboolean on_bus_message(GstBus* /*bus*/, GstMessage* msg, gpointer /*user_data*/) {
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError* err = nullptr; gchar* dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      std::cerr << "[GStreamer] ERROR: " << (err?err->message:"") << " debug: " << (dbg?dbg:"") << std::endl;
      if (err) g_error_free(err);
      if (dbg) g_free(dbg);
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError* err = nullptr; gchar* dbg = nullptr;
      gst_message_parse_warning(msg, &err, &dbg);
      std::cerr << "[GStreamer] WARNING: " << (err?err->message:"") << " debug: " << (dbg?dbg:"") << std::endl;
      if (err) g_error_free(err);
      if (dbg) g_free(dbg);
      break;
    }
    case GST_MESSAGE_EOS:
      std::cout << "[GStreamer] EOS" << std::endl;
      break;
    default:
      break;
  }
  return TRUE;
}

static void on_notify_ice_conn(GObject*, GParamSpec*, gpointer user_data) {
  auto* app = static_cast<AppState*>(user_data);
  GstWebRTCICEConnectionState st;
  g_object_get(app->webrtcbin, "ice-connection-state", &st, nullptr);
  std::cout << "[WebRTC] ICE state: " << ice_state_str(st) << std::endl;
}
static void on_notify_conn_state(GObject*, GParamSpec*, gpointer user_data) {
  auto* app = static_cast<AppState*>(user_data);
  GstWebRTCPeerConnectionState st;
  g_object_get(app->webrtcbin, "connection-state", &st, nullptr);
  std::cout << "[WebRTC] PeerConnection state: " << pc_state_str(st) << std::endl;
}
static void on_notify_sig_state(GObject*, GParamSpec*, gpointer user_data) {
  auto* app = static_cast<AppState*>(user_data);
  GstWebRTCSignalingState st;
  g_object_get(app->webrtcbin, "signaling-state", &st, nullptr);
  std::cout << "[WebRTC] Signaling state: " << sig_state_str(st) << std::endl;
}

static void on_ice_candidate(GstElement* /*webrtc*/, guint mlineindex, gchar* candidate, gpointer user_data) {
  auto* app = static_cast<AppState*>(user_data);
  if (app->cfg.role == Role::Answer) {
    json j = {{"type","candidate"}, {"candidate", candidate}, {"sdpMLineIndex",(int)mlineindex}};
    ws_send(app, j);
  } else {
    json j = {{"type","ice"}, {"ice", {{"candidate", candidate}, {"sdpMLineIndex",(int)mlineindex}}}};
    ws_send(app, j);
  }
}

static void handle_remote_answer(AppState* app, const std::string& sdp_text) {
  GstSDPMessage* sdp = nullptr;
  if (gst_sdp_message_new(&sdp) != GST_SDP_OK) return;
  if (gst_sdp_message_parse_buffer(reinterpret_cast<const guint8*>(sdp_text.data()),
                                   sdp_text.size(), sdp) != GST_SDP_OK) {
    gst_sdp_message_free(sdp);
    return;
  }
  GstWebRTCSessionDescription* answer =
      gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp);
  g_signal_emit_by_name(app->webrtcbin, "set-remote-description", answer, nullptr);
  gst_webrtc_session_description_free(answer);
  std::cout << "[WebRTC] Remote SDP answer set\n";
}

static void on_answer_created(GstPromise* promise, gpointer user_data) {
  auto* app = static_cast<AppState*>(user_data);
  GstWebRTCSessionDescription* answer = nullptr;
  gst_promise_wait(promise);
  auto reply = gst_promise_get_reply(promise);
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
  gst_promise_unref(promise);
  if (!answer) { std::cerr << "create-answer returned null\n"; return; }

  g_signal_emit_by_name(app->webrtcbin, "set-local-description", answer, nullptr);
  gchar* sdp_text = gst_sdp_message_as_text(answer->sdp);
  json j = {{"type","answer"}, {"sdp", sdp_text ? sdp_text : ""}};
  if (sdp_text) g_free(sdp_text);
  gst_webrtc_session_description_free(answer);
  ws_send(app, j);
  std::cout << "[WebRTC] Answer created and sent\n";
}

static void handle_remote_offer_and_create_answer(AppState* app, const std::string& sdp_text) {
  GstSDPMessage* sdp = nullptr;
  if (gst_sdp_message_new(&sdp) != GST_SDP_OK) return;
  if (gst_sdp_message_parse_buffer(reinterpret_cast<const guint8*>(sdp_text.data()),
                                   sdp_text.size(), sdp) != GST_SDP_OK) {
    gst_sdp_message_free(sdp); return;
  }
  GstWebRTCSessionDescription* offer =
      gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp);
  g_signal_emit_by_name(app->webrtcbin, "set-remote-description", offer, nullptr);
  gst_webrtc_session_description_free(offer);

  GstPromise* pr = gst_promise_new_with_change_func(on_answer_created, app, nullptr);
  g_signal_emit_by_name(app->webrtcbin, "create-answer", nullptr, pr);
}

static void handle_remote_candidate(AppState* app, int mline, const std::string& cand) {
  g_signal_emit_by_name(app->webrtcbin, "add-ice-candidate", mline, cand.c_str());
}

static GstElement* make_or_fail(const char* factory, const char* name) {
  GstElement* e = gst_element_factory_make(factory, name);
  if (!e) std::cerr << "Failed to create element: " << factory << " (" << (name?name:"") << ")\n";
  return e;
}

static bool build_pipeline(const AppConfig& cfg, AppState* app) {
  app->pipeline = gst_pipeline_new("webrtc-pipeline");
  app->webrtcbin = gst_element_factory_make("webrtcbin", "webrtcbin");
  if (!app->pipeline || !app->webrtcbin) {
    std::cerr << "Failed to create pipeline/webrtcbin\n";
    return false;
  }
  if (!cfg.stun_server.empty())
    g_object_set(app->webrtcbin, "stun-server", cfg.stun_server.c_str(), nullptr);

  // Bus watch
  auto* bus = gst_element_get_bus(app->pipeline);
  gst_bus_add_signal_watch(bus);
  g_signal_connect(bus, "message", G_CALLBACK(on_bus_message), app);
  gst_object_unref(bus);

  // Observe webrtc states
  g_signal_connect(app->webrtcbin, "notify::ice-connection-state", G_CALLBACK(on_notify_ice_conn), app);
  g_signal_connect(app->webrtcbin, "notify::connection-state", G_CALLBACK(on_notify_conn_state), app);
  g_signal_connect(app->webrtcbin, "notify::signaling-state", G_CALLBACK(on_notify_sig_state), app);

  // Elements
  GstElement *src = nullptr, *caps0 = nullptr, *convert = nullptr, *caps_fmt = nullptr, *rate = nullptr;
  GstElement *queue_enc = nullptr, *encoder = nullptr, *parse = nullptr, *pay = nullptr;

  // Source selection
  std::string src_choice = cfg.source;
  if (src_choice == "auto") {
    src = gst_element_factory_make("libcamerasrc", "src");
    if (src) src_choice = "libcamera";
    else { src = gst_element_factory_make("v4l2src", "src"); src_choice = "v4l2"; }
  } else if (src_choice == "libcamera") {
    src = gst_element_factory_make("libcamerasrc", "src");
  } else if (src_choice == "v4l2") {
    src = gst_element_factory_make("v4l2src", "src");
  }
  if (!src) { std::cerr << "Failed to create source: " << cfg.source << "\n"; return false; }

  caps0     = make_or_fail("capsfilter", "caps0");
  convert   = make_or_fail("videoconvert", "convert");
  caps_fmt  = make_or_fail("capsfilter", "caps_fmt");
  rate      = make_or_fail("videorate", "rate");
  queue_enc = make_or_fail("queue", "qenc");
  if (!caps0 || !convert || !caps_fmt || !rate || !queue_enc) return false;

  // WxH@FPS caps
  GstCaps* caps = gst_caps_new_simple("video/x-raw",
    "width", G_TYPE_INT, cfg.width,
    "height", G_TYPE_INT, cfg.height,
    "framerate", GST_TYPE_FRACTION, cfg.fps, 1,
    nullptr);
  g_object_set(caps0, "caps", caps, nullptr);
  gst_caps_unref(caps);

  // Encoder selection
  std::string enc = cfg.encoder;
  std::string prefer_fmt;
  bool force_sw = (enc == "h264-sw");
  if (enc == "h264" || force_sw) {
    if (!force_sw) {
      encoder = gst_element_factory_make("v4l2h264enc", "enc");
    }
    if (encoder) {
      prefer_fmt = "NV12"; // HW prefers NV12
    } else {
      encoder = gst_element_factory_make("x264enc", "enc");
      prefer_fmt = "I420";
      if (encoder) {
        g_object_set(encoder,
          "tune", 0x00000004 /* zerolatency */,
          "speed-preset", 1 /* ultrafast */,
          "byte-stream", TRUE,
          "key-int-max", cfg.fps,
          "bframes", 0,
          nullptr);
      }
    }
    parse = gst_element_factory_make("h264parse", "parse");
    if (parse) g_object_set(parse, "alignment", 1 /* au */, nullptr);
    pay   = gst_element_factory_make("rtph264pay", "pay");
    if (pay) {
      g_object_set(pay,
        "pt", 96,
        "config-interval", -1, // send SPS/PPS every keyframe
        nullptr);
    }
  } else if (enc == "vp8") {
    encoder = gst_element_factory_make("vp8enc", "enc");
    prefer_fmt = "I420";
    if (encoder) {
      g_object_set(encoder,
        "deadline", (gint64)1,
        "error-resilient", 1,
        "keyframe-max-dist", cfg.fps,
        nullptr);
    }
    pay = gst_element_factory_make("rtpvp8pay", "pay");
    if (pay) g_object_set(pay, "pt", 96, nullptr);
  } else {
    std::cerr << "Unknown encoder: " << enc << " (use h264 | h264-sw | vp8)\n";
    return false;
  }
  if (!encoder || !pay) {
    std::cerr << "Failed to create encoder/payloader\n";
    return false;
  }

  // Caps format before encoder
  if (!prefer_fmt.empty()) {
    GstCaps* fmt_caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, prefer_fmt.c_str(),
      nullptr);
    g_object_set(caps_fmt, "caps", fmt_caps, nullptr);
    gst_caps_unref(fmt_caps);
  }

  // Add elements safely
  gst_bin_add(GST_BIN(app->pipeline), app->webrtcbin);
  gst_bin_add(GST_BIN(app->pipeline), src);
  gst_bin_add(GST_BIN(app->pipeline), caps0);
  gst_bin_add(GST_BIN(app->pipeline), convert);
  gst_bin_add(GST_BIN(app->pipeline), caps_fmt);
  gst_bin_add(GST_BIN(app->pipeline), rate);
  gst_bin_add(GST_BIN(app->pipeline), queue_enc);
  gst_bin_add(GST_BIN(app->pipeline), encoder);
  if (parse) gst_bin_add(GST_BIN(app->pipeline), parse);
  gst_bin_add(GST_BIN(app->pipeline), pay);

  // Link pre-encoder chain
  if (!gst_element_link_many(src, caps0, convert, caps_fmt, rate, queue_enc, encoder, nullptr)) {
    std::cerr << "Failed to link pre-encoder chain\n"; return false;
  }
  // Link encoder -> [parse] -> pay
  if (parse) {
    if (!gst_element_link_many(encoder, parse, pay, nullptr)) {
      std::cerr << "Failed to link encoder->parse->pay\n"; return false;
    }
  } else {
    if (!gst_element_link(encoder, pay)) {
      std::cerr << "Failed to link encoder->pay\n"; return false;
    }
  }

  // Debug: show pay src caps
  {
    GstPad* psrc = gst_element_get_static_pad(pay, "src");
    if (psrc) {
      GstCaps* sc = gst_pad_get_current_caps(psrc);
      if (sc) {
        gchar* s = gst_caps_to_string(sc);
        std::cout << "[GStreamer] pay src caps: " << (s?s:"") << std::endl;
        if (s) g_free(s);
        gst_caps_unref(sc);
      }
      gst_object_unref(psrc);
    }
  }

  // Link pay -> webrtcbin (pad template "sink_%u"; remove deprecated fallback)
  GstPad* srcpad = gst_element_get_static_pad(pay, "src");
  GstPad* sinkpad = gst_element_request_pad_simple(app->webrtcbin, "sink_%u");
  if (!srcpad || !sinkpad) {
    std::cerr << "Failed to get RTP pads for webrtcbin (srcpad=" << (srcpad?"ok":"null")
              << ", sinkpad=" << (sinkpad?"ok":"null") << ")\n";
    if (srcpad) gst_object_unref(srcpad);
    if (sinkpad) gst_object_unref(sinkpad);
    return false;
  }
  if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
    std::cerr << "Failed to link pay -> webrtcbin\n";
    gst_object_unref(srcpad);
    gst_object_unref(sinkpad);
    return false;
  }
  gst_object_unref(srcpad);
  gst_object_unref(sinkpad);

  // WebRTC callbacks and state observers
  g_signal_connect(app->webrtcbin, "on-ice-candidate", G_CALLBACK(on_ice_candidate), app);
  g_signal_connect(app->webrtcbin, "notify::ice-connection-state", G_CALLBACK(on_notify_ice_conn), app);
  g_signal_connect(app->webrtcbin, "notify::connection-state", G_CALLBACK(on_notify_conn_state), app);
  g_signal_connect(app->webrtcbin, "notify::signaling-state", G_CALLBACK(on_notify_sig_state), app);

  return true;
}

static void ws_thread_func(AppState* app) {
  try {
    app->ws_client.clear_access_channels(websocketpp::log::alevel::all);
    app->ws_client.clear_error_channels(websocketpp::log::elevel::all);
    app->ws_client.init_asio();

    app->ws_client.set_open_handler([app](websocketpp::connection_hdl hdl){
      app->ws_hdl = hdl;
      app->ws_open.store(true);
      std::cout << "WS connected\n";
    });

    app->ws_client.set_message_handler([app](websocketpp::connection_hdl, websocket_client::message_ptr msg){
      try {
        auto j = json::parse(msg->get_payload());
        std::string type = j.value("type", "");

        if (app->cfg.role == Role::Answer) {
          if (type == "offer") {
            std::string sdp = j.value("sdp", "");
            post_to_main(app, [app, sdp]() { handle_remote_offer_and_create_answer(app, sdp); });
          } else if (type == "candidate") {
            int mline = j.value("sdpMLineIndex", 0);
            std::string cand = j.value("candidate", "");
            post_to_main(app, [app, mline, cand]() { handle_remote_candidate(app, mline, cand); });
          } else if (type == "ready") {
            std::cout << "WS: peer ready\n";
          }
        } else {
          // Offer role (kept for completeness)
          if (type == "answer") {
            std::string sdp = j.value("sdp", "");
            post_to_main(app, [app, sdp]() { handle_remote_answer(app, sdp); });
          } else if (type == "ice") {
            auto ice = j["ice"];
            int mline = ice.value("sdpMLineIndex", 0);
            std::string cand = ice.value("candidate", "");
            post_to_main(app, [app, mline, cand]() { handle_remote_candidate(app, mline, cand); });
          }
        }
      } catch (const std::exception& e) {
        std::cerr << "WS message parse error: " << e.what() << "\n";
      }
    });

    app->ws_client.set_close_handler([app](websocketpp::connection_hdl){
      app->ws_open.store(false);
      std::cout << "WS closed\n";
      if (app->loop) g_main_loop_quit(app->loop);
    });

    websocketpp::lib::error_code ec;
    auto con = app->ws_client.get_connection(app->cfg.signaling_url, ec);
    if (ec) {
      std::cerr << "WS get_connection error: " << ec.message() << "\n";
      if (app->loop) g_main_loop_quit(app->loop);
      return;
    }

    app->ws_client.connect(con);
    app->ws_client.run();
  } catch (const std::exception& e) {
    std::cerr << "WS exception: " << e.what() << "\n";
    if (app->loop) g_main_loop_quit(app->loop);
  }
}

static void usage() {
  std::cout <<
    "Usage: webrtc_rpi [--signaling-url ws://host:port] [--stun stun:host:port]\n"
    "                  [--width N] [--height N] [--fps N]\n"
    "                  [--encoder h264|h264-sw|vp8] [--source auto|libcamera|v4l2]\n"
    "                  [--role answer|offer]\n";
}

int main(int argc, char* argv[]) {
  AppConfig cfg;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](std::string& out){ if (i+1<argc) out = argv[++i]; };
    auto nexti = [&](int& out){ if (i+1<argc) out = std::stoi(argv[++i]); };
    if (a == "--signaling-url") next(cfg.signaling_url);
    else if (a == "--stun") next(cfg.stun_server);
    else if (a == "--width") nexti(cfg.width);
    else if (a == "--height") nexti(cfg.height);
    else if (a == "--fps") nexti(cfg.fps);
    else if (a == "--encoder") next(cfg.encoder);
    else if (a == "--source") next(cfg.source);
    else if (a == "--role") { std::string r; next(r); cfg.role = (r=="offer"? Role::Offer : Role::Answer); }
    else if (a == "-h" || a == "--help") { usage(); return 0; }
  }

  gst_init(&argc, &argv);

  AppState app;
  app.cfg = cfg;
  app.loop = g_main_loop_new(nullptr, FALSE);
  app.context = g_main_loop_get_context(app.loop);

  if (!build_pipeline(cfg, &app)) return 1;

  std::thread ws_thr(ws_thread_func, &app);

  gst_element_set_state(app.pipeline, GST_STATE_PLAYING);

  g_main_loop_run(app.loop);

  gst_element_set_state(app.pipeline, GST_STATE_NULL);
  gst_object_unref(app.pipeline);
  g_main_loop_unref(app.loop);

  if (ws_thr.joinable()) ws_thr.join();
  return 0;
}