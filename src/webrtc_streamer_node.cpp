#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include "webrtc_gst_cpp/gst_webrtc_adapter.h"

#include <memory>

int main(int argc, char** argv) {
  ros::init(argc, argv, "webrtc_streamer_node");
  ros::NodeHandle nh("~");

  std::string signaling_url;
  nh.param<std::string>("signaling_url", signaling_url, "ws://localhost:8080");
  int width, height, fps;
  nh.param("width", width, 320);
  nh.param("height", height, 240);
  nh.param("fps", fps, 15);

  auto adapter = std::make_shared<GstWebRTCAdapter>(signaling_url, width, height, fps);
  if (!adapter->Start()) {
    ROS_ERROR("Failed to start GstWebRTCAdapter");
    return 1;
  }

  image_transport::ImageTransport it(nh);
  cv_bridge::CvImagePtr last_cv;

  auto cb = [adapter, width, height](const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      cv::Mat frame = cv_ptr->image;
      if (frame.cols != width || frame.rows != height) {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(width, height));
        adapter->PushFrame(resized);
      } else {
        adapter->PushFrame(frame);
      }
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
      ROS_ERROR("Exception in image callback: %s", e.what());
    }
  };

  image_transport::Subscriber sub = it.subscribe("/main_camera/image_raw", 1, cb);

  ros::spin();
  return 0;
}