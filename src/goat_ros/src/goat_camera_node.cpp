/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Publishes the GOAT O600 fisheye camera as a ROS image topic.
 *
 * The camera itself is owned by open_goat's CameraService, which asks the stock
 * EcoVacs `cameraProvider` daemon to stream and keeps that stream alive. The
 * daemon publishes NV12 frames into a double-buffered shared memory segment;
 * this node maps that segment read-only and republishes the frames, so it adds
 * one copy and no re-encoding. The segment supports any number of independent
 * readers, so the firmware's browser preview and this node coexist happily.
 *
 * The shared memory layout and the control protocol are reverse engineered -
 * see hardware_control_RE/camtest.py and the driver this node shares with the
 * firmware, open_goat_firmware/src/drivers/camera/camera_driver.hpp.
 *
 * Note that the project README tells you to disable cameraProvider.service as
 * part of switching off the EcoVacs stack. It has to be running for this node
 * to produce anything.
 */

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>

#include <drivers/camera/camera_driver.hpp>

using xbot::driver::camera::CameraDriver;
using xbot::driver::camera::FrameInfo;
using xbot::driver::camera::FrameReader;

int main(int argc, char **argv) {
  ros::init(argc, argv, "goat_camera");
  ros::NodeHandle nh;
  ros::NodeHandle param_nh("~");

  xbot::driver::camera::Config camera_config;
  std::string frame_id, camera_name, camera_info_url, encoding;
  int decimation = 0;
  double poll_rate = 0.0;
  bool manage_stream = false;
  param_nh.param<std::string>("shm_path", camera_config.shm_path, xbot::driver::camera::kDefaultShmPath);
  param_nh.param<std::string>("control_socket", camera_config.control_socket,
                              xbot::driver::camera::kDefaultControlSocket);
  param_nh.param<std::string>("client_name", camera_config.client_name, "openmower_ros");
  param_nh.param<std::string>("frame_id", frame_id, "camera_link");
  param_nh.param<std::string>("camera_name", camera_name, "goat_fisheye");
  param_nh.param<std::string>("camera_info_url", camera_info_url, "");
  // mono8 is the cheap path: it is the Y plane as-is, with no colour
  // conversion, and is what feature detection wants anyway.
  param_nh.param<std::string>("encoding", encoding, "bgr8");
  param_nh.param("decimation", decimation, 1);
  // The source runs at about 10 fps; poll well under that so we pick frames up
  // promptly without busy-waiting.
  param_nh.param("poll_rate", poll_rate, 200.0);
  // Off by default: open_goat's CameraService owns the stream, and two clients
  // toggling it over the control socket would fight. Set true to run this node
  // on its own, without the firmware.
  param_nh.param("manage_stream", manage_stream, false);

  if (encoding != sensor_msgs::image_encodings::MONO8 && encoding != sensor_msgs::image_encodings::BGR8) {
    ROS_ERROR_STREAM("Unsupported encoding '" << encoding << "', expected mono8 or bgr8");
    return 1;
  }
  if (decimation < 1) {
    ROS_WARN_STREAM("decimation of " << decimation << " is invalid, using 1");
    decimation = 1;
  }

  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher publisher = it.advertiseCamera("camera/image_raw", 1);
  camera_info_manager::CameraInfoManager info_manager(nh, camera_name, camera_info_url);
  if (!camera_info_url.empty() && !info_manager.isCalibrated()) {
    ROS_WARN_STREAM("Could not load camera calibration from " << camera_info_url);
  }

  CameraDriver camera(camera_config);
  if (manage_stream) {
    std::string reply;
    if (camera.EnableStream(reply)) {
      ROS_INFO_STREAM("Enabled camera stream: " << reply);
    } else {
      // Not fatal: the daemon may still be coming up, and we retry below.
      ROS_WARN_STREAM("Could not enable camera stream: " << reply);
    }
  }

  std::unique_ptr<FrameReader> reader;
  std::vector<uint8_t> nv12;
  FrameInfo info;
  ros::Time last_frame = ros::Time::now();
  ros::Time last_retry = last_frame;
  uint64_t frame_count = 0;

  ros::Rate rate(poll_rate);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();

    if (reader == nullptr) {
      reader = camera.OpenReader();
      if (reader == nullptr) {
        ROS_WARN_STREAM_THROTTLE(5, "Waiting for camera shared memory at "
                                        << camera_config.shm_path << ". Is cameraProvider.service running?");
        continue;
      }
    }

    if (!reader->GrabNV12(nv12, info)) {
      // No new frame. If the stream has gone quiet, the daemon may have been
      // restarted or dropped our client, so re-open - and, if we are the ones
      // holding the stream, re-enable it too.
      if ((ros::Time::now() - last_frame).toSec() > 5.0 && (ros::Time::now() - last_retry).toSec() > 5.0) {
        last_retry = ros::Time::now();
        if (manage_stream) {
          std::string reply;
          ROS_WARN_STREAM("No camera frames for 5s, re-enabling the stream");
          camera.EnableStream(reply);
        } else {
          ROS_WARN_STREAM_THROTTLE(30, "No camera frames for 5s. Is open_goat's CameraService running?");
        }
        reader.reset();
      }
      continue;
    }
    last_frame = ros::Time::now();

    // The frame header carries a timestamp, but its epoch and latency are
    // uncharacterised, so stamp on receipt rather than trusting it.
    const ros::Time stamp = last_frame;
    const uint32_t width = info.width;
    const uint32_t height = info.height;

    cv::Mat image;
    if (encoding == sensor_msgs::image_encodings::MONO8) {
      // The Y plane is the first width*height bytes, already 8-bit grey.
      image = cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC1, nv12.data());
    } else {
      const cv::Mat wrapped(static_cast<int>(height * 3 / 2), static_cast<int>(width), CV_8UC1, nv12.data());
      cv::cvtColor(wrapped, image, cv::COLOR_YUV2BGR_NV12);
    }

    if (decimation > 1) {
      cv::Mat resized;
      cv::resize(image, resized, cv::Size(static_cast<int>(width) / decimation, static_cast<int>(height) / decimation),
                 0, 0, cv::INTER_AREA);
      image = resized;
    }

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    header.seq = static_cast<uint32_t>(frame_count);

    // toImageMsg() copies, which matters: for mono8 `image` is only a header
    // over our nv12 buffer, and that buffer is reused on the next grab.
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();

    sensor_msgs::CameraInfo camera_info = info_manager.getCameraInfo();
    camera_info.header = header;
    if (camera_info.width != image_msg->width || camera_info.height != image_msg->height) {
      if (info_manager.isCalibrated() && camera_info.width == width && camera_info.height == height) {
        // Calibrated at the source resolution but we publish decimated, so
        // scale the intrinsics to match. Distortion coefficients are
        // normalised and the rectification rotation is scale-free.
        const double scale = 1.0 / static_cast<double>(decimation);
        for (const int i : {0, 2, 4, 5}) {
          camera_info.K[i] *= scale;
        }
        for (const int i : {0, 2, 3, 5, 6, 7}) {
          camera_info.P[i] *= scale;
        }
      } else if (info_manager.isCalibrated()) {
        ROS_WARN_STREAM_THROTTLE(60, "Calibration is for " << camera_info.width << "x" << camera_info.height
                                                           << " but publishing " << image_msg->width << "x"
                                                           << image_msg->height << "; intrinsics will not match");
      }
      camera_info.width = image_msg->width;
      camera_info.height = image_msg->height;
    }

    publisher.publish(*image_msg, camera_info);

    if (frame_count == 0) {
      ROS_INFO_STREAM("Publishing " << image_msg->width << "x" << image_msg->height << " " << encoding << " (source "
                                    << width << "x" << height << ", decimation " << decimation << ")");
    }
    ++frame_count;
  }

  if (manage_stream) {
    std::string reply;
    if (camera.DisableStream(reply)) {
      ROS_INFO_STREAM("Disabled camera stream: " << reply);
    }
  }
  return 0;
}
