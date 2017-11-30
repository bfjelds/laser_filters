/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef ROS2
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

typedef sensor_msgs::LaserScan LaserScane;

// TF
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

typedef tf::TransformException TransformException;
typedef tf::TransformListener TransformListener;

#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/Laser_Scan.hpp>

typedef sensor_msgs::msg::LaserScan LaserScan;

// TF
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

typedef tf2::TransformException TransformException;
typedef tf2_ros::TransformListener TransformListener;

#include "message_filters/subscriber.h"

#define NO_TIMER

#endif // !ROS2

#include "filters/filter_chain.h"

class ScanToScanFilterChain
{
protected:
  // Our NodeHandle
#ifndef ROS2
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
#else
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr private_nh_;
#endif // !ROS2

  // Components for tf::MessageFilter
  TransformListener tf_;
  tf2_ros::Buffer buffer_;

  message_filters::Subscriber<LaserScan> scan_sub_;
#ifndef ROS2
  tf2::MessageFilter<LaserScan> tf_filter_;
#else
  tf2_ros::MessageFilter<LaserScan> tf_filter_;
#endif // !ROS2
  double tf_filter_tolerance_;

  // Filter Chain
  filters::FilterChain<LaserScan> filter_chain_;

  // Components for publishing
  LaserScan msg_;
#ifndef ROS2
  ros::Publisher output_pub_;
#else
  rclcpp::Publisher<LaserScan>::SharedPtr output_pub_;
#endif // !ROS2

  // Deprecation helpers
#ifndef NO_TIMER
  ros::Timer deprecation_timer_;
#endif // !NO_TIMER
  bool  using_filter_chain_deprecated_;

public:
  // Constructor
  ScanToScanFilterChain() :
    nh_(rclcpp::Node::make_shared("scan_to_scan_filter_chain")),
    private_nh_(rclcpp::Node::make_shared("~")),
    scan_sub_(nh_, "scan", 50),
    tf_(buffer_),
    tf_filter_(scan_sub_, buffer_, "", 50),
    filter_chain_("LaserScan")
  {
    // Configure filter chain
    
    rclcpp::parameter::ParameterVariant variant;
    using_filter_chain_deprecated_ = !private_nh_->get_parameter("filter_chain", variant);

    if (using_filter_chain_deprecated_)
      filter_chain_.configure("filter_chain"/*, private_nh_*/);
    else
      filter_chain_.configure("scan_filter_chain"/*, private_nh_*/);
    
    std::string tf_message_filter_target_frame;

    if (!private_nh_->get_parameter("tf_message_filter_target_frame", variant))
    {
      private_nh_->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame);

      private_nh_->get_parameter_or("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

      tf_filter_.setTargetFrame(tf_message_filter_target_frame);
      tf_filter_.setTolerance(tf2::Duration(ros::Duration(tf_filter_tolerance_).toNSec()));

      // Setup tf::MessageFilter generates callback
      auto boostFxn = boost::bind(&ScanToScanFilterChain::callback, this, _1);
      tf_filter_.registerCallback(boostFxn);
    }
    else 
    {
      // Pass through if no tf_message_filter_target_frame
      auto boostFxn = boost::bind(&ScanToScanFilterChain::callback, this, _1);
      scan_sub_.registerCallback(boostFxn);
    }
    
    // Advertise output
    output_pub_ = nh_->create_publisher<LaserScan>("scan_filtered", 1000);

#ifndef NO_TIMER
    // Set up deprecation printout
    deprecation_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&ScanToScanFilterChain::deprecation_warn, this, _1));
#endif // !NO_TIMER
  }

  // Destructor
  ~ScanToScanFilterChain()
  {
#ifndef ROS2
    if (tf_filter_)
      delete tf_filter_;
    if (tf_)
      delete tf_;
#endif // !ROS2
  }
  
#ifndef NO_TIMER
  // Deprecation warning callback
  void deprecation_warn(const ros::TimerEvent& e)
  {
    if (using_filter_chain_deprecated_)
      ROS_WARN("Use of '~filter_chain' parameter in scan_to_scan_filter_chain has been deprecated. Please replace with '~scan_filter_chain'.");
  }
#endif // !NO_TIMER

  // Callback
  void callback(const boost::shared_ptr<const LaserScan>& msg_in)
  {
    // Run the filter chain
    if (filter_chain_.update(*msg_in, msg_))
    {
      //only publish result if filter succeeded
      output_pub_->publish(msg_);
    }
  }
};

int main(int argc, char **argv)
{
#ifndef ROS2
  ros::init(argc, argv, "scan_to_scan_filter_chain");

  ScanToScanFilterChain t;
  ros::spin();
#else
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("scan_to_scan_filter_chain");
  ScanToScanFilterChain t;

  rclcpp::spin(nh);
#endif // !ROS2
  
  return 0;
}
