/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  box_filter.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */



#ifndef BOXFILTER_H
#define BOXFILTER_H

#include <filters/filter_base.h>

#ifndef ROS2
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef sensor_msgs::LaserScan LaserScan;
typedef tf::TransformListener TransformListener;
typedef tf::TransformException TransformException;
typedef tf::Point Point;

#include "ros/ros.h"
#else
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/Laser_Scan.hpp>

typedef sensor_msgs::msg::LaserScan LaserScan;
typedef tf2_ros::TransformListener TransformListener;
typedef tf2::TransformException TransformException;
typedef tf2::Vector3 Point;


#define ROS_WARN_THROTTLE(...)
#define ROS_INFO_THROTTLE(...)

#endif // !ROS2

#include <laser_geometry/laser_geometry.h>



namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
 */
class LaserScanBoxFilter : public filters::FilterBase<LaserScan>
{
  public:
    LaserScanBoxFilter();
    bool configure();

    bool update(
      const LaserScan& input_scan,
      LaserScan& filtered_scan);

  private:
    bool inBox(Point &point);
    std::string box_frame_;
    laser_geometry::LaserProjection projector_;
    
    // tf listener to transform scans into the box_frame
    TransformListener tf_; 
    tf2_ros::Buffer buffer_;

    // defines two opposite corners of the box
    Point min_, max_; 
    bool up_and_running_;
};

}


#endif /* box_filter.h */
