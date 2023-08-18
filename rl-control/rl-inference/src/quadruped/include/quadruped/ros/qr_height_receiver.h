// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_HEIGHTRECEIVER_H
#define QR_HEIGHTRECEIVER_H

#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "robots/qr_robot.h"
#include "estimators/qr_ground_surface_estimator.h"


namespace Quadruped {

/**
 * @brief Receive command velocity from high level planning module.
 */
class qrHeightReceiver {

public:

    qrHeightReceiver(ros::NodeHandle &nhIn, qrRobot *robotIn, qrGroundSurfaceEstimator* surfaceEstimatorIn);

    ~qrHeightReceiver() = default;

    void HeightCallback(const std_msgs::Float32MultiArray::ConstPtr &input);
    
    void PubCallback();

    ros::NodeHandle &nh;
    ros::Subscriber heightSub;
    ros::Publisher rpyPub;
    std::string heightTopic = "/height_receiver";

private:
    qrRobot* robot;
    qrGroundSurfaceEstimator* surfaceEstimator;
    std_msgs::Float32MultiArray rpyMsg;
};

} // Namespace Quadruped

#endif // QR_HEIGHTRECEIVER_H
