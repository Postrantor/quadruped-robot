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

#include "ros/qr_height_receiver.h"

namespace Quadruped {

qrHeightReceiver::qrHeightReceiver(ros::NodeHandle &nhIn, qrRobot *robotIn, qrGroundSurfaceEstimator* surfaceEstimatorIn):
    nh(nhIn), robot(robotIn), surfaceEstimator(surfaceEstimatorIn)
{
    rpyMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rpyMsg.layout.dim[0].label = "z";
    rpyMsg.data = std::vector<float>(3, 0);
    // rpyMsg.layout.dim[0].size = surfaceEstimator->HeightPointNum;
    // rpyMsg.layout.dim[0].stride = surfaceEstimator->HeightPointNum;
    rpyMsg.layout.data_offset = 0;

    ROS_INFO("terrain height topic: %s", heightTopic.c_str());
    heightSub = nh.subscribe(heightTopic, 2, &qrHeightReceiver::HeightCallback, this);
    rpyPub = nh.advertise<std_msgs::Float32MultiArray>("/rpy_pub", 10);
}


void qrHeightReceiver::HeightCallback(const std_msgs::Float32MultiArray::ConstPtr &input)
{
    {
        std::lock_guard<std::mutex> lock(robot->mapMutex_);
        for (int i=0; i < surfaceEstimator->HeightPointNum; ++i) {
            surfaceEstimator->terrainHeights[i] = input->data[i];
        }
    }
    // std::cout << surfaceEstimator->terrainHeights[0] << std::endl;
    // terrainHeights = Eigen::Map<Eigen::Matrix<float, 187, 1>>(input->data);
    ROS_INFO_STREAM("[HeightCallback] received height samples");
}

void qrHeightReceiver::PubCallback()
{
    for (int i=0; i < 3; ++i) {
        rpyMsg.data[i] = robot->baseRollPitchYaw[i];
    }
    
    rpyPub.publish(rpyMsg);
}


} // Namespace Quadruped
