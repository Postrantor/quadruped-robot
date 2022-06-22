// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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

#ifndef QR_VEL_PARAM_RECEIVER_H
#define QR_VEL_PARAM_RECEIVER_H

#include <iostream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

/**
 * @brief A qrVelocityParamReceiver object recieves the velocity parameters from a ROS topic when a gait updates
 * 
 */
class qrVelocityParamReceiver {
    public:
        /**
         * @brief Construct a qrVelocityParamReceiver object using a givne ROS nodeHandle.
         * @param nhIn specifies the ROS node to communicate
         */
        qrVelocityParamReceiver(ros::NodeHandle &nhIn);

        /**
         * @brief Deconstruct a qrVelocityParamReceiver object.
         */
        ~qrVelocityParamReceiver() = default;

        /**
        * @brief Receive the updated information from a ROS topic
        * @param msg the gait msg from a ROS topic
        */
        void VelocityParamCallback(const geometry_msgs::Twist::ConstPtr &msg);

        /**
        * @brief Get the linear velocity.
        * @return Eigen::Matrix<float, 3, 1>: linearVel  
        */
        Eigen::Matrix<float, 3, 1> GetLinearVelocity();

       /**
        * @brief Get the angular velocity.
        * @return Eigen::Matrix<float, 3, 1>: angularVel  
        */
        Eigen::Matrix<float, 3, 1> GetAngularVelocity();

        /**
        * @brief Get the angular velocity.
        * @return float: the z coordinate of the angularVel  
        */
        float GetAngularVelocity();
        
        geometry_msgs::Twist velParam;
        ros::NodeHandle &nh;
        ros::Subscriber velParamSub;
        std::string velParamTopic = "/velocity_param";

    private:
        Eigen::Matrix<float, 3, 1> linearVel = Eigen::Matrix<float, 3, 1>::Zero();
        Eigen::Matrix<float, 3, 1> angularVel = Eigen::Matrix<float, 3, 1>::Zero();

};
#endif //QR_VEL_PARAM_RECEIVER_H