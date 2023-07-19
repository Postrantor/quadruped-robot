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

#ifndef RL_LOCOMOTION_CONTROLLER_H
#define RL_LOCOMOTION_CONTROLLER_H

#include "controllers/qr_locomotion_controller.h"
#if USE_NPU
#include <acl/acl.h>
#endif
#if USE_ONNX
#include <onnxruntime/onnxruntime_cxx_api.h>
#endif

namespace Quadruped {

class LocomotionControllerRLWrapper{

public:

    /**
     * @brief Constructor of class LocomotionControllerRLWrapper.
     * @param robot: pointer to the Robot.
     * @param gaitGenerator: pointer to the Gait Generator.
     * Openloop Gait Generator or Walk Gait Generator.
     * @param desiredStateCommand: pointer to DesiredStateCommand.
     * @param stateEstimator: pointer to StateEstimatorContainer.
     * @param comAdjuster: pointer to ComAdjuster.
     * @param posePlanner: pointer to PosePlanner.
     * @param swingLegController: pointer to RaibertSwingLegController.
     * @param stanceLegController: pointer to StanceLegControllerInterface.
     * @param userParameters: pointer to UserParameters.
     */
    LocomotionControllerRLWrapper(qrLocomotionController* locomotionController);

    ~LocomotionControllerRLWrapper();

    void Reset();

    void BindCommand(std::string networkPath);

    /**
     * @brief Update components in the locomotion controller every control loop.
     */
    void RLUpdate(bool enableInference);

    void CollectProprioceptiveObs();
    
    void Inference(Eigen::Matrix<float, 320, 1>& obs);

    void PMTGStep();


    /** @brief Compute all motors' commands via subcontrollers.
     *  @return control ouputs (e.g. positions/torques) for all (12) motors.
     */
    std::vector<qrMotorCommand> GetRLAction();

    const float MaxHorizontalOffset = 0.05; // 0.05
    const float  MaxClearance = 0.15; // 0.15

    qrLocomotionController* locomotionController;
    qrRobot *robot;

private:

    std::string net_model_dir;
    const int PROPRIOCEPTION_SIZE = 133; // Proprioceptive
    const int EXTEROCEPTION_SIZE = 187; // Exteroceptive
    const int NUM_HISTORY_STEPS = 40;
    Eigen::Matrix<float, 133, 1> proprioceptiveObs;
    Eigen::Matrix<float, 187, 1> exteroceptiveObs;
    Eigen::Matrix<float, 320, 1> total_obs;
    std::vector<float> obs_history;
    Vec12<float> residualAngle;
    Vec4<float> deltaPhi;

    Vec12<float> target_joint_angles;
    Eigen::Matrix<float, 4, 3> foot_target_position_in_hip_frame;

    bool allowUpdateObs = true;

    Vec3<float> command;
    float command_scale, rpy_scale, v_scale, w_scale, dp_scale, dv_scale, cpg_scale, height_scale;
    float gaitFreq;
    Eigen::Matrix<float, 13, 1> cpg_info;

    Eigen::Matrix<float, 12*3, 1> dof_p_history;
    Eigen::Matrix<float, 12*2, 1> dof_v_history;
    Eigen::Matrix<float, 12*2, 1> dof_p_target_history;
    
    std::deque<Vec12<float>> deque_dof_p;
    std::deque<Vec12<float>> deque_dof_v;
    std::deque<Vec12<float>> deque_dof_p_target;

    #if USE_ONNX
    Ort::Env onnxEnv;
    Ort::Session onnxSession(nullptr);
    Ort::AllocatorWithDefaultOptions onnxAllocator;
    #endif
};

} // Namespace Quadruped

#endif // RL_LOCOMOTION_CONTROLLER_H
