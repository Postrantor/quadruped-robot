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

#include "controllers/rl_locomotion_controller.h"

namespace Quadruped {

inline float CubicUp(float x) {
    return -16 * pow(x, 3) + 12 * pow(x, 2);
}

inline float CubicDown(float x) {
    return 16 * pow(x, 3) - 36 * pow(x, 2) + 24 * x - 4;
}

void SwapFourLeg(Vec12<float>& data) {
    Vec3<float> temp;
    for (int i=0; i < 12; i = i + 6) {
        temp = data.block<3, 1>(0 + i, 0);
        data.block<3, 1>(0 + i, 0) = data.block<3, 1>(3 + i,0);
        data.block<3, 1>(3 + i, 0) = temp;
    }
}

rlLocomotionController::rlLocomotionController(
    qrRobot *robotIn,
    qrGaitGenerator *gaitGeneratorIn,
    qrDesiredStateCommand* desiredStateCommandIn,
    qrStateEstimatorContainer *stateEstimatorIn,
    qrComAdjuster *comAdjusterIn,
    qrPosePlanner *posePlannerIn,
    qrRaibertSwingLegController *swingLegControllerIn,
    qrStanceLegControllerInterface *stanceLegControllerIn,
    qrUserParameters *userParameters):
    
    qrLocomotionController(robotIn, gaitGeneratorIn, desiredStateCommandIn, stateEstimatorIn, comAdjusterIn, posePlannerIn, swingLegControllerIn, stanceLegControllerIn, userParameters),
    onnxSession(nullptr)
{
    command_scale = 1;
    rpy_scale = 1;
    v_scale = 1;
    w_scale = 1;
    dp_scale = 1;
    dv_scale = 0.1;
    cpg_scale = 1;
    height_scale = 5;
    obs_history.reserve(NUM_HISTORY_STEPS * 320);

    // Reset();

}

void rlLocomotionController::Reset()
{
    qrLocomotionController::Reset();
    // resetTime = robot->GetTimeSinceReset();
    // timeSinceReset = 0.;
    // gaitGenerator->Reset(timeSinceReset);
    // comAdjuster->Reset(timeSinceReset);
    // posePlanner->Reset(timeSinceReset);
    // swingLegController->Reset(timeSinceReset);
    // stanceLegController->Reset(timeSinceReset);
    // BindCommand();

    deque_dof_p.clear();
    deque_dof_v.clear();
    deque_dof_p_target.clear();
    obs_history.clear();
    gaitFreq = 1.f/gaitGenerator->fullCyclePeriod[0];
    
    target_joint_angles.setZero();
    deltaPhi.setZero();
    residualAngle.setZero();
    proprioceptiveObs.setZero();
    exteroceptiveObs.setZero();
    total_obs.setZero();
    // cpg_info << 0,0,0,0,
    //             1,1,1,1,
    //             0,0,0,0,
    //             gaitFreq;
    PMTGStep(0);

}


void rlLocomotionController::BindCommand(std::string networkPath, std::string networkType) {
    qrLocomotionController::BindCommand();
    
    if (networkPath.size() == 0) {
        throw std::logic_error("");
    } else {
        // TODO
        // LoadModel
        if (networkType == "onnx") {
            onnxEnv = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "test");
            Ort::SessionOptions session_options;
            onnxSession = Ort::Session(onnxEnv, "./a1.onnx", session_options);
        } else if (networkType == "om") {
            ;
        } else {
            throw std::logic_error("");
        }
        // ACL LOAD FUNCTION
        // ONNX LOAD FUNCTION
        // CKPT LOAD FUNCTION
    }    
}


void rlLocomotionController::RLUpdate(bool enableInference)
{
    if (!robot->stop) {
        timeSinceReset = robot->GetTimeSinceReset() - resetTime;
    }
    if (enableInference) {
        if (allowUpdateObs) {
            CollectProprioceptiveObs(); // proprioceptiveObs
            
            total_obs << proprioceptiveObs, 
                        exteroceptiveObs;
            // allowUpdateObs = false;
        }
        // MITTimer tik;
        Inference(total_obs);
        // printf("Inference TIME: %.3f [ms]\n", tik.getMs()); 
        
    }
    
    
    // pmtg update, not swing controllers
    gaitGenerator->Update(timeSinceReset);
    target_joint_angles.setZero();
    PMTGStep(timeSinceReset);
    /*
    -0.331181   0.98929 -0.239799  
    0.357972   1.98558 -0.847333  
    0.188387   2.77172  -1.35047  
    0.365737   2.31064  -1.36795
    */
    // std::cout << "target_joint = " << target_joint_angles.transpose() << std::endl;
    if (deque_dof_p_target.size() < 2) {
        Vec12<float> swaped_target_joint_angles = target_joint_angles;
        SwapFourLeg(swaped_target_joint_angles);
        deque_dof_p_target.push_back(swaped_target_joint_angles);
    }
}


void rlLocomotionController::Inference(Eigen::Matrix<float, 320, 1>& obs)
{
    // std::cout << "dof p = " << robot->GetMotorAngles().transpose() << std::endl;
    std::vector<int64_t> input0_node_dims = {1, 320};
    std::vector<float> input0_tensor_values(obs.data(), obs.data() + 320);
    std::vector<int64_t> input1_node_dims = {1, 320*40};
    // std::cout << "obs = " << obs.block<133, 1>(0,0).transpose() << std::endl;

    if (obs_history.empty()) {
        Eigen::Matrix<float, 12800, 1> obs_history_data = obs.replicate<40,1>();
        obs_history.insert(obs_history.end(), obs_history_data.data(),  obs_history_data.data() + obs_history_data.size());
    } else {
        auto it = obs_history.begin();
        obs_history.erase(it, it+320);
        obs_history.insert(obs_history.end(), obs.data(), obs.data()+obs.size());
    }

    // pretty_print(obs_history.data() + 320*38, "obs_his[-2]", 320);
    // pretty_print(obs_history.data() + 320*39, "obs_his[-1]", 320);
    // std::cout << "obs_his[-2] = " << obs_history.block<1, 320>(0, 320*38) << std::endl;
    // std::cout << "obs_his[-1] = " << obs_history.block<1, 320>(0, 320*39) << std::endl;
    // std::vector<float> input1_tensor_values(obs_history.data(), obs_history.data() + obs_history.rows() * obs_history.cols());
    std::array<float, 16> results_{};
    std::array<int64_t, 2> output_shape_{1, 16};
    // std::vector<float> output_tensor_values(results_.data(), results_.size());

    // Fill input_tensor_values with your input data.
    std::vector<Ort::Value> input_tensor;
    
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    // Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    // Ort::Value input0_tensor = Ort::Value::CreateTensor<float>(memory_info, input0_tensor_values.data(), 320, input0_node_dims.data(), input0_node_dims.size());
    // Ort::Value input1_tensor = Ort::Value::CreateTensor<float>(memory_info, obs_history.data(), 320*40, input1_node_dims.data(), input1_node_dims.size());
    input_tensor.emplace_back(Ort::Value::CreateTensor<float>(memory_info, input0_tensor_values.data(), 320, input0_node_dims.data(), input0_node_dims.size()));
    input_tensor.emplace_back(Ort::Value::CreateTensor<float>(memory_info, obs_history.data(), 320*40, input1_node_dims.data(), input1_node_dims.size()));
    Ort::Value output_tensor = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(),
                                                     output_shape_.data(), output_shape_.size());

    Ort::AllocatedStringPtr inputNodeName0 = onnxSession.GetInputNameAllocated(0, onnxAllocator);
    Ort::AllocatedStringPtr inputNodeName1 = onnxSession.GetInputNameAllocated(1, onnxAllocator);
    Ort::AllocatedStringPtr outputNodeName = onnxSession.GetOutputNameAllocated(0, onnxAllocator);
    const char* inputName0 = inputNodeName0.get();
    // std::cout << "Input Name0: " << inputName0 << std::endl;
    const char* inputName1 = inputNodeName1.get();
    // std::cout << "Input Name1: " << inputName1 << std::endl;
    const char* outputName = outputNodeName.get();
    // std::cout << "Output Name: " << outputName << std::endl;
    
    //Run Inference
    std::vector<const char*> inputNames{inputName0, inputName1};
    std::vector<const char*> outputNames{outputName};
    // std::vector<Ort::Value*> input_tensor{&input0_tensor, &input1_tensor};
    // input_tensor.push_back(input0_tensor);
    // input_tensor.push_back(input1_tensor);
    
    // Run inference.
    // onnxSession.Run(Ort::RunOptions{nullptr}, inputNames.data(), *((const Ort::Value* const*)input_tensor.data()), inputNames.size(), outputNames.data(),  &output_tensor, outputNames.size());
    onnxSession.Run(Ort::RunOptions{nullptr}, inputNames.data(), input_tensor.data(), inputNames.size(), outputNames.data(),  &output_tensor, outputNames.size());

    
    // auto& output_tensor = output_tensors.front().Get<Tensor>();
    // auto output_shape = output_tensor.Shape();
    // std::vector<float> output_tensor_values(output_shape.Size());
    // memcpy(output_tensor_values.data(), output_tensor.Data<float>(), output_shape.Size() * sizeof(float));
    
    // deltaPhi = output[:4];
    // residualAngle = output[4:16];
    for(int i=0; i<4; ++i) {
        int j = (i%2==0 ? i+1:i-1); // todo
        deltaPhi[i] = results_[j] * 0.2f;
        for (int k=0; k < 3; ++k) {
            residualAngle[3*i+k] = results_[4+3*j+k] * 0.2f;
        }
    }
    // Visualization2D& vis = robot->stateDataFlow.visualizer;
    // vis.datax.push_back(timeSinceReset);
    // vis.datay1.push_back(deltaPhi[0]);
    // vis.datay2.push_back(deltaPhi[1]);
    // vis.datay3.push_back(deltaPhi[2]);
    // vis.datay4.push_back(deltaPhi[3]);
    // vis.datay5.push_back(residualAngle[0]);
    // vis.datay6.push_back(residualAngle[1]);
    
    // memcpy(deltaPhi, results_.data(), 4*sizeof(float));
    // memcpy(residualAngle, results_.data() + 4, 12*sizeof(float));

    // std::cout << "results_  = ";
    // for (float ii : results_) {
    //     std::cout << " " << ii;
    // }
    // std::cout <<'\n';

}

void rlLocomotionController::CollectProprioceptiveObs()
{
    Vec3<float> command;
    command << desiredStateCommand->stateDes(6),
                desiredStateCommand->stateDes(7),
                desiredStateCommand->stateDes(11); 
    Vec12<float> dof_pos = robot->GetMotorAngles();
    Vec12<float> dof_v = robot->GetMotorVelocities();
    SwapFourLeg(dof_pos); // todo    
    SwapFourLeg(dof_v);
    // std::cout << "dof_pos = "<< dof_pos.transpose() << std::endl;
    
    while (deque_dof_p.size() < 4) {
        deque_dof_p.push_back(dof_pos);
    }
    while (deque_dof_v.size() < 3) {
        deque_dof_v.push_back(dof_v);
    }
    while (deque_dof_p_target.size() < 2) {
        deque_dof_p_target.push_back(dof_pos);
    }

    for (int i=0; i<3; ++i) {
        dof_p_history.block(12*i, 0, 12, 1) = deque_dof_p[i];
    }
    for (int i=0; i<2; ++i) {
        dof_v_history.block(12*i, 0, 12, 1) = deque_dof_v[i];
    }
    for (int i=0; i<2; ++i) {
        dof_p_target_history.block(12*i, 0, 12, 1) = deque_dof_p_target[i];
    }
    deque_dof_p.pop_front();
    deque_dof_v.pop_front();
    deque_dof_p_target.pop_front();

    /*
     0          0          0 
     -0.0231767  -0.120742   0.108926   
     0.126287  0.0200281  0.0321777  
     -0.113515   0.360507  -0.031097  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     -0.030091   0.232572  -0.269918  0.0227078   0.172648 -0.0502232  
     0.0922905   0.127618   0.278503  0.0641741    0.39569   0.446557  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     -0.030091   0.232572  -0.269918  0.0227078   0.172648 -0.0502232  
     0.0922905   0.127618   0.278503  0.0641741    0.39569   0.446557  
     -0.030091   0.232572  -0.269918  0.0227078   0.172648 -0.0502232  
     0.0922905   0.127618   0.278503  0.0641741    0.39569   0.446557  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823          
     0          0          0          0          1          1          1          1          0          0          0          0        1.4
    */
    proprioceptiveObs <<  command * command_scale,
                        robot->baseRollPitchYaw * rpy_scale,
                        robot->baseVelocityInBaseFrame * v_scale,
                        robot->baseRollPitchYawRate * w_scale,
                        dof_pos * dp_scale,
                        dof_v * dv_scale,
                        dof_p_history * dp_scale,
                        dof_v_history * dv_scale,
                        dof_p_target_history * dp_scale, // 
                        cpg_info[1] * cpg_scale, cpg_info[0] * cpg_scale, cpg_info[3] * cpg_scale, cpg_info[2] * cpg_scale,
                        cpg_info[5] * cpg_scale, cpg_info[4] * cpg_scale, cpg_info[7] * cpg_scale, cpg_info[6] * cpg_scale,
                        cpg_info[9] * cpg_scale, cpg_info[8] * cpg_scale, cpg_info[11] * cpg_scale, cpg_info[10] * cpg_scale,
                        cpg_info[12] * cpg_scale;  // 13

    // std::cout << "proprioceptiveObs= " << proprioceptiveObs.transpose() << std::endl;
}


void rlLocomotionController::PMTGStep(double timeSinceReset)
{
    // pretty_print(delta_phi_, "delta_phi_", 4);
    // pretty_print(residual_angle_, "residual_angle_", 12);
    // gen_foot_target_position_in_horizontal_hip_frame(delta_phi, residual_xyz, **kwargs)
    // foot_trajectory = gen_foot_trajectory_axis_z(delta_phi, timeSinceReset);
    // Eigen::Map<Eigen::MatrixXf> residual_angle(residual_angle_,12, 1);;
    // Eigen::Map<Eigen::MatrixXf> delta_phi(delta_phi_,4, 1);;

    // std::cout << gaitGenerator->phaseInFullCycle << std::endl;
    Vec4<float> phi = 2 * M_PI * gaitGenerator->phaseInFullCycle + 1*deltaPhi;
    // phi = phi.unaryExpr([](const float x) { return fmod(x, 3.1415f*2);});
    // std::cout << "phi = " << phi.transpose() << std::endl;
    cpg_info << deltaPhi, phi.array().cos(), phi.array().sin(), gaitFreq;

    Vec4<bool> is_swing = gaitGenerator->desiredLegState.cwiseEqual(0); //swing:0, stance: 1
    // is_swing.setZero(); // todo
    // std::cout << "is_swing = " << is_swing.transpose() << std::endl;
    Vec4<float> swing_phi = gaitGenerator->normalizedPhase;  // [0,1)
    Vec4<float> sin_swing_phi = (swing_phi * M_2PI).array().sin();
    // std::cout << "sin_swing_phi = " <<  sin_swing_phi.transpose() << std::endl;
    Vec4<float> factor{0,0,0,0};
    for (int i=0; i<4; ++i) {
        if(is_swing[i]) {
            factor[i] = swing_phi[i] < 0.5 ? CubicUp(swing_phi[i]):CubicDown(swing_phi[i]);
        }
    }

    foot_target_position_in_hip_frame.setZero();
    foot_target_position_in_hip_frame.col(2) = factor.cwiseProduct(is_swing.cast<float>() * MaxClearance).array() - robot->bodyHeight;
    foot_target_position_in_hip_frame.col(0) = -MaxHorizontalOffset * sin_swing_phi.cwiseProduct(is_swing.cast<float>());
    // std::cout << foot_target_position_in_hip_frame << std::endl;
    // foot_target_position_in_base_frame = transform_to_base_frame(foot_target_position_in_hip_frame, base_orientation)
    Mat3<float> RT = robotics::math::coordinateRotation(robotics::math::CoordinateAxis::X, robot->baseRollPitchYaw[0]) *
                     robotics::math::coordinateRotation(robotics::math::CoordinateAxis::Y, robot->baseRollPitchYaw[1]);
    Eigen::Matrix<float, 4, 3> foot_target_position_in_base_frame = robot->defaultHipPosition.transpose() +  foot_target_position_in_hip_frame * RT.transpose();  // WORLD --> BASE  [0.185, 0.135, 0]
    // std::cout << foot_target_position_in_base_frame << std::endl;

    // target_joint_angles =  get_target_joint_angles( foot_target_position_in_base_frame)
    Vec3<int> joint_idx;
    Vec3<float> target_joint_angles_per_leg;
    for (int i = 0; i < 4; ++i) {
        robot->ComputeMotorAnglesFromFootLocalPosition(i, foot_target_position_in_base_frame.row(i), joint_idx, target_joint_angles_per_leg);
        for (int j = 0; j < 3; ++j) {
            target_joint_angles(joint_idx[j]) = target_joint_angles_per_leg(j) + 1*residualAngle(joint_idx[j]);
        }
    }
    
}

std::vector<qrMotorCommand> rlLocomotionController::GetRLAction()
{
    action.clear(); 
    // target_joint_angles << 0, 0.8 , -1.6,
    //                         0, 0.8 , -1.6,
    //                         0, 0.8 , -1.6,
    //                         0, 0.8 , -1.6;
    target_joint_angles = target_joint_angles.cwiseMax(-3.0f).cwiseMin(3.0f);
    
    for (int jointId(0); jointId < NumMotor; ++jointId) {
        float kp=20;
        // if (jointId < 6) kp=0;
        action.push_back({target_joint_angles[jointId], kp, 0, 0.5, 0});
    }
    return action;
}

} // Namespace Quadruped
