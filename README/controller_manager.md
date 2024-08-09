##

```bash
/opt/ros/humble/lib/controller_manager/spawner \
  --activate-as-group \
    joint_state_broadcaster \
    FL_hip_controller \
    FL_thigh_controller \
    FL_calf_controller \
    FR_hip_controller \
    FR_thigh_controller \
    FR_calf_controller \
    RL_hip_controller \
    RL_thigh_controller \
    RL_calf_controller \
    RR_hip_controller \
    RR_thigh_controller \
    RR_calf_controller \
  --ros-args
```
