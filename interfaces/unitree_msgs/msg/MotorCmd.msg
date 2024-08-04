# // https://design.ros2.org/articles/generated_interfaces_cpp.html

# motor target mode
#   - servo(PMSM): 0x0A
#   - damping(BRAKE): 0x00.
# uint8 mode
# float32 q # target position
# float32 dq # target velocity
# float32 tau # target torque
# float32 kp # spring stiffness coefficient
# float32 kd # damper coefficient
# uint32[3] reserve # target torque

# 针对go-m8010-6型号的sdk重构了msg
std_msgs/Header header # 包含时间戳和帧ID

int32 hex_len # 17;
uint16 id # motor id: 0~14, 15: broadcast id, no return
uint16 mode # motor mode
float32 q # 期望关节位置(电机转子位置 rad) ±411774
float32 dq # 期望关节速度(电机转子转速 rad/s) ±804.00
float32 tau # 期望关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
float32 k_q # 关节刚度系数 0~25.599
float32 k_dq # 关节速度系数 0~25.599
