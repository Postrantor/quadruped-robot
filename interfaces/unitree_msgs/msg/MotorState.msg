# // https://design.ros2.org/articles/generated_interfaces_cpp.html

# uint8 mode           # motor current mode
# float32 q            # motor current position（rad）
# float32 dq           # motor current speed（rad/s）
# float32 ddq          # motor current speed（rad/s）
# float32 tau_est       # current estimated output torque（N*m）
# float32 q_raw        # motor current position（rad）
# float32 dq_raw       # motor current speed（rad/s）
# float32 ddq_raw      # motor current speed（rad/s）
# int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
# uint32[2] reserve

# 针对go-m8010-6型号的sdk重构了msg
std_msgs/Header header # 包含时间戳和帧ID

int32 hex_len # 接收的命令长度: 16Byte
uint8 id # motor id: 0~14, 15: broadcast id, no return
uint8 mode # motor mode
float32 q # 关节位置(电机转子位置 rad) ±411774
float32 dq # 关节速度(电机转子转速 rad/s) ±804.00
float32 ddq          # motor current speed（rad/s）
float32 tau_est       # current estimated output torque（N*m）
float32 q_raw        # motor current position（rad）
float32 dq_raw       # motor current speed（rad/s）
float32 ddq_raw      # motor current speed（rad/s）
int32 temperature # temperature: -50~127°C
int32 error # error flag: 0.正常 1.过热 2.过流 3.过压 4.编码器故障
int32 foot_force # 足端气压传感器接口 ADC原始值
bool correct # 接收数据是否完整(true完整，false不完整或断联)
