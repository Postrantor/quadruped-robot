# motor target mode
# - servo : 0x0A
# - damping : 0x00.
uint8 mode

float32 q # motor target position
float32 dq # motor target velocity
float32 tau # motor target torque
float32 kp # motor spring stiffness coefficient
float32 kd # motor damper coefficient

uint32[3] reserve # motor target torque
