#ifndef QR_CONFIG_H
#define QR_CONFIG_H

#define A1_BODY_MASS 10
#define Go1_BODY_MASS 10
#define A1_BODY_HIGHT 0.27  // for robot position init

#define DEFAULT_WINDOW_SIZE 20

#define NumLeg 4
#define NumMotor 12

#define BaseFreedomDim 6

#define MAX_TIME_SECONDS 1000.0f

#define JOY_CMD_VELX_MAX 0.3
#define JOY_CMD_VELY_MAX 0.3
#define JOY_CMD_YAW_MAX 0.3
#define JOY_CMD_ROLL_MAX 0.1
#define JOY_CMD_PITCH_MAX 0.1

const int NumMotorOfOneLeg = 3;
const float MAXIMUM_STEP = 0.001f;

#endif  // QR_CONFIG_H
