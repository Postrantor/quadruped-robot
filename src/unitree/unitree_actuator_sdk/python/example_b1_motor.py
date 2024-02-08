from unitree_actuator_sdk import *
import time
import sys
sys.path.append('../lib')


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

while True:
  data.motorType = MotorType.B1
  cmd.motorType = MotorType.B1
  cmd.mode = queryMotorMode(MotorType.B1, MotorMode.FOC)
  cmd.id = 0
  cmd.q = 0.0
  cmd.dq = 6.28 * queryGearRatio(MotorType.B1)
  cmd.kp = 0.0
  cmd.kd = 3
  cmd.tau = 0.0
  serial.sendRecv(cmd, data)
  print('\n')
  print("q: " + str(data.q))
  print("dq: " + str(data.dq))
  print("temp: " + str(data.temp))
  print("merror: " + str(data.merror))
  print('\n')
  time.sleep(0.0002)  # 200 us
