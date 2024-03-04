/**
 * @brief
 * @date 2024-01-25
 * @copyright Copyright (c) 2024
 */

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>

#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_SPACE 0x20

// pulsed mode or continuous mode
int mode = 1;

class teleForceCmd {
public:
  teleForceCmd();

  void keyLoop();
  void pubForce(double x, double y, double z);

private:
  double Fx, Fy, Fz;
  ros::NodeHandle n;
  ros::Publisher force_pub;
  geometry_msgs::Wrench Force;
};

teleForceCmd::teleForceCmd() {
  Fx = 0;
  Fy = 0;
  Fz = 0;

  force_pub = n.advertise<geometry_msgs::Wrench>("/apply_force/trunk", 20);

  sleep(1);
  pubForce(Fx, Fy, Fz);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "external_force");

  teleForceCmd remote;
  signal(SIGINT, quit);
  remote.keyLoop();

  return (0);
}

void teleForceCmd::pubForce(double x, double y, double z) {
  Force.force.x = Fx;
  Force.force.y = Fy;
  Force.force.z = Fz;

  force_pub.publish(Force);
  ros::spinOnce();
}

void teleForceCmd::keyLoop() {
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);

  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("reading from keyboard");
  puts("---------------------------");
  puts("use 'space' to change mode, default is pulsed mode:");
  puts("use 'up/down/left/right' to change direction");

  for (;;) {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }
    ROS_DEBUG("value: 0x%02X\n", c);
    switch (c) {
      case KEYCODE_UP:
        if (mode > 0) {
          Fx = 60;
        } else {
          Fx += 16;
          if (Fx > 220) Fx = 220;
          if (Fx < -220) Fx = -220;
        }
        ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
        dirty = true;
        break;
      case KEYCODE_DOWN:
        if (mode > 0) {
          Fx = -60;
        } else {
          Fx -= 16;
          if (Fx > 220) Fx = 220;
          if (Fx < -220) Fx = -220;
        }
        ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
        dirty = true;
        break;
      case KEYCODE_LEFT:
        if (mode > 0) {
          Fy = 30;
        } else {
          Fy += 8;
          if (Fy > 220) Fy = 220;
          if (Fy < -220) Fy = -220;
        }
        ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        if (mode > 0) {
          Fy = -30;
        } else {
          Fy -= 8;
          if (Fy > 220) Fy = 220;
          if (Fy < -220) Fy = -220;
        }
        ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
        dirty = true;
        break;
      case KEYCODE_SPACE:
        mode = mode * (-1);
        if (mode > 0) {
          ROS_INFO("Change to Pulsed mode.");
        } else {
          ROS_INFO("Change to Continuous mode.");
        }
        Fx = 0;
        Fy = 0;
        Fz = 0;
        ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
        dirty = true;
        break;
    }
    if (dirty == true) {
      pubForce(Fx, Fy, Fz);
      if (mode > 0) {
        usleep(100000);  // 100 ms
        Fx = 0;
        Fy = 0;
        Fz = 0;
        pubForce(Fx, Fy, Fz);
      }
      dirty = false;
    }
  }
  return;
}
