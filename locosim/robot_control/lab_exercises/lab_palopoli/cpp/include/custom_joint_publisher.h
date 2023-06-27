#ifndef CUSTOM_JOINT_PUB_H
#define CUSTOM_JOINT_PUB_H

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <custom_msgs/Coord.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <realtime_tools/realtime_publisher.h>
#include <math.h>
#include <std_msgs/String.h>
#include <cstring>
#include <iostream>
#include "ur5kinematics.h"
#include "time.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 11, 6> Matrix116d;

// Variables
Vector6d filter_1 = Vector6d::Zero();
Vector6d filter_2 = Vector6d::Zero();
Vector6d q_des0 = Vector6d::Zero();
Matrix116d detected_pos_blocchetti = Matrix116d::Zero(11, 6);
bool apri = true;
bool detection = true;
clock_t start_time, end_time;
double total_time;
Vector3d shifted_final_stand = Vector3d::Zero();

double loop_time = 0.;
double loop_frequency = 1000.;

// costanti
const Vector3d shift(0.5, 0.35, 0.12);
const Vector3d final_stand(-0.4, -0.2, 0.73);
const Vector3d up_down_di(0.0, 0.0, 0.12);

// Publishers
ros::Publisher pub_des_jstate;
std_msgs::Float64MultiArray jointState_msg_robot;

// metodi
void send_des_jstate(const Vector6d &joint_pos, const Vector3d &gripper_joint);
void initFilter(const Vector6d &joint_pos);
Vector6d secondOrderFilter(const Vector6d &input, const double rate, const double settling_time);
void detect(const custom_msgs::Coord::ConstPtr &msg);
bool homing();
Vector6d rotate(Vector6d q, Vector3d rot);
Vector3d open_gripper(double d);
Vector6d moveTo(Vector3d pos_iniziale, Vector3d pos_finale, Vector3d rot_iniziale, Vector3d rot_finale);
bool grasp(Vector6d q);
void motionPlan(Vector6d pos_blocchetto, int classe);

int use_gripper = 1; // 0 -> no gripper    1-> soft gripper     2-> 3-finger gripper
#endif
