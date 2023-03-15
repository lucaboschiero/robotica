#ifndef CUSTOM_JOINT_PUB_H
#define CUSTOM_JOINT_PUB_H


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <custom_msgs/Coord.h>
#include <custom_msgs/PosRobot.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <realtime_tools/realtime_publisher.h>

using namespace std;
using namespace Eigen;

typedef  Matrix<double, 6, 1> Vector6d;
typedef  Matrix<double, 11, 3> Matrix113d;


// Methods
void send_des_jstate(const Vector6d & joint_pos, const Vector3d & gripper_joint);
Vector6d secondOrderFilter(const Vector6d & input, const double rate, const double settling_time);

// Variables
Vector6d q_des = Vector6d::Zero();
Vector6d qd_des = Vector6d::Zero();
Vector6d tau_ffwd = Vector6d::Zero();
Vector6d filter_1 = Vector6d::Zero();
Vector6d filter_2 = Vector6d::Zero();
Vector6d q_des0 = Vector6d::Zero();
Matrix113d detected_pos_blocchetti=Matrix113d::Zero(10,3);


const Vector3d shift(0.5,0.35,0.17);
const Vector3d final_stand(0.3,-0.2,0.7);
const Vector3d up_down_di(0.0,0.0,0.15);


double  loop_time = 0.;
double  loop_frequency = 1000.;
bool apri = true;

// Publishers
//std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > pub_des_jstate_sim_rt;
ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_sim;
std_msgs::Float64MultiArray jointState_msg_robot;

//void detect(const custom_msgs::Coord::ConstPtr& msg);
bool homing();
Vector6d moveTo(Vector3d pos_iniziale,Vector3d pos_finale, Vector3d rot_iniziale, Vector3d rot_finale);
bool grasp(Vector6d q);
void motionPlan(Vector3d pos_blocchetto,int classe);


bool real_robot = true;
int use_gripper = 1;        //0 -> no gripper    1-> soft gripper     2-> 3-finger gripper

#endif
