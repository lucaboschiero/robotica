#include <custom_joint_publisher.h>
#include <math.h>
#include "include/ur5kinematics.h"
#include <std_msgs/String.h>
#include <cstring>
#include <custom_msgs/Coord.h>

void send_des_jstate(const JointStateVector & joint_pos)
{
    //std::cout << "q_des " << joint_pos.transpose() << std::endl;
    if (real_robot)
    {
        for (int i = 0; i < joint_pos.size(); i++)
        {
          jointState_msg_robot.data[i] = joint_pos[i];
        }

        pub_des_jstate.publish(jointState_msg_robot);


    } else {
        for (int i = 0; i < joint_pos.size(); i++)
        {
          jointState_msg_sim.position[i] = joint_pos[i];
          jointState_msg_sim.velocity[i] = 0.0;
          jointState_msg_sim.effort[i] = 0.0;
        }

        pub_des_jstate.publish(jointState_msg_sim);
    }

/*   if (pub_des_jstate_sim_rt->trylock())
  {
    pub_des_jstate_sim_rt->msg_ = jointState_msg;
    pub_des_jstate_sim_rt->unlockAndPublish();
  } */
}

void initFilter(const JointStateVector & joint_pos)
{
        filter_1 = joint_pos;
        filter_2 = joint_pos;
}

JointStateVector secondOrderFilter(const JointStateVector & input, const double rate, const double settling_time)
{

        double dt = 1 / rate;
        double gain =  dt / (0.1*settling_time + dt);
        filter_1 = (1 - gain) * filter_1 + gain * input;
        filter_2 = (1 - gain) * filter_2 + gain *filter_1;
        return filter_2;
}

void coordinate(const custom_msgs::Coord::ConstPtr& msg){

    //cout<<"S: "<<s;
    int cl;
    double x,y,z;

    x=msg->x;
    y=msg->y;
    z=msg->z;
    cl=msg->cl;
    cl--;

    blocks(cl,0)=x;
    blocks(cl,1)=y;
    blocks(cl,2)=z;

    //cout<<blocks<<endl;


    /*char *div; // declare a ptr pointer
    div = strtok(s, " "); // use strtok() function to separate string using comma (,) delimiter.
    // use while loop to check ptr is not null
    int cont=0;
    while (div!=NULL || cont<4)
    {
        if(cont==0) x=atof(div);
        if(cont==1) y=atof(div);
        if(cont==2) z=atof(div);
        if(cont==3) cl=atoi(div);
        div = strtok (NULL, " ");
        cont++;
    }
    cl--;
    //cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<" classe: "<<cl<<endl;
    blocks(cl,0)=x;
    blocks(cl,1)=y;
    blocks(cl,2)=z;
    cout<<blocks<<endl;*/

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;

  //pub_des_jstate_sim_rt.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(node, "/command", 1));

  //node.getParam("/real_robot", real_robot);


  if (real_robot)
  {
      pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);

  } else {
      pub_des_jstate = node.advertise<sensor_msgs::JointState>("/command", 1);
  }

  ros::Rate loop_rate(loop_frequency);

  jointState_msg_sim.position.resize(6);
  jointState_msg_sim.velocity.resize(6);
  jointState_msg_sim.effort.resize(6);
  jointState_msg_robot.data.resize(6);

  q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
  initFilter(q_des0);

  Vector3d peIniziale;
  peIniziale=ur5Direct(q_des0);

  JointStateVector amp;
  JointStateVector freq;
  amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;
  freq << 0.2, 0.0, 0.0, 0.0, 0., 0.0;

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/coordinates", 1000, coordinate);
  Vector3d pe;

  pe << 0.5, 0.4, 0.87;  // posizione desiderata rispetto al world frame (problemi con la y)


  Vector3d shift;
  shift=pe-Vector3d(0.5,0.65,0.15);                //posizione shiftata per avere pe rispetto al base frame
  
  Matrix86d Thresult=ur5Inverse(shift);
  JointStateVector q_prova,q_prova0;
  q_prova << Thresult(0,0), Thresult(0,1), Thresult(0,2), Thresult(0,3), Thresult(0,4), Thresult(0,5);
  q_prova0 << 0.0,0.0,0.0,0.0,0.0,0.0;
  initFilter(q_prova);
  
  //JointStateVector q_prova,q_prova0;
  Matrix3d Kp = Matrix3d::Identity()*10;
  Matrix3d Kphi = Matrix3d::Identity()*0.00001;
  //q_prova0 << 0.00000,0.00000,0.00000,0.00000,0.00000,0.00000;
  MatrixXd differentialTH=invDiffKinematicControlSimComplete(peIniziale,shift,rotm2eulFDR(Re),Vector3d(0.0,0.0,M_PI),q_des0,Kp,Kphi, TMIN, TMAX, DELTAT);
  //cout<<differentialTH.rows()<<endl;

  int i=0;
  while (ros::ok())
  {

    //1- step reference
    if (loop_time < 5.)
    {
      q_des = q_des0;
      //send_des_jstate(q_des);
    } else {
      //JointStateVector delta_q;
      //delta_q << 0., 0.4, 0., 0., 0., 0.;
      //q_des = q_des0 + delta_q;

      if(i<differentialTH.rows()){

        q_prova << differentialTH(i,0),differentialTH(i,1),differentialTH(i,2),differentialTH(i,3),differentialTH(i,4),differentialTH(i,5);
        //initFilter(q_prova);
        //q_des = secondOrderFilter(q_prova, loop_frequency, 5.);
        q_des=q_prova;
        
        //send_des_jstate_sim(q_des);
        //send_des_jstate(q_des);
      
        i++;
        cout<<i<<endl;
      }
    }
    send_des_jstate(q_des);
    loop_time += (double)1/loop_frequency;
    //2- sine reference
//    q_des = q_des0.array() + amp.array()*(2*M_PI*freq*loop_time).array().sin();

    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
