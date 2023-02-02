#include <custom_joint_publisher.h>
#include <math.h>
#include "include/ur5kinematics.h"
#include <std_msgs/String.h>
#include <cstring>
#include <custom_msgs/Coord.h>


void send_des_jstate(const JointStateVector & joint_pos)
{
    std::cout << "q_des " << joint_pos.transpose() << std::endl;
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


void detect(const custom_msgs::Coord::ConstPtr& msg){
  //funzione che si sottoscrive al topic e riceve la matrice coi blocchetti

  Matrix103d blocks;
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

}


void motionPlan(Vector3d pos_blocchetto,int classe){

  Vector3d posIniziale=ur5Direct(q_des0);

  while(!moveTo(posIniziale,pos_blocchetto));  //muovo in base ai cubetti nella matrice

  while(!grasp());


  switch(classe){
    case 1:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));              //muovo da pos_blocchetto a final stand (non so dove sia ma vabbe)
    break;
    case 2:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 3:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 4:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 5:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 6:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 7:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 8:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 9:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
    case 10:
      while(!moveTo(pos_blocchetto-Vector3d(0.5,-0.001,0.1),Vector3d(0.2,0.5,HTAVOLO)));
    break;
  }

 

}



bool homing(){
  
  
  initFilter(q_des0);

  send_des_jstate(q_des0);

  return 1;
}

bool moveTo(Vector3d pos_iniziale,Vector3d pos_blocchetto){
  Vector3d shift=Vector3d(0.5,-0.001,0.1);
  pos_blocchetto=pos_blocchetto-shift;


  Matrix86d Thresult=ur5Inverse(pos_iniziale);
  Vector6d q_prova;
  q_prova << Thresult(0,0), Thresult(0,1), Thresult(0,2), Thresult(0,3), Thresult(0,4), Thresult(0,5);
  initFilter(q_prova);

  MatrixXd differentialTH=invDiffKinematicControlSimComplete(pos_iniziale,pos_blocchetto,rotm2eulFDR(Re),Vector3d(0.0,0.0,0.0),q_prova,Kp,Kphi, TMIN, TMAX, DELTAT);

  for(int i=0;i<differentialTH.rows();i++){
    q_prova << differentialTH(i,0),differentialTH(i,1),differentialTH(i,2),differentialTH(i,3),differentialTH(i,4),differentialTH(i,5);
    send_des_jstate(q_prova);
  }

  return 1;

}

bool grasp(){
  sleep(5);
  return 1;
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

  
  JointStateVector amp;
  JointStateVector freq;
  amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;
  freq << 0.2, 0.0, 0.0, 0.0, 0., 0.0;


  //globali
  Kp = Matrix3d::Identity()*5;
  Kphi = Matrix3d::Identity()*0.01;
  q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;



  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/coordinates", 1000, detect);
        //funzione che si sottoscrive al topic e genera una matrice con posizione e classe dei blocchetti
                         
  cout<<detected_pos_blocchetti<<endl;
                            



  int i=0;
  while (ros::ok())
  {

    //1- step reference
    if (loop_time < 5.)
    {
      while(!homing());
      //cout<<"Fine Homing\n";
    } else {
      

      for(int i=0;i<10;i++){

        if(detected_pos_blocchetti(i,0)!=0 && detected_pos_blocchetti(i,1)!=0 && detected_pos_blocchetti(i,2)!=0){
          
          motionPlan(Vector3d(detected_pos_blocchetti(i,0),detected_pos_blocchetti(i,1),detected_pos_blocchetti(i,2)),i+1);       //passo posizione e classe

          detected_pos_blocchetti(i,0)=0;
          detected_pos_blocchetti(i,1)=0;
          detected_pos_blocchetti(i,2)=0;

          sleep(8);
          while(!homing());
          sleep(5);
        }
        
      }

      
    }

    loop_time += (double)1/loop_frequency;
  
    
    //2- sine reference
//    q_des = q_des0.array() + amp.array()*(2*M_PI*freq*loop_time).array().sin();

    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

