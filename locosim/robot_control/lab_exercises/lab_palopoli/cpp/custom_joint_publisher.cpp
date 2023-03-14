#include "custom_joint_publisher.h"
#include <math.h>
#include "include/ur5kinematics.h"
#include <std_msgs/String.h>
#include <cstring>
#include <custom_msgs/Coord.h>
#include <custom_msgs/PosRobot.h>
#include <iostream>


void send_des_jstate(const Vector6d & joint_pos, const Vector3d & gripper_joint)
{
    //std::cout << "q_des " << joint_pos.transpose() <<" "<<gripper_joint.transpose() << std::endl;
    switch(use_gripper){
        case 0:
          if(real_robot){
            
            for (int i = 0; i < joint_pos.size(); i++){
              jointState_msg_robot.data[i] = joint_pos[i];
            }

            pub_des_jstate.publish(jointState_msg_robot);
          }else{
            
            for (int i = 0; i < joint_pos.size(); i++){ 
              jointState_msg_sim.position[i] = joint_pos[i];
              jointState_msg_sim.velocity[i] = 0.0;
              jointState_msg_sim.effort[i] = 0.0;
            }

            pub_des_jstate.publish(jointState_msg_sim);
          }
        break;
        case 1:

          if(real_robot){
            
            jointState_msg_robot.data.resize(joint_pos.size() + gripper_joint.size() -1);
            for (int i = 0; i < joint_pos.size(); i++){
              jointState_msg_robot.data[i] = joint_pos[i];
            }
            jointState_msg_robot.data[6] = gripper_joint(0);
            jointState_msg_robot.data[7] = gripper_joint(1);

            pub_des_jstate.publish(jointState_msg_robot);
          }else{

            for (int i = 0; i < joint_pos.size(); i++){ 
              jointState_msg_sim.position[i] = joint_pos[i];
              jointState_msg_sim.velocity[i] = 0.0;
              jointState_msg_sim.effort[i] = 0.0;
            }
            for(int i=joint_pos.size();i<joint_pos.size() + gripper_joint.size() -1;i++){
              jointState_msg_sim.position[i] = gripper_joint[i];
              jointState_msg_sim.velocity[i] = 0.0;
              jointState_msg_sim.effort[i] = 0.0;
            }

            pub_des_jstate.publish(jointState_msg_sim);
          }
        break;
        case 2:
          if(real_robot){

            jointState_msg_robot.data.resize(joint_pos.size() + gripper_joint.size());
            for (int i = 0; i < joint_pos.size(); i++){
              jointState_msg_robot.data[i] = joint_pos[i];
            }
            jointState_msg_robot.data[6] = gripper_joint(0);
            jointState_msg_robot.data[7] = gripper_joint(1);
            jointState_msg_robot.data[8] = gripper_joint(2);

            pub_des_jstate.publish(jointState_msg_robot);
          }else{

            for (int i = 0; i < joint_pos.size(); i++){ 
              jointState_msg_sim.position[i] = joint_pos[i];
              jointState_msg_sim.velocity[i] = 0.0;
              jointState_msg_sim.effort[i] = 0.0;
            }
            for(int i=joint_pos.size();i<joint_pos.size() + gripper_joint.size();i++){
              jointState_msg_sim.position[i] = gripper_joint[i];
              jointState_msg_sim.velocity[i] = 0.0;
              jointState_msg_sim.effort[i] = 0.0;
            }

            pub_des_jstate.publish(jointState_msg_sim);
          }
        break;

    }
}



void initFilter(const Vector6d & joint_pos)
{
        filter_1 = joint_pos;
        filter_2 = joint_pos;
}

Vector6d secondOrderFilter(const Vector6d & input, const double rate, const double settling_time)
{

        double dt = 1 / rate;
        double gain =  dt / (0.1*settling_time + dt);
        filter_1 = (1 - gain) * filter_1 + gain * input;
        filter_2 = (1 - gain) * filter_2 + gain *filter_1;
        return filter_2;
}


Vector3d open_gripper(double d){

    Vector3d q;
    double opening;
    double D0 = 40;
    double L = 60;

    switch(use_gripper){
      case 0:

        q << -100,-100,-100;

        break;
      case 1:

        opening = atan2(0.5*(d - D0), L);
        q << opening, opening, 0;

        break;
      case 2:

        opening = (d - 22) / 108 * -M_PI + M_PI;
        q << opening, opening, opening;

        break;
      default:

        cout<<"ERROR"<<endl;
        exit(1); 
    }

    return q;
}


void detect(const custom_msgs::Coord::ConstPtr& msg){
  //funzione che si sottoscrive al topic e riceve la matrice coi blocchetti

  string cl;
  double x,y,z;
  int cl_int;

  x=msg->x;
  y=msg->y;
  z=msg->z;
  //cl=msg->cl;
  //cl--;
  
  if (strcmp(cl.c_str(),"1")==0)
    cl_int=1;
  if (strcmp(cl.c_str(),"2")==0)
    cl_int=2;
  if (strcmp(cl.c_str(),"3")==0)
    cl_int=3;
  if (strcmp(cl.c_str(),"4")==0)
    cl_int=4;
  if (strcmp(cl.c_str(),"5")==0)
    cl_int=5;
  if (strcmp(cl.c_str(),"6")==0)
    cl_int=6;
  if (strcmp(cl.c_str(),"7")==0)
    cl_int=7;
  if (strcmp(cl.c_str(),"8")==0)
    cl_int=8;
  if (strcmp(cl.c_str(),"9")==0)
    cl_int=9;
  if (strcmp(cl.c_str(),"6-FILLET")==0)
    cl_int=10;
  if (strcmp(cl.c_str(),"9-FILLET")==0)
    cl_int=11;
         

  if(detected_pos_blocchetti(cl_int,0)==0 && detected_pos_blocchetti(cl_int,1)==0 && detected_pos_blocchetti(cl_int,2)==0){
    detected_pos_blocchetti(cl_int,0)=x;
    detected_pos_blocchetti(cl_int,1)=y;
    detected_pos_blocchetti(cl_int,2)=z;
  }

}

void posAttuale(const custom_msgs::PosRobot::ConstPtr& msg){
       pos_attuale << msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5];
}


void motionPlan(Vector3d pos_blocchetto,int classe){

  Vector3d posIniziale=ur5Direct(q_des0);

  Vector6d q_temp;
  q_temp=moveTo(posIniziale,pos_blocchetto,Vector3d(M_PI,0.0,0.0),Vector3d(M_PI,0.0,0.0));  //muovo in base ai cubetti nella matrice
  

  apri=false;                 //chiudi gripper
  while(!grasp(q_temp));

  sleep(1);
  q_temp=moveTo(ur5Direct(q_temp),ur5Direct(q_temp)-Vector3d(0.0,0.0,0.1), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));  //alzo il cubetto
  sleep(1);

  switch(classe){
    case 1:
      q_temp=moveTo(ur5Direct(q_temp),Vector3d(0.2,0.3,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));        //muovo da pos_blocchetto a final stand (non so dove sia ma vabbe)
    break;
    case 2:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 3:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 4:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 5:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 6:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 7:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 8:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 9:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
    case 10:
      q_temp=moveTo(pos_blocchetto-Vector3d(0.0,0.0,0.2),Vector3d(0.2,0.5,HTAVOLO-0.15), Vector3d(M_PI,0.0,0.0), Vector3d(M_PI,0.0,0.0));
    break;
  }

  apri=true;                            //apri gripper
  while(!grasp(q_temp));

 

}



bool homing(){
  
  
  initFilter(q_des0);

  send_des_jstate(q_des0,open_gripper(50));

  return 1;
}

Vector6d moveTo(Vector3d pos_iniziale,Vector3d pos_finale, Vector3d rot_iniziale, Vector3d rot_finale){
  
  //pos_blocchetto=pos_blocchetto-shift;

  Matrix86d Thresult=ur5Inverse(pos_iniziale);
  Vector6d q_prova;
  q_prova << Thresult(7,0), Thresult(7,1), Thresult(7,2), Thresult(7,3), Thresult(7,4), Thresult(7,5);
  initFilter(q_prova);

  //MatrixXd differentialTH=invDiffKinematicControlSimComplete(pos_iniziale,pos_blocchetto,rotm2eulFDR(Re),Vector3d(0.0,0.0,0.0),q_prova,Kp,Kphi, TMIN, TMAX, DELTAT);
  MatrixXd differentialTH=invDiffKinematicControlSimComplete(pos_iniziale,pos_finale,rot_iniziale,rot_finale,q_prova,Kp,Kphi, TMIN, TMAX, DELTAT);


  for(int i=0;i<differentialTH.rows();i++){
    q_prova << differentialTH(i,0),differentialTH(i,1),differentialTH(i,2),differentialTH(i,3),differentialTH(i,4),differentialTH(i,5);
    //q_prova = secondOrderFilter(q_prova, loop_frequency, 5.);
    if(apri){
      send_des_jstate(q_prova,open_gripper(50));
    }else{
      send_des_jstate(q_prova,open_gripper(0));
    }
    
  }

  return differentialTH.row(differentialTH.rows()-1);

}

bool grasp(Vector6d q){
  sleep(1);
  switch(apri){
    case false:
      send_des_jstate(q,open_gripper(0));
    break;
    case true:
      send_des_jstate(q,open_gripper(50));
    break;
  }
  
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

  
  Vector6d amp;
  Vector6d freq;
  amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;
  freq << 0.2, 0.0, 0.0, 0.0, 0., 0.0;


  //globali
  Kp = Matrix3d::Identity()*5;
  Kphi = Matrix3d::Identity()*10;
  //q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
  //q_des0 << -0.32 ,  -0.78002  ,  -2.56007  , -1.63004 ,    -1.57004 , 3.49009;
  q_des0 << -0.321997 ,  -0.919283,   -2.61359 ,       -1.17952 ,       -1.5708 ,  -1.2488;
  shift=Vector3d(0.5,0.35,0.12);



  //ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe("/coordinates", 1000, detect);
        //funzione che si sottoscrive al topic e genera una matrice con posizione e classe dei blocchetti

  //ros::Subscriber subPos=n.subscribe("/posattualerobot", 1000, posAttuale);
  
  detected_pos_blocchetti <<  0.5,0.35,0.85,
                              0.0,0.0,0.0,
                              0.0,0.0,0.0,
                              0.0,0.0,0.0;
                              
                         
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
      //cout<<pos_attuale.transpose()<<endl;
      //cout<<detected_pos_blocchetti<<endl;

      for(int i=0;i<11;i++){

        //cout<<detected_pos_blocchetti(i,0)<<" "<<detected_pos_blocchetti(i,1)<<" "<<detected_pos_blocchetti(i,2)<<endl;

        if((detected_pos_blocchetti(i,0)!=0 || detected_pos_blocchetti(i,1)!=0 || detected_pos_blocchetti(i,2)!=0) && (detected_pos_blocchetti(i,0)!=-1 && detected_pos_blocchetti(i,1)!=-1 && detected_pos_blocchetti(i,2)!=-1)){
          
          detected_pos_blocchetti.row(i)-=shift;

          motionPlan(Vector3d(detected_pos_blocchetti(i,0),-detected_pos_blocchetti(i,1),detected_pos_blocchetti(i,2)),i+1);       //passo posizione e classe
          
          detected_pos_blocchetti(i,0)=-1;
          detected_pos_blocchetti(i,1)=-1;
          detected_pos_blocchetti(i,2)=-1;

          

          sleep(2);
          while(!homing());
          sleep(3);
          

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
