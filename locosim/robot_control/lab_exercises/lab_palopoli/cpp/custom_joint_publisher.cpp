#include "include/custom_joint_publisher.h"


void send_des_jstate(const Vector6d & joint_pos, const Vector3d & gripper_joint)
{
  
    switch(use_gripper){
        case 0:
          
            
            for (int i = 0; i < joint_pos.size(); i++){
              jointState_msg_robot.data[i] = joint_pos[i];
            }

            pub_des_jstate.publish(jointState_msg_robot);
        break;
        case 1:

          
            
            jointState_msg_robot.data.resize(joint_pos.size() + gripper_joint.size() -1);
            for (int i = 0; i < joint_pos.size(); i++){
              jointState_msg_robot.data[i] = joint_pos[i];
            }
            jointState_msg_robot.data[6] = gripper_joint(0);
            jointState_msg_robot.data[7] = gripper_joint(1);

            pub_des_jstate.publish(jointState_msg_robot);
          
        break;
        case 2:
          

            jointState_msg_robot.data.resize(joint_pos.size() + gripper_joint.size());
            for (int i = 0; i < joint_pos.size(); i++){
              jointState_msg_robot.data[i] = joint_pos[i];
            }
            jointState_msg_robot.data[6] = gripper_joint(0);
            jointState_msg_robot.data[7] = gripper_joint(1);
            jointState_msg_robot.data[8] = gripper_joint(2);

            pub_des_jstate.publish(jointState_msg_robot);
          
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


void detect(const custom_msgs::Coord::ConstPtr& msg){
  //funzione che si sottoscrive al topic e riceve la matrice coi blocchetti

  string cl;
  double x,y,z,roll,pitch,yaw;
  int cl_int;
  Matrix3d Rot;
  
  x=msg->x;
  y=msg->y;
  z=msg->z;
  //roll=msg->roll;
  //pitch=msg->pitch;
  //yaw=msg->yaw;
  std::vector<float> roto (msg->R.begin(), msg->R.end());

  Rot << roto[0],roto[1],roto[2],
         roto[3],roto[4],roto[5],
         roto[6],roto[7],roto[8]; 

  cout<<rotm2eulFDR(Rot).transpose()<<endl;
  cl=msg->cl;
  cout<<msg->cl<<endl;
  
  if (strcmp(cl.c_str(),"1")==0)
    cl_int=0;
  if (strcmp(cl.c_str(),"2")==0)
    cl_int=1;
  if (strcmp(cl.c_str(),"3")==0)
    cl_int=2;
  if (strcmp(cl.c_str(),"4")==0)
    cl_int=3;
  if (strcmp(cl.c_str(),"5")==0)
    cl_int=4;
  if (strcmp(cl.c_str(),"6")==0)
    cl_int=5;
  if (strcmp(cl.c_str(),"7")==0)
    cl_int=6;
  if (strcmp(cl.c_str(),"8")==0)
    cl_int=7;
  if (strcmp(cl.c_str(),"9")==0)
    cl_int=8;
  if (strcmp(cl.c_str(),"6-FILLER")==0)
    cl_int=9;
  if (strcmp(cl.c_str(),"9-FILLER")==0)
    cl_int=10;
         

  if(detected_pos_blocchetti(cl_int,0)==0 && detected_pos_blocchetti(cl_int,1)==0 && detected_pos_blocchetti(cl_int,2)==0 && detection){
    detected_pos_blocchetti(cl_int,0)=x+0.015;
    detected_pos_blocchetti(cl_int,1)=y;
    detected_pos_blocchetti(cl_int,2)=z;
    detected_pos_blocchetti(cl_int,3)=rotm2eulFDR(Rot)(0);
    detected_pos_blocchetti(cl_int,4)=rotm2eulFDR(Rot)(1);
    detected_pos_blocchetti(cl_int,5)=M_PI-rotm2eulFDR(Rot)(2);
  }

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


Vector6d rotate(Vector6d q, Vector3d rot){
  Vector6d rotation;
  rotation << 0.0,0.0,0.0,-rot(0),-rot(1),-rot(2);
  
  return q+rotation;
}


void motionPlan(Vector6d pos_blocchetto,int classe){

  
  Vector3d posIniziale=ur5Direct(q_des0);
  cout<<posIniziale.transpose()<<endl;
  cout<<rotm2eulFDR(Re)<<endl;
  //cout<<approx(Re)<<endl;

  posIniziale(1)*=-1;

  detection = false;


  Vector6d q_temp;
 
  q_temp=moveTo(posIniziale,pos_blocchetto.head(3)-up_down_di,rotm2eulFDR(Re),Vector3d(0.,0.,pos_blocchetto(5)));  //muovo in base ai cubetti nella matrice
  
  sleep(2);
  cout<<ur5Direct(q_temp).transpose()<<endl;
  q_temp=moveTo(ur5Direct(q_temp),pos_blocchetto.head(3),Vector3d(0.,0.,0.),Vector3d(0.,0.,pos_blocchetto(5)));   //abbasso braccio per prendere cubetto
  sleep(1);

  apri=false;                 //chiudi gripper
  while(!grasp(q_temp));


  sleep(2);
  q_temp=moveTo(ur5Direct(q_temp),ur5Direct(q_temp)-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));  //alzo il cubetto
  sleep(1);


  switch(classe){
    case 0:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0)); 
    break;
    case 1:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));        //muovo da pos_blocchetto a final stand 
    break;
    case 2:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 3:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 4:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 5:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 6:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 7:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 8:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 9:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
    case 10:
      q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));
    break;
  }
   

  sleep(3);
  q_temp=moveTo(ur5Direct(q_temp),final_stand+Vector3d(0.0,0.0,0.04), Vector3d(0.,0.0,0), Vector3d(0.,0.0,0));   //abbasso il cubetto

  apri=true;                            //apri gripper
  while(!grasp(q_temp));

  sleep(2);
  q_temp=moveTo(ur5Direct(q_temp),final_stand-up_down_di, Vector3d(0.,0.0,0.0), Vector3d(0.,0.0,0.0));       //alzo cubetto

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
  q_prova = Thresult.row(7);

  //MatrixXd differentialTH=invDiffKinematicControlSimComplete(pos_iniziale,pos_blocchetto,rotm2eulFDR(Re),Vector3d(0.0,0.0,0.0),q_prova,Kp,Kphi, TMIN, TMAX, DELTAT);
  MatrixXd differentialTH=invDiffKinematicControlSimComplete(pos_iniziale,pos_finale,rot_iniziale,rot_finale,q_prova,Kp,Kphi, TMIN, TMAX, DELTAT);


  for(int i=0;i<differentialTH.rows();i++){
    q_prova << differentialTH(i,0),differentialTH(i,1),differentialTH(i,2),differentialTH(i,3),differentialTH(i,4),differentialTH(i,5);
    //q_prova = secondOrderFilter(q_prova, loop_frequency, 5.);
    if(i==differentialTH.rows()-1){
      Vector6d rot;
      //rot << 0.0,0.0,0.0,-rot_finale(0),-rot_finale(1),-rot_finale(2);
      //rot << 0.0,0.0,0.0,0.0,0.0,-rot_finale(2);
      //q_prova+=rot;
      q_prova=rotate(q_prova,rot_finale);
    }
    
    if(apri){
      send_des_jstate(q_prova,open_gripper(60));
    }else{
      send_des_jstate(q_prova,open_gripper(10));
    }
    
  }

  

  //return differentialTH.row(differentialTH.rows()-1);
  return q_prova; 
}

bool grasp(Vector6d q){
  sleep(2);
  switch(apri){
    case false:
      send_des_jstate(q,open_gripper(10));
    break;
    case true:
      send_des_jstate(q,open_gripper(60));
    break;
  }
  
  return 1;
}





int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "custom_joint_publisher");
  ros::NodeHandle node;



  
  pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
  

  ros::Rate loop_rate(loop_frequency);

  
  //jointState_msg_robot.data.resize(6);

  
  Vector6d amp;
  Vector6d freq;
  amp << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;
  freq << 0.2, 0.0, 0.0, 0.0, 0., 0.0;
  
  q_des0 << -0.321997,-0.919283,-2.61359,-1.17952,-1.5708,-1.2488;
  initFilter(q_des0);


  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/coordinates", 1000, detect);
        

  
  /*
  detected_pos_blocchetti <<  0.7,0.5,0.85,0.,0.,2.66,
                              0.0,0.0,0.0,0.,0.,0.,
                              0.0,0.0,0.0,0.,0.,0.,
                              0.0,0.0,0.0,0.,0.,0.;
                             
     */               
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
      cout<<detected_pos_blocchetti<<endl;

      for(int i=0;i<11;i++){

        //cout<<detected_pos_blocchetti(i,0)<<" "<<detected_pos_blocchetti(i,1)<<" "<<detected_pos_blocchetti(i,2)<<endl;

        if((detected_pos_blocchetti(i,0)!=0 || detected_pos_blocchetti(i,1)!=0 || detected_pos_blocchetti(i,2)!=0) && (detected_pos_blocchetti(i,0)!=-1 && detected_pos_blocchetti(i,1)!=-1 && detected_pos_blocchetti(i,2)!=-1)){
          
          detected_pos_blocchetti.row(i).head(3)-=shift;
          detected_pos_blocchetti(i,1) *= -1;

          motionPlan(detected_pos_blocchetti.row(i),i);       //passo posizione e classe
          
          detected_pos_blocchetti(i,0)=-1;
          detected_pos_blocchetti(i,1)=-1;
          detected_pos_blocchetti(i,2)=-1;
          detected_pos_blocchetti(i,3)=0;
          detected_pos_blocchetti(i,4)=0;
          detected_pos_blocchetti(i,5)=0;
          

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
