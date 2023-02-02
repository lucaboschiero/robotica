#include "ros/ros.h"
#include <detection_msgs/BoundingBoxes.h>
#include <std_msgs/String.h>
#include <cstring>
#include "include/ur5kinematics.h"
#include <custom_msgs/Points.h>
#include <stdexcept>
#include <iostream>

using namespace std;

string c;
ros::Publisher pub;
ros::Subscriber sub;

    void chatterCallback(const detection_msgs::BoundingBoxes::ConstPtr& msg)
{
    custom_msgs::Points point;
    int u;
    int v;


    for(int i=0;i<10;i++){
    //cout<<i<<endl;
            if(msg->bounding_boxes[i].xmin>50 && msg->bounding_boxes[i].xmin<2000){
            //cout<<msg->bounding_boxes[i].xmin<<endl;
                point.u=((int)(msg->bounding_boxes[i].xmin)+(int)(msg->bounding_boxes[i].xmax))/2;
                point.v=((int)(msg->bounding_boxes[i].ymin)+(int)(msg->bounding_boxes[i].ymax))/2;
                point.Class=msg->bounding_boxes[i].Class;
                //cout<<"punti: "<<point;

                //pis.data=to_string(u)+" "+to_string(v)+" "+c;
                //pis.data = point;
                pub.publish(point);
            }
    }
            /*u=((int)(msg->bounding_boxes[0].xmin)+(int)(msg->bounding_boxes[0].xmax))/2;
            v=((int)(msg->bounding_boxes[0].ymin)+(int)(msg->bounding_boxes[0].ymax))/2;
            c=msg->bounding_boxes[0].Class;
            point=to_string(u)+" "+to_string(v)+" "+c;
            cout<<"punti: "<<point;

            //pis.data=to_string(u)+" "+to_string(v)+" "+c;
            pis.data = point;
            pub.publish(pis);*/

}


int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    pub= n.advertise<custom_msgs::Points>("/points", 1000);
    ros::Subscriber sub = n.subscribe("/yolov5/detections", 1000, chatterCallback);
    /*ros::Publisher pubgr= n.advertise<std_msgs::String>("/gripper_controller_cmd", 10);
    std_msgs::String str;

    str.data="close";
    pubgr.publish(str);*/

    ros::spin();


    return 0;
}


