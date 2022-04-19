#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

class robot_velocity{
    private:
    
        double radius = 0.07; //meters
        double distanceFromCenterX = 0.2; //meters
        double distanceFromCenterY = 0.169; //meters
        double gearRatio = 0.2; //meters
        double encoderResolution = 42; //counts per revolution
        ros::Time t_previous;

        ros::NodeHandle n;
        ros::Subscriber sub;
        float wheelTickPrevious[4];
        float wheelTick[4];
        float wheelRpmFromTick[4];
        int count =223;
        double vx;
        double vy;
        double wz;
    public:

         robot_velocity(){
            sub = n.subscribe("/wheel_states", 1, &robot_velocity::savingPosition, this);
            for(int i=0; i<4; i++){
                wheelTickPrevious[i] = 0;
                wheelTick[i] = 0;
                wheelRpmFromTick[i] = 0;
            }
            vx = 0;
            vy = 0;
            wz = 0;
        }

        void savingPosition(const sensor_msgs::JointState::ConstPtr& msg){
            double delta_t;
           
            delta_t = (msg->header.stamp.operator-(this->t_previous)).toSec();
            printf("%d Calculated rad/min: ", count);
            for(int i=0; i<4; i++){
                wheelTick[i] = msg->position[i];
                if(wheelTickPrevious[0] != 0){
                    wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*60*3.14/(encoderResolution*delta_t);
                }
                vx = 1/2*radius*(wheelRpmFromTick[0]+wheelRpmFromTick[1]);
                vy = 1/2*radius*(wheelRpmFromTick[1]-wheelRpmFromTick[2]);
                wz = radius*(wheelRpmFromTick[2]-wheelRpmFromTick[0])/(2*(distanceFromCenterX+distanceFromCenterY));
                printf("%f  ", wheelRpmFromTick[i]);
                //ROS_INFO("I heard : [%f]", wheelRpmFromTick[i]);
            }
            printf("\n");
            count ++;
            this->t_previous = msg->header.stamp;
            for(int j=0; j<4; j++){
                wheelTickPrevious[j] = wheelTick[j];           }
            }
        
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity");
    robot_velocity robot_velocity;
    ros::spin();
    return 0;
}

    /*
    ROS_INFO("t previous : [%f]", t_previous);
    ROS_INFO("t : [%f]", msg->header.stamp);
    ROS_INFO("Delta t : [%f]", delta_t);
    */