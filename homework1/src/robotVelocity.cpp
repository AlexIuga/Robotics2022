#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"

class robotVelocity{
    private:
    
        double radius = 0.07; //meters
        double distanceFromCenterX = 0.2; //meters
        double distanceFromCenterY = 0.169; //meters
        double gearRatio = 0.2; //meters
        double encoderResolution = 42; //counts per revolution
        ros::Time t_previous;

        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        float wheelTickPrevious[4];
        float wheelTick[4];
        float wheelRpmFromTick[4];
        int count =223;
        double vx;
        double vy;
        double wz;
    public:

         robotVelocity(){
            this->sub = this->n.subscribe("/wheel_states", 1, &robotVelocity::calculateRobotVelocity, this);
            this->pub = this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
            for(int i=0; i<4; i++){
                wheelTickPrevious[i] = 0;
                wheelTick[i] = 0;
                wheelRpmFromTick[i] = 0;
            }
            vx = 0;
            vy = 0;
            wz = 0;
        }

        void calculateRobotVelocity(const sensor_msgs::JointState::ConstPtr& msg){
            double delta_t;
           
            delta_t = (msg->header.stamp.operator-(this->t_previous)).toSec();
            printf("%d Calculated rad/min: ", count);
            for(int i=0; i<4; i++){
                wheelTick[i] = msg->position[i];
                if(wheelTickPrevious[0] != 0){
                    wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*3.14*gearRatio/(encoderResolution*delta_t);
                }
                printf("%f  ", wheelRpmFromTick[i]);
            }
            vx = radius*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
            vy = radius*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
            wz = radius*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(distanceFromCenterX+distanceFromCenterY));
            printf("\n");
            printf("Calculated vx vy w: %f  %f  %f ", vx, vy, wz);
            printf("\n");

            count ++;
            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.frame_id = "base_link";
            vel_msg.header.seq = msg->header.seq;
            vel_msg.header.stamp = msg->header.stamp;

            vel_msg.twist.linear.x = vx;
            vel_msg.twist.linear.y = vy;
            vel_msg.twist.linear.z = 0;

            vel_msg.twist.angular.z = wz;
            vel_msg.twist.angular.y = 0;
            vel_msg.twist.angular.x = 0;

            pub.publish(vel_msg);
            this->t_previous = msg->header.stamp;
            for(int j=0; j<4; j++){
                wheelTickPrevious[j] = wheelTick[j];           }
            }
        
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity");
    robotVelocity robotVelocity;
    ros::spin();
    return 0;
}

    /*
    ROS_INFO("t previous : [%f]", t_previous);
    ROS_INFO("t : [%f]", msg->header.stamp);
    ROS_INFO("Delta t : [%f]", delta_t);
    */
    //ROS_INFO("I heard : [%f]", wheelRpmFromTick[i]);