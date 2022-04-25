#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "homework1/wheelSpeed.h"

class robotVelocity{
    private:

        ros::Time t_previous;
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub_wheels;

        double radius = 0.07; //meters
        double distanceFromCenterX = 0.2; //meters
        double distanceFromCenterY = 0.169; //meters
        double gearRatio = 0.2; //meters
        double encoderResolution = 42; //counts per revolution
        double vx;
        double vy;
        double wz;
        
        float wheelTickPrevious[4];
        float wheelTick[4];
        float wheelRpmFromTick[4];

        int count =223;
        
    public:

         robotVelocity(){
            this->sub = this->n.subscribe("/wheel_states", 1, &robotVelocity::calculateRobotVelocity, this);
            this->pub = this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

            //publisher of wheel custom message on wheels_rpm topic
            this->pub_wheels = this->n.advertise<homework1::wheelSpeed>("/wheels_rpm", 1);

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
            }

            //to delete
            printf("wheel speed read from bag and converted to rpm");
            printf("\n");
            printf("Calculated wheel speeds: %f  %f  %f  %f", wheelRpmFromTick[0], wheelRpmFromTick[1], wheelRpmFromTick[2], wheelRpmFromTick[3]);
            printf("\n");


            vx = radius*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
            vy = radius*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
            wz = radius*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(distanceFromCenterX+distanceFromCenterY));


            //to remove
            printf("\n");
            printf("Calculated vx vy w: %f  %f  %f ", vx, vy, wz);


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

            //inverts formulae and recalculates wheel speeds
            //publishWheelSpeed(vel_msg.header,vx,vy,wz);
            homework1::wheelSpeed wheel_msg;
            wheel_msg.header = vel_msg.header;
            wheel_msg.rpm_fl=(vx-vy-(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_fr=(vx+vy+(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_rr=(vx-vy+(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_rl=(vx+vy-(distanceFromCenterX+distanceFromCenterY)*wz)/radius;

            //to remove
            //printf("\n");
            printf("Calculated wheel speeds: %f  %f  %f  %f", wheel_msg.rpm_fl, wheel_msg.rpm_fr, wheel_msg.rpm_rr, wheel_msg.rpm_rl);
            //printf("\n");
            ROS_INFO("send msg = %d", wheel_msg.header.seq);   // prints to see if header is right
            printf("\n");

            //publishes wheel speeds
            this->pub_wheels.publish(wheel_msg);



            this->t_previous = msg->header.stamp;
            for(int j=0; j<4; j++){
                wheelTickPrevious[j] = wheelTick[j];           
            }


        }
        /*
        void publishWheelSpeed(double vx,double vy,double wz){
            homework1::wheelSpeed wheel_msg;
            //I have to also assign header, not sure if it-s right
            wheel_msg.header=header;
            wheel_msg.rpm_fl=(vx-vy-(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_fr=(vx+vy+(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_rr=(vx-vy+(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_rl=(vx+vy-(distanceFromCenterX+distanceFromCenterY)*wz)/radius;

            //to remove
            printf("\n");
            printf("Calculated wheel speeds: %f  %f  %f  %f", wheel_msg.rpm_fl, wheel_msg.rpm_fr, wheel_msg.rpm_rr, wheel_msg.rpm_rl);
            printf("\n");

            this->pub_wheels.publish(wheel_msg);

        }*/
        
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity");
    robotVelocity robotVelocity;
    ros::spin();
    return 0;
}

