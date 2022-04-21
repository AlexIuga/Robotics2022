#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <dynamic_reconfigure/server.h>
#include <homework1/parametersConfig.h>

enum integrationTypes {EULER, RK};

class robotOdometry{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;

        double xPrevious;
		double yPrevious;
		double thetaPrevious;
		ros::Time tPrevious;

        dynamic_reconfigure::Server<homework1::parametersConfig> server;
        integrationTypes chosenMethod; 
        int count = 143997;

    public :
        robotOdometry(){
            this->sub = n.subscribe("/cmd_vel", 1, &robotOdometry::calculateRobotOdometry, this);
            this->pub = n.advertise<nav_msgs::Odometry>("/odom",1);

            dynamic_reconfigure::Server<homework1::parametersConfig>::CallbackType callbackObject;
            callbackObject = boost::bind(&robotOdometry::integrationMethodHandler, this, _1, _2);
            this->server.setCallback(callbackObject);

            chosenMethod = EULER;

            xPrevious = 0;
            yPrevious = 0;
            thetaPrevious = 0;

        }

        void calculateRobotOdometry(const geometry_msgs::TwistStamped::ConstPtr& msg){
            double xNext, yNext, thetaNext, deltaT;

            deltaT = (msg->header.stamp.operator-(this->tPrevious)).toSec();

            if(this->chosenMethod == EULER){
                xNext = this->xPrevious + (msg->twist.linear.x*cos(thetaPrevious)-msg->twist.linear.y*sin(thetaPrevious))*deltaT;
                yNext = this->yPrevious + (msg->twist.linear.x*sin(thetaPrevious)+msg->twist.linear.y*cos(thetaPrevious))*deltaT;
                 printf("Applying EULER");
                printf("\n");
            }else{
                xNext = this->xPrevious + (msg->twist.linear.x*cos(thetaPrevious + msg->twist.angular.z*deltaT*0.5)-msg->twist.linear.y*sin(thetaPrevious + msg->twist.angular.z*deltaT*0.5))*deltaT;
                yNext = this->yPrevious + (msg->twist.linear.x*sin(thetaPrevious + msg->twist.angular.z*deltaT*0.5)+msg->twist.linear.y*cos(thetaPrevious + msg->twist.angular.z*deltaT*0.5))*deltaT;
                printf("Applying RUNGE KUTTA");
                printf("\n");
            }

            
            thetaNext = this->thetaPrevious + msg->twist.angular.z*deltaT;
            
            printf("seq %d", count);
            printf("\n");
            printf("X position: %f ", xNext);
            printf("\n");
            printf("Y position: %f ", yNext);
            printf("\n");
            printf("Theta position: %f ", thetaNext);
            printf("\n");
            count++;
            this->xPrevious = xNext;
            this->yPrevious = yNext;
            this->thetaPrevious = thetaNext;
            this->tPrevious = msg->header.stamp;
        }

        void integrationMethodHandler(homework1::parametersConfig &config, uint32_t level){
            if(config.integrationMethod == 0){
                this->chosenMethod = EULER;
            }else{
                this->chosenMethod = RK;
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    robotOdometry robotOdometry;
    ros::spin();
    return 0;
    }