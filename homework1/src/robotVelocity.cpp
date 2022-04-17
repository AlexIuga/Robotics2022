#define _USE_MATH_DEFINES
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

    public:

         robot_velocity(){
            sub = n.subscribe("/wheel_states", 1, &robot_velocity::savingPosition, this);
        }

        void savingPosition(const sensor_msgs::JointState::ConstPtr& msg){
            double delta_t;
            if(wheelTickPrevious[0] == 0){
                
            }
            delta_t = (msg->header.stamp.operator-(this->t_previous)).toSec();
            for(int i=0; i<4; i++){
                wheelTick[i] = msg->position[i];
                ROS_INFO("I heard : [%f]", wheelTick[i]);
            }
            /*
            ROS_INFO("t previous : [%d]", t_previous);
            ROS_INFO("t : [%d]", msg->header.stamp);
            ROS_INFO("Delta t : [%d]", delta_t);
            this->t_previous = msg->header.stamp;
            */
        }
        
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity");
    robot_velocity robot_velocity;
    ros::spin();
    return 0;
}