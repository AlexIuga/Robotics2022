#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH    

class parametersCalibration{
    private:

        ros::NodeHandle n;

        tf2::Quaternion thetaQuaternion;

        double vx;
        double vy;
        double wz;
        
        float wheelTickPrevious[4];
        float wheelTick[4];
        float wheelRpmFromTick[4];

        double radius = 0.07; //meters
        double distanceFromCenterX = 0.2; //meters
        double distanceFromCenterY = 0.169; //meters
        double encoderResolution = 42; //counts per revolution
        double gearRatio = 0.2; //meters

        double xNext[200], yNext[200], thetaNext[200], xPrevious[200], yPrevious[200], thetaPrevious[200];
        double xMean = 0.0, yMean = 0.0, thetaMean = 0.0, delta_t;

        double radiusCalibrated = radius;
        double lCalibrated = distanceFromCenterX;
        double wCalibrated = distanceFromCenterY;
        double nCalibrated = encoderResolution;
        double shapeCalibrated = distanceFromCenterX + distanceFromCenterY;

    public:
        void calculatingBestRadius(){
            ros::Time t_previous;
            rosbag::Bag bag;
            const std::string bagName1 = "bag3.bag";
            bag.open(bagName1, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back(std::string("wheel_states"));
            topics.push_back(std::string("robot/pose"));

            rosbag::View view(bag, rosbag::TopicQuery(topics));

              
            double bestR;
            double cfRadius[200];
            double rParamaters[200];

            for(int i=0; i<4; i++){
                wheelRpmFromTick[i] = 0.0;
                wheelTick[i] = 0.0;
                wheelTickPrevious[i] = 0.0;
            }

            for(int i=0; i<200; i++){
                xPrevious[i] = 0.0;
                yPrevious[i] = 0.0;
                thetaPrevious[i] = 0.0;
                xNext[i] = 0.0;
                yNext[i] = 0.0;
                thetaNext[i] = 0.0;
                cfRadius[i] = 0.0;
            }

            for(rosbag::MessageInstance const m: rosbag::View(bag)){
                
                if (m.getTopic() == "/robot/pose"|| ("/" + m.getTopic() == "/robot/pose")){
                    geometry_msgs::PoseStamped::ConstPtr msg_pose = m.instantiate<geometry_msgs::PoseStamped>();
                    if(msg_pose != NULL){
                       calculatingCostFunction(msg_pose, cfRadius, 200);
                    }
                }
                    
                if (m.getTopic() == "/wheel_states" || ("/" + m.getTopic() == "/wheel_states")){
                    sensor_msgs::JointState::ConstPtr msg_wheel = m.instantiate<sensor_msgs::JointState>();
                    if(msg_wheel != NULL){
                        delta_t = (msg_wheel->header.stamp.operator-(t_previous)).toSec();
                        for(int i=0; i<4; i++){
                            wheelTick[i] = msg_wheel->position[i];
                            if(wheelTickPrevious[0] != 0){
                                wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*3.14*gearRatio/(encoderResolution*delta_t);
                            }
                        }
                        for(int i=0; i<200; i++){

                            rParamaters[i] = radiusCalibrated;

                            vx = radiusCalibrated*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
                            vy = radiusCalibrated*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
                            wz = radiusCalibrated*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(shapeCalibrated));

                            xNext[i] = xPrevious[i] + (vx*cos(thetaPrevious[i] + wz*delta_t*0.5)-vy*sin(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            yNext[i] = yPrevious[i] + (vx*sin(thetaPrevious[i] + wz*delta_t*0.5)+vy*cos(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            thetaNext[i] = thetaPrevious[i] + wz*delta_t;

                            t_previous = msg_wheel->header.stamp;
                            xPrevious[i] = xNext[i];
                            yPrevious[i] = yNext[i];
                            thetaPrevious[i] = thetaNext[i];

                            if(i<100){
                                radiusCalibrated = radiusCalibrated + 0.0001;
                            }else{
                                if(i == 100){
                                    radiusCalibrated = radius - 0.0001;
                                }
                                radiusCalibrated = radiusCalibrated - 0.0001;
                            }
                        }
                        radiusCalibrated = radius;
                        for(int j=0; j<4; j++){
                            wheelTickPrevious[j] = wheelTick[j];           
                        }
                    }
                }
            }
            bestR = rParamaters[0];
            int bestIndex = 0;
            for(int i=0; i<199; i++){
                //printf("Parameter: %f Cf: %f\n", rParamaters[i], cfRadius[i]);
                if(cfRadius[i+1]<cfRadius[bestIndex]){
                    bestR = rParamaters[i+1];
                    bestIndex = i+1;
                }
            }
            printf("Calibrated r: %f\n", bestR);
            radiusCalibrated = bestR;
            bag.close();
    
        }

        void calculatingBestDistancesFromCenter(){
            rosbag::Bag bag;
            ros::Time t_previous;
            const std::string bagName1 = "bag3.bag";
            bag.open(bagName1, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back(std::string("wheel_states"));
            topics.push_back(std::string("robot/pose"));

            rosbag::View view(bag, rosbag::TopicQuery(topics));

            double bestShape;
            double cfshape[200];
            double shapeParameters[200];

            xMean = 0.0;
            yMean = 0.0;
            thetaMean = 0.0;

            for(int i=0; i<4; i++){
                wheelRpmFromTick[i] = 0.0;
                wheelTick[i] = 0.0;
                wheelTickPrevious[i] = 0.0;
            }
            for(int i=0; i<200; i++){
                xPrevious[i] = 0.0;
                yPrevious[i] = 0.0;
                thetaPrevious[i] = 0.0;
                xNext[i] = 0.0;
                yNext[i] = 0.0;
                thetaNext[i] = 0.0;
                cfshape[i] = 0.0;
                
            }

            for(rosbag::MessageInstance const m: rosbag::View(bag)){
                
                if (m.getTopic() == "/robot/pose"|| ("/" + m.getTopic() == "/robot/pose")){
                    
                    geometry_msgs::PoseStamped::ConstPtr msg_pose = m.instantiate<geometry_msgs::PoseStamped>();
                    //printf("Robot Pose: %f\n", msg_pose->header.stamp.toSec());
                    if(msg_pose != NULL && wheelTick[0] != 0.0){
                       calculatingCostFunction(msg_pose, cfshape, 200);
                    }
                }
                    
                if (m.getTopic() == "/wheel_states" || ("/" + m.getTopic() == "/wheel_states")){
                    sensor_msgs::JointState::ConstPtr msg_wheel = m.instantiate<sensor_msgs::JointState>();
                    //printf("Robot Wheel: %f\n", msg_wheel->header.stamp.toSec());
                    if(msg_wheel != NULL){
                        delta_t = (msg_wheel->header.stamp.operator-(t_previous)).toSec();
                        for(int i=0; i<4; i++){
                            wheelTick[i] = msg_wheel->position[i];
                            if(wheelTickPrevious[0] != 0){
                                wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*3.14*gearRatio/(encoderResolution*delta_t);
                            }
                        }
                        for(int i=0; i<200; i++){

                            shapeParameters[i] = shapeCalibrated;

                            vx = radius*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
                            vy = radius*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
                            wz = radius*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(shapeCalibrated));

                            xNext[i] = xPrevious[i] + (vx*cos(thetaPrevious[i] + wz*delta_t*0.5)-vy*sin(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            yNext[i] = yPrevious[i] + (vx*sin(thetaPrevious[i] + wz*delta_t*0.5)+vy*cos(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            thetaNext[i] = thetaPrevious[i] + wz*delta_t;

                            t_previous = msg_wheel->header.stamp;
                            xPrevious[i] = xNext[i];
                            yPrevious[i] = yNext[i];
                            thetaPrevious[i] = thetaNext[i];

                            if(i<100){
                                shapeCalibrated = shapeCalibrated + 0.001;
                            }else{
                                if(i == 100){
                                    shapeCalibrated = distanceFromCenterX + distanceFromCenterY - 0.001;
                                }
                                shapeCalibrated = shapeCalibrated - 0.001;
                            }
                        }
                        shapeCalibrated = distanceFromCenterX + distanceFromCenterY;
                        for(int j=0; j<4; j++){
                            wheelTickPrevious[j] = wheelTick[j];           
                        }
                    }
                }
            }
            bestShape = shapeParameters[0];
            int bestIndex = 0;
            for(int i=0; i<199; i++){
                //printf("Parameter: %f Cf: %f\n", shapeParameters[i], cfshape[i]);
                if(cfshape[i+1]<cfshape[bestIndex]){
                    bestShape = shapeParameters[i+1];
                    bestIndex = i+1;
                }
            }
            printf("Calibrated shape: %f\n", bestShape);
            shapeCalibrated = bestShape;
            bag.close();
        }
        /*
        void calculatingBestDistanceFromCenterY(){
            rosbag::Bag bag;
            ros::Time t_previous;
            const std::string bagName1 = "bag3.bag";
            bag.open(bagName1, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back(std::string("wheel_states"));
            topics.push_back(std::string("robot/pose"));

            rosbag::View view(bag, rosbag::TopicQuery(topics));

            double bestw;
            double cfw[200];
            double wParameters[200];

            xMean = 0.0;
            yMean = 0.0;
            thetaMean = 0.0;

            for(int i=0; i<4; i++){
                wheelRpmFromTick[i] = 0.0;
                wheelTick[i] = 0.0;
                wheelTickPrevious[i] = 0.0;
            }
            for(int i=0; i<200; i++){
                xPrevious[i] = 0.0;
                yPrevious[i] = 0.0;
                thetaPrevious[i] = 0.0;
                xNext[i] = 0.0;
                yNext[i] = 0.0;
                thetaNext[i] = 0.0;
                cfw[i] = 0.0;
                
            }

            for(rosbag::MessageInstance const m: rosbag::View(bag)){
                
                if (m.getTopic() == "/robot/pose"|| ("/" + m.getTopic() == "/robot/pose")){
                    geometry_msgs::PoseStamped::ConstPtr msg_pose = m.instantiate<geometry_msgs::PoseStamped>();
                    if(msg_pose != NULL && wheelTick[0] != 0.0){
                       calculatingCostFunction(msg_pose, cfw, 200);
                    }
                }
                    
                if (m.getTopic() == "/wheel_states" || ("/" + m.getTopic() == "/wheel_states")){
                    sensor_msgs::JointState::ConstPtr msg_wheel = m.instantiate<sensor_msgs::JointState>();
                    if(msg_wheel != NULL){
                        delta_t = (msg_wheel->header.stamp.operator-(t_previous)).toSec();
                        for(int i=0; i<4; i++){
                            wheelTick[i] = msg_wheel->position[i];
                            if(wheelTickPrevious[0] != 0){
                                wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*3.14*gearRatio/(encoderResolution*delta_t);
                            }
                        }
                        for(int i=0; i<200; i++){

                            wParameters[i] = wCalibrated;

                            vx = radiusCalibrated*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
                            vy = radiusCalibrated*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
                            wz = radiusCalibrated*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(lCalibrated+wCalibrated));

                            xNext[i] = xPrevious[i] + (vx*cos(thetaPrevious[i] + wz*delta_t*0.5)-vy*sin(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            yNext[i] = yPrevious[i] + (vx*sin(thetaPrevious[i] + wz*delta_t*0.5)+vy*cos(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            thetaNext[i] = thetaPrevious[i] + wz*delta_t;

                            t_previous = msg_wheel->header.stamp;
                            xPrevious[i] = xNext[i];
                            yPrevious[i] = yNext[i];
                            thetaPrevious[i] = thetaNext[i];

                            if(i<100){
                                wCalibrated = wCalibrated + 0.001;
                            }else{
                                if(i == 100){
                                    wCalibrated = distanceFromCenterY - 0.001;
                                }
                                wCalibrated = wCalibrated - 0.001;
                            }
                        }
                        wCalibrated = distanceFromCenterY;
                        for(int j=0; j<4; j++){
                            wheelTickPrevious[j] = wheelTick[j];           
                        }
                    }
                }
            }
            bestw = wParameters[0];
            int bestIndex = 0;
            for(int i=0; i<199; i++){
                //printf("Parameter: %f Cf: %f\n", wParameters[i], cfw[i]);
                if(cfw[i+1]<cfw[bestIndex]){
                    bestw = wParameters[i+1];
                    bestIndex = i+1;
                }
            }
            printf("Calibrated w: %f\n", bestw);
            wCalibrated = bestw;
            bag.close();
        }
        */

        void calculatingBestEncoderResolution(){
            rosbag::Bag bag;
            ros::Time t_previous;
            const std::string bagName1 = "bag3.bag";
            bag.open(bagName1, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back(std::string("wheel_states"));
            topics.push_back(std::string("robot/pose"));

            rosbag::View view(bag, rosbag::TopicQuery(topics));

            double bestn;
            double cfn[20];
            double nParameters[20];

            xMean = 0.0;
            yMean = 0.0;
            thetaMean = 0.0;

            for(int i=0; i<4; i++){
                wheelRpmFromTick[i] = 0.0;
                wheelTick[i] = 0.0;
                wheelTickPrevious[i] = 0.0;
            }
            for(int i=0; i<20; i++){
                xPrevious[i] = 0.0;
                yPrevious[i] = 0.0;
                thetaPrevious[i] = 0.0;
                xNext[i] = 0.0;
                yNext[i] = 0.0;
                thetaNext[i] = 0.0;
                cfn[i] = 0.0;
                
            }

            for(rosbag::MessageInstance const m: rosbag::View(bag)){
                
                if (m.getTopic() == "/robot/pose"|| ("/" + m.getTopic() == "/robot/pose")){
                    geometry_msgs::PoseStamped::ConstPtr msg_pose = m.instantiate<geometry_msgs::PoseStamped>();
                    if(msg_pose != NULL && wheelTick[0] != 0.0){
                       calculatingCostFunction(msg_pose, cfn, 20);
                    }
                }
                    
                if (m.getTopic() == "/wheel_states" || ("/" + m.getTopic() == "/wheel_states")){
                    sensor_msgs::JointState::ConstPtr msg_wheel = m.instantiate<sensor_msgs::JointState>();
                    if(msg_wheel != NULL){
                        delta_t = (msg_wheel->header.stamp.operator-(t_previous)).toSec();
                        for(int i=0; i<20; i++){
                            nParameters[i] = nCalibrated;
                            for(int i=0; i<4; i++){
                                wheelTick[i] = msg_wheel->position[i];
                                if(wheelTickPrevious[0] != 0){
                                    wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*3.14*gearRatio/(nCalibrated*delta_t);
                                }
                            }
                            vx = radiusCalibrated*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
                            vy = radiusCalibrated*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
                            wz = radiusCalibrated*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(shapeCalibrated));

                            xNext[i] = xPrevious[i] + (vx*cos(thetaPrevious[i] + wz*delta_t*0.5)-vy*sin(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            yNext[i] = yPrevious[i] + (vx*sin(thetaPrevious[i] + wz*delta_t*0.5)+vy*cos(thetaPrevious[i] + wz*delta_t*0.5))*delta_t;
                            thetaNext[i] = thetaPrevious[i] + wz*delta_t;

                            t_previous = msg_wheel->header.stamp;
                            xPrevious[i] = xNext[i];
                            yPrevious[i] = yNext[i];
                            thetaPrevious[i] = thetaNext[i];

                            if(i<10){
                                nCalibrated = nCalibrated + 1;
                            }else{
                                if(i == 10){
                                    nCalibrated = encoderResolution - 1;
                                }
                                nCalibrated = nCalibrated - 1;
                            }
                        }
                        nCalibrated = encoderResolution;
                        for(int j=0; j<4; j++){
                            wheelTickPrevious[j] = wheelTick[j];           
                        }
                    }
                }
            }
            bestn = nParameters[0];
            int bestIndex = 0;
            for(int i=0; i<19; i++){
                //printf("Parameter: %f Cf: %f\n", nParameters[i], cfn[i]);
                if(cfn[i+1]<cfn[bestIndex]){
                    bestn = nParameters[i+1];
                    bestIndex = i+1;
                }
            }
            printf("Calibrated N: %f\n", bestn);
            nCalibrated = bestn;
            bag.close();
        }

        void calculatingBestParameters(){
            rosbag::Bag bag;
            ros::Time t_previous;
            const std::string bagName1 = "bag3.bag";
            bag.open(bagName1, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back(std::string("wheel_states"));
            topics.push_back(std::string("robot/pose"));

            rosbag::View view(bag, rosbag::TopicQuery(topics));
            int r = 30, s=30, n=30;
            double xNext = 0.0, yNext = 0.0, thetaNext = 0.0, cfBest = 0.0, rBest = 0.0, shapeBest = 0.0, nBest = 0.0;
            double xPrevious[r][s][n], yPrevious[r][s][n], thetaPrevious[r][s][n], cfValuated[r][s][n];
            double rParameters[r], shapeParameters[s], nParameters[n];

            radiusCalibrated = 0.007;
            shapeCalibrated = 0.369;
            nCalibrated = 42;

            for(int z=0; z<n; z++){
                nParameters[z] = 0.0;
            }

            for(int i=0; i<r; i++){
                
                shapeParameters[i] = 0.0;
                rParameters[i] = 0.0;

                for(int j=0; j<s; j++){
                    for(int k=0; k<n; k++){
                        xPrevious[i][j][k] = 0.0;
                        yPrevious[i][j][k] = 0.0;
                        thetaPrevious[i][j][k] = 0.0;
                        cfValuated[i][j][k] = 0.0;
                    }
                }
            }

            for(rosbag::MessageInstance const m: rosbag::View(bag)){
                
                if (m.getTopic() == "/robot/pose"|| ("/" + m.getTopic() == "/robot/pose")){
                    geometry_msgs::PoseStamped::ConstPtr msg_pose = m.instantiate<geometry_msgs::PoseStamped>();
                    if(msg_pose != NULL && wheelTick[0] != 0.0 && wheelTickPrevious[0] != 0.0){
                        for(int i=0; i<r; i++){
                            
                            rParameters[i] = radiusCalibrated;
                            
                            for(int j=0; j<s; j++){
                                shapeParameters[j] = shapeCalibrated;
                                if(shapeCalibrated == 0.0){
                                    printf("shape calibrated %f\n", shapeCalibrated);
                                }
                                for(int k=0; k<n; k++){
                                    nParameters[k] = nCalibrated;
                                    for(int t=0; t<4; t++){
                                        wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*3.14*gearRatio/(nCalibrated*delta_t);
                                        
                                    }
                                    if(nCalibrated == 0.0){
                                    printf("n calibrated %f\n", nCalibrated);
                                    }
                                    vx = radiusCalibrated*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
                                    vy = radiusCalibrated*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
                                    wz = radiusCalibrated*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(shapeCalibrated));

                                    xNext = xPrevious[i][j][k] + (vx*cos(thetaPrevious[i][j][k] + wz*delta_t*0.5)-vy*sin(thetaPrevious[i][j][k] + wz*delta_t*0.5))*delta_t;
                                    yNext = yPrevious[i][j][k] + (vx*sin(thetaPrevious[i][j][k] + wz*delta_t*0.5)+vy*cos(thetaPrevious[i][j][k] + wz*delta_t*0.5))*delta_t;
                                    thetaNext = thetaPrevious[i][j][k] + wz*delta_t;

                                    xPrevious[i][j][k] = xNext;
                                    yPrevious[i][j][k] = yNext;
                                    thetaPrevious[i][j][k] = thetaNext;

                                    tf2::Quaternion q(msg_pose->pose.orientation.x, msg_pose->pose.orientation.y, msg_pose->pose.orientation.z, msg_pose->pose.orientation.w);
                                    tf2::Matrix3x3 matrix(q);

                                    double roll, pitch, yaw, xMessage, yMessage, thetaMessage;

                                    matrix.getRPY(roll, pitch, yaw);

                                    if(yaw<0){
                                        yaw = -yaw;
                                    }
                                    if(msg_pose->pose.position.x<0){
                                        xMessage = - msg_pose->pose.position.x;
                                    }else{
                                        xMessage = msg_pose->pose.position.x;
                                    }
                                    if(msg_pose->pose.position.y<0){
                                        yMessage = - msg_pose->pose.position.y;
                                    }else{
                                        yMessage = msg_pose->pose.position.y;
                                    }
                            
                                    if(xNext<0){
                                        xMean = -xNext;
                                    }else{
                                        xMean = xNext;
                                    }
                                    if(yNext<0){
                                        yMean = -yNext;
                                    }else{
                                        yMean = yNext;
                                    }
                                    if(thetaNext<0){
                                        thetaMean = -thetaNext;
                                    }else{
                                        thetaMean = thetaNext;
                                    }

                                    cfValuated[i][j][k] = cfValuated[i][j][k] + sqrt((xMessage-xMean)*(xMessage-xMean) + (yMessage-yMean)*(yMessage-yMean) + (yaw-thetaMean)*(yaw-thetaMean));

                                    if(k<n/2){
                                        nCalibrated = nCalibrated + 1;
                                    }else{
                                        if(k == n/2){
                                            nCalibrated = 42;
                                        }
                                        nCalibrated = nCalibrated - 1;
                                    }
                                }
                                nCalibrated = 42;
                                if(j<s/2){
                                    shapeCalibrated = shapeCalibrated + 0.001;
                                }else{
                                    if(j == s/2){
                                        shapeCalibrated = 0.369;
                                    }
                                    shapeCalibrated = shapeCalibrated - 0.001;
                                }
                            }
                            shapeCalibrated = 0.369;
                            if(i<r/2){
                                radiusCalibrated = radiusCalibrated + 0.0001;
                            }else{
                                if(i == r/2){
                                    radiusCalibrated = 0.07;
                                }
                                radiusCalibrated = radiusCalibrated - 0.0001;
                            }
                        }
                        radiusCalibrated = 0.07;
                    }
                }
                    
                if (m.getTopic() == "/wheel_states" || ("/" + m.getTopic() == "/wheel_states")){
                    sensor_msgs::JointState::ConstPtr msg_wheel = m.instantiate<sensor_msgs::JointState>();
                    delta_t = (msg_wheel->header.stamp.operator-(t_previous)).toSec();
                    if(msg_wheel != NULL){
                        if(wheelTick[0] != 0){
                            for(int j=0; j<4; j++){
                                wheelTickPrevious[j] = wheelTick[j];           
                            }
                        }
                        for(int i=0; i<4; i++){
                            wheelTick[i] = msg_wheel->position[i];
                        }
                    }
                    t_previous = msg_wheel->header.stamp;
                }
            }
            cfBest = cfValuated[0][0][0];
            for(int k=0; k<n; k++){
                for(int i=0; i<r; i++){
                    for(int j=0; j<s; j++){
                        //printf("Cf : %f r : %f shape : %f n : %f\n", cfValuated[i][j][k], rParameters[i], shapeParameters[j], nParameters[k]);
                        if(cfValuated[i][j][k]<cfBest){
                            cfBest = cfValuated[i][j][k];
                            rBest = rParameters[i];
                            shapeBest = shapeParameters[j];
                            nBest = nParameters[k];
                        }
                    }
                }
            }
            printf("Calibrated r: %f\n", rBest);
            printf("Calibrated shape: %f\n", shapeBest);
            printf("Calibrated N: %f\n", nBest);
        }

        void calculatingCostFunction(geometry_msgs::PoseStamped::ConstPtr& msg, double cf[], int n){
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 matrix(q);

            double roll, pitch, yaw, xMessage, yMessage, thetaMessage;

            matrix.getRPY(roll, pitch, yaw);
            
            if(yaw<0){
                yaw = -yaw;
            }
            if(msg->pose.position.x<0){
                xMessage = - msg->pose.position.x;
            }else{
                xMessage = msg->pose.position.x;
            }
            if(msg->pose.position.y<0){
                yMessage = - msg->pose.position.y;
            }else{
                yMessage = msg->pose.position.y;
            }
            
            //xMessage = msg->pose.position.x;
            //yMessage = msg->pose.position.y;
            for(int i=0; i<n; i++){
                
                if(xNext[i]<0){
                    xMean = -xPrevious[i];
                }else{
                    xMean = xPrevious[i];
                }
                if(yNext[i]<0){
                    yMean = -yPrevious[i];
                }else{
                    yMean = yPrevious[i];
                }
                if(thetaNext[i]<0){
                    thetaMean = -thetaPrevious[i];
                }else{
                    thetaMean = thetaPrevious[i];
                }
                //xMessage = msg->pose.position.x;
                //yMessage = msg->pose.position.y;
                //xMean = xNext[i];
                //yMean = yNext[i];
                //thetaMean = thetaNext[i];
                cf[i] = cf[i] + sqrt((xMessage-xMean)*(xMessage-xMean) + (yMessage-yMean)*(yMessage-yMean) + (yaw-thetaMean)*(yaw-thetaMean));
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "calibration");
    parametersCalibration parametersCalibration;

    //parametersCalibration.calculatingBestDistancesFromCenter();
    //parametersCalibration.calculatingBestRadius();
    //parametersCalibration.calculatingBestEncoderResolution();
    //parametersCalibration.calculatingBestDistanceFromCenterY();
    parametersCalibration.calculatingBestParameters();
        
    return 0;
}