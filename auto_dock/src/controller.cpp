#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <chrono>

#define _USE_MATH_DEFINES

// ROS
ros::Publisher vel_pub;

//parameter setting
std::string base_frame_id;
double min_v, min_w, max_v, max_w;
double dist_to_dock, dist_to_center;
double threshold_v, threshold_w;

bool reach_target_pos = false;
bool reach_target_ang = false;
bool target_alignment = false;
bool left = false;
bool left_check = false;
bool pattern = false;
int step = 0;


static double clamp(double v, double v_min, double v_max)
{
    return std::min(std::max(v_min,v),v_max);
}

void lrCallback(const std_msgs::Bool::ConstPtr& msg){
    left = msg -> data;
}

// Setting linear and angular velocity
void setVel(float x, float y, float yaw){
    static geometry_msgs::Twist vel_msg;

    if (step == 0){
        vel_msg.linear.x = 0;
        ROS_INFO("docking......");
        if (left){ 
            if (((M_PI_2-0.03)<=yaw) && (yaw<=(M_PI_2+0.03))){
                vel_msg.angular.z = 0;
                vel_pub.publish(vel_msg);
                step = 1;
                left_check = true;
            }
            else if (((M_PI_2-threshold_w)<=yaw) && (yaw<(M_PI_2-0.03))) {
                vel_msg.angular.z = -min_w;
            }
            else if (((-M_PI_2)<=yaw) && (yaw<(M_PI_2-threshold_w))){
                vel_msg.angular.z = -max_w;
            }
            else if (((M_PI_2+0.03)<=yaw) && (yaw<(M_PI_2+threshold_w))){
                vel_msg.angular.z = min_w;
            }
            else {
                vel_msg.angular.z = max_w;
            }
        }
        else {
            if (((-M_PI_2-0.03)<=yaw) && (yaw<=(-M_PI_2+0.03))){
                vel_msg.angular.z = 0;
                vel_pub.publish(vel_msg);
                step = 1;
                left_check = false;
            }
            else if (((-M_PI_2+0.03)<=yaw) && (yaw<(-M_PI_2+threshold_w))){
                vel_msg.angular.z = min_w;
            }
            else if (((-M_PI_2+threshold_w)<=yaw) && (yaw<(M_PI_2))){
                vel_msg.angular.z = max_w;
            }
            else if (((-M_PI_2-threshold_w)<=yaw) && (yaw<(-M_PI_2-0.03))){
                vel_msg.angular.z = -min_w;
            }
            else {
                vel_msg.angular.z = -max_w;
            }
        }
    }
    else if (step == 1){
        ROS_INFO("docking......");
        if (fabs(x) <= dist_to_center){
            vel_msg.linear.x = 0;
            vel_pub.publish(vel_msg);
            step = 2;
        }
        else if ((dist_to_center<(-x)) && ((-x)<(dist_to_center+threshold_v))){
            vel_msg.linear.x = -min_v;
        }
        else if ((dist_to_center<x) && (x<(dist_to_center+threshold_v))){
            vel_msg.linear.x = min_v;
        }
        else {
            vel_msg.linear.x = -max_v;
        }
    }
    else if (step == 2){
        ROS_INFO("docking......");
        if (fabs(yaw)<=0.03){
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
            step = 3;
        }
        else if ((0.03<=yaw)&&(yaw<threshold_w)){
            vel_msg.angular.z = min_w;
        }
        else if((-threshold_w<=yaw)&&(yaw<-0.03)){
            vel_msg.angular.z = -min_w;
        }
        else if (threshold_w<=yaw){
            vel_msg.angular.z = max_w;
        }
        else {
            vel_msg.angular.z = -max_w;
        }
    }
    else if (step == 3){
        ROS_INFO("docking......");
        if (fabs(x)<dist_to_dock){
            vel_msg.linear.x = 0;
            vel_pub.publish(vel_msg);
            step = 4;
        }
        else if (fabs(x)<=(dist_to_dock+threshold_v)){
            vel_msg.linear.x = -min_v;
        }
        else {
            vel_msg.linear.x = -max_v;
        }
    }
    else if (step == 4){
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);
        ROS_INFO("Finish Docking!");
    }
    vel_pub.publish(vel_msg);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n_;

    // Load Parameters
    n_.param<std::string>("base_frame_id",base_frame_id, "base_link");
    n_.param<double>("min_v",min_v, 0.1);
    n_.param<double>("min_w",min_w, 0.1);
    n_.param<double>("max_v",max_v, 0.3);
    n_.param<double>("max_w",max_w, 0.3);
    n_.param<double>("dist_to_dock",dist_to_dock, 0.3);
    n_.param<double>("dist_to_center",dist_to_center, 0.03);
    n_.param<double>("threshold_v",threshold_v, 0.3);
    n_.param<double>("threshold_w",threshold_w, 0.4);
    

    ros::Subscriber lr_sub_ = n_.subscribe("l_or_r", 10, lrCallback);
    vel_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    tf::TransformListener listener_dock;
    ros::Rate rate(20.0);
    while(ros::ok()){
        tf::StampedTransform tf_dock;
        try{
            listener_dock.waitForTransform("base_link","dock_frame",ros::Time(0),ros::Duration(3.0));
            listener_dock.lookupTransform("base_link","dock_frame",ros::Time(0),tf_dock);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Did not find the pattern!");
            ros::Duration(1.0).sleep();
            continue;
        }

        // Dock_frame's origin and yaw
        float dock_x = tf_dock.getOrigin().x();
        float dock_y = tf_dock.getOrigin().y();
        float dock_yaw = tf::getYaw(tf_dock.getRotation());
        ros::spinOnce();
        setVel(dock_x, dock_y, dock_yaw);
        rate.sleep();
    }
    return 0;
}