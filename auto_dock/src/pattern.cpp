#include <ros/ros.h>
#include <laser_line_extraction/LineSegmentList.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

#include <vector>
#include <cmath>

#define _USE_MATH_DEFINES

// ROS
visualization_msgs::Marker points_msg;
std_msgs::Bool lr_msg;

// Parameters
double pattern_angle1, pattern_angle2, pattern_angle3;
double detect_angle_tolerance, group_dist_tolerance;
std::string laser_frame_id;


int v_a, v_b, v_c, v_d;
int v_a_t, v_b_t, v_c_t, v_d_t;
bool find_once = false;
double p[6];


bool calAngle(double a, double b, double angle_ab, double detect_angle_tolerance){
    double angle;
    if ((a*b) > 0) angle = fabs(a-b);
    else angle = 2* M_PI-fabs(a-b);
    
    if (fabs(angle_ab-angle)<=detect_angle_tolerance) return true;
    else return false;
}


void populateMarkerMsg(double x, double y){
    points_msg.id = 1;
    points_msg.ns = "points";
    points_msg.type = visualization_msgs::Marker::POINTS;
    points_msg.action = visualization_msgs::Marker::ADD;
    points_msg.scale.x = 0.03;
    points_msg.scale.y = 0.03;
    points_msg.color.r = 0.0;
    points_msg.color.g = 0.0;
    points_msg.color.b = 1.0;
    points_msg.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    points_msg.points.push_back(p);
    points_msg.header.frame_id = laser_frame_id;
    points_msg.header.stamp = ros::Time::now();
}

void populateTF(double x, double y, double theta, std::string name){
    // publish dock_frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0));
    tf::Quaternion q;
    q.setRPY(0,0,theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),laser_frame_id,name));
}

void findXY(float xa, float ya, float theta_a, float xb, float yb, float theta_b, int i){
    double ka = ya - tan(theta_a)*xa;
    double kb = yb - tan(theta_b)*xb;
    p[2*i-2] = xa;   //(-ka+kb)/(tan(theta_a)-tan(theta_b));
    p[2*i-1] = ya;   //tan(theta_a)*x+ka;
}

void updateVectors(){
    v_a = v_a_t;
    v_b = v_b_t;
    v_c = v_c_t;
    v_d = v_d_t;
    printf("Upate vectors!\n");
}

void patternCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg){
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> vectors = msg->line_segments;

    // Number of the line
    int lineNum = vectors.size();
    
    bool check_vec_size = true;
    bool check_angle1 = false;
    bool check_angle2 = false;
    bool check_angle3 = false;

    fprintf(stderr, "Number of line = %d\n", lineNum);

    // Check whether topic line_segments is publishing
    if (lineNum < 4){
        ROS_ERROR("There isn't enough line in the laser field!");
        check_vec_size = false;
    }

    // Find Theta2 and label vector b and c
    if (check_vec_size){
        ROS_INFO("Searching Pattern......");
        for(int i=0; i<lineNum; i++){
            for (int j=i+1; j<lineNum; j++){
                
                if (calAngle(vectors[i].angle,vectors[j].angle, 3.14-pattern_angle2, detect_angle_tolerance)){
                
                    // Label vector_b and vector_c
                    double dist_x1 = fabs(vectors[i].start[0] - vectors[j].end[0]);
                    double dist_y1 = fabs(vectors[i].start[1] - vectors[j].end[1]);
                    double dist_1 = sqrt(pow(fabs(vectors[i].start[0] - vectors[j].end[0]),2)+pow(fabs(vectors[i].start[1] - vectors[j].end[1]),2));
                    double dist_x2 = fabs(vectors[j].start[0] - vectors[i].end[0]);
                    double dist_y2 = fabs(vectors[j].start[1] - vectors[i].end[1]);
                    double dist_2 = sqrt(pow(fabs(vectors[j].start[0] - vectors[i].end[0]),2)+pow(fabs(vectors[j].start[1] - vectors[i].end[1]),2));
                    
                    if (dist_1 <= group_dist_tolerance){
                        v_b_t = i;
                        v_c_t = j;
                        check_angle2 = true;
                        printf("find b:%d, find c:%d\n",v_b_t,v_c_t);
                    } 
                    else if (dist_2 <= group_dist_tolerance){
                        v_b_t = j;
                        v_c_t = i;
                        check_angle2 = true;
                        printf("find b:%d, find c:%d\n",v_b_t,v_c_t);
                    }               
                }
            }
        }
    }

    // Find vector a
    if (check_angle2){
        for (int i=0; i<lineNum; i++){
            double dist_x = fabs(vectors[i].start[0] - vectors[v_b_t].end[0]);
            double dist_y = fabs(vectors[i].start[1] - vectors[v_b_t].end[1]);
            double dist_= sqrt(pow(fabs(vectors[i].start[0] - vectors[v_b_t].end[0]),2)+pow(fabs(vectors[i].start[1] - vectors[v_b_t].end[1]),2));
            
            if (dist_ <= group_dist_tolerance){
                if (calAngle(vectors[i].angle,vectors[v_b_t].angle, pattern_angle1-3.14, detect_angle_tolerance)){
                    v_a_t = i;
                    check_angle1 = true;
                    printf("find a = %d\n",v_a_t);
                }
            
            }
        }
    }

    
    if (check_angle1){
        // Find vector d
        for (int i=0; i<lineNum; i++){
            double dist_x = fabs(vectors[v_c_t].start[0] - vectors[i].end[0]);
            double dist_y = fabs(vectors[v_c_t].start[1] - vectors[i].end[1]);
            double dist_= sqrt(pow(fabs(vectors[v_c_t].start[0] - vectors[i].end[0]),2)+pow(fabs(vectors[v_c_t].start[1] - vectors[i].end[1]),2));
            
            if (dist_ <= group_dist_tolerance){
                if (calAngle(vectors[i].angle,vectors[v_c_t].angle, pattern_angle1-3.14, detect_angle_tolerance)){
                    v_d_t = i;
                    find_once = true;
                    check_angle3 = true;
                    printf("find d = %d\n",v_d_t);
                    ROS_INFO("Find the Pattern!");
                    updateVectors();
                    findXY(vectors[v_a].start[0],vectors[v_a].start[1],vectors[v_a].angle,vectors[v_b].start[0],vectors[v_b].start[1],vectors[v_b].angle,1);
                    findXY(vectors[v_b].start[0],vectors[v_b].start[1],vectors[v_b].angle,vectors[v_c].start[0],vectors[v_c].start[1],vectors[v_c].angle,2);
                    findXY(vectors[v_c].start[0],vectors[v_c].start[1],vectors[v_c].angle,vectors[v_d].start[0],vectors[v_d].start[1],vectors[v_d].angle,3); 
                }
            }
        }
    }

    if (check_angle3){
        //Set origin of frame
        #if 0
        double x = (vectors[v_b].end[0] + vectors[v_c].start[0])/2;
        double y = (vectors[v_b].end[1] + vectors[v_c].start[1])/2;
        double theta =  atan2((y-vectors[v_b].start[1]),(x-vectors[v_b].start[0]));
        #endif
        double x = (p[0]+p[4])/2;
        double y = (p[1]+p[5])/2;
        double theta = atan2((y-p[3]),(x-p[2]));
        
        // populate dock origin marker
        populateMarkerMsg(x,y);
        // populate dock_frame
        populateTF(x,y,theta,"dock_frame");
        // populate target_frame
        //populateTF((x+min_alignment_position*cos(theta)),(y+min_alignment_position*sin(theta)),theta,"target_frame");

        // whether robot at the left_hand side or not
        double dist_left = pow(p[0],2)+pow(p[1],2);
        double dist_right = pow(p[4],2)+pow(p[5],2);
        if (dist_left < dist_right){
            lr_msg.data = true;
        }
        else {
            lr_msg.data = false;
        }
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "pattern_node");
    ros::NodeHandle nh_;

    // Load Parameters
    nh_.param<double>("pattern_angle1",pattern_angle1, 3.81);
    nh_.param<double>("pattern_angle2",pattern_angle2, 1.8);
    nh_.param<double>("pattern_angle3",pattern_angle3, 3.81);
    nh_.param<double>("detect_angle_tolerance",detect_angle_tolerance, 0.1);
    nh_.param<double>("group_dist_tolerance",group_dist_tolerance, 0.1);
    nh_.param<std::string>("laser_frame_id",laser_frame_id, "laser_frame");
    #if 0
    nh_.setParam("pattern_angle1",pattern_angle1);
    nh_.setParam("pattern_angle2",pattern_angle2);
    nh_.setParam("pattern_angle3",pattern_angle3);
    nh_.setParam("detect_angle_tolerance",detect_angle_tolerance);
    nh_.setParam("group_dist_tolerance",group_dist_tolerance);
    nh_.setParam("pattern_angle1",pattern_angle1);
    nh_.setParam("laser_frame_id",laser_frame_id);
    #endif

    ros::Subscriber line_sub_ = nh_.subscribe("line_segments", 10, patternCallback);
    ros::Publisher marker_pub_ =nh_.advertise<visualization_msgs::Marker>("origin_markers",1);
    ros::Publisher lr_pub_ =nh_.advertise<std_msgs::Bool>("l_or_r",100);

    ros::Rate rate(20.0);

    while(ros::ok()){
        marker_pub_.publish(points_msg);
        lr_pub_.publish(lr_msg);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
