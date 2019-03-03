#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>

#include "../src/Planner.cpp"
#include "../include/GUI.hpp"

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/TransformStamped.h" 

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "tf2/convert.h" 
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include <hybrid_astar/polygonArray.h>

using namespace cv;

typedef struct _Quaternion
{
    float x;
    float y;
    float z;
    float w;
}Quaternion;

bool DEBUG = false;

State start, dest;
tf2_ros::Buffer tfBuffer;

vector< vector<Point> > obs;

bool map_ch = false;
bool dest_ch = false;
bool start_ch = false;

void mapCallback(const hybrid_astar::polygonArray& msg)
{
    cout<<"Inside mapCallback"<<endl;    
    map_ch = true;
    
    // The map is received in form of polygon points from lidar data. These points are in
    // base_link frame and have to be converter to hybrid_astar frame.
    geometry_msgs::TransformStamped trans_msg;
    try{
        trans_msg = tfBuffer.lookupTransform("hybrid_astar", "center_laser_link",ros::Time(0));
    }catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
    }
    
    // Creating vector of vector of Obstacles from hull points of lidar data.
    obs.resize(msg.obstacles.size());
    for (int i = 0; i < msg.obstacles.size(); ++i)
        for(int j = 0; j < msg.obstacles[i].polygon.size(); j++)
        {
            geometry_msgs::PointStamped trans,temp;
	        temp = msg.obstacles[i].polygon[j];
            tf2::doTransform(temp,trans,trans_msg);
            obs[i].push_back(Point {trans.point.x,trans.point.y});
        }

}

// Used to locate the current position of vehicle
void odomCallback(const nav_msgs::Odometry& odom_msg) 
{
    cout<<"Inside OdomCallback"<<endl;
    start_ch = true;

    geometry_msgs::PoseStamped begin;
    begin.pose = odom_msg.pose.pose;

    geometry_msgs::PoseStamped  trans_begin;
    geometry_msgs::TransformStamped trans_msg;
    
    try{
        trans_msg = tfBuffer.lookupTransform("hybrid_astar", "map",ros::Time(0), ros::Duration(10));
        tf2::doTransform(begin,trans_begin,trans_msg);
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    start.x = trans_begin.pose.position.x ;
    start.y = trans_begin.pose.position.y ;

    tf::Quaternion q(trans_begin.pose.orientation.x, trans_begin.pose.orientation.y, trans_begin.pose.orientation.z, trans_begin.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    start.theta = fmod(yaw+2*M_PI,2*M_PI);
    
}

// Used to accept the goal position of vehicle
void goalCallback(const geometry_msgs::PoseStamped&  goal)
{
    cout<<"Inside goalCallback"<<endl;
    dest_ch = true;
    
    geometry_msgs::PoseStamped  trans_goal;
	geometry_msgs::TransformStamped trans_msg;
	
	try{
        trans_msg = tfBuffer.lookupTransform("hybrid_astar", "odom",ros::Time(0));
        tf2::doTransform(goal,trans_goal,trans_msg);
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }
    
    dest.x= trans_goal.pose.position.x ;
    dest.y= trans_goal.pose.position.y ;
        
    tf::Quaternion q(trans_goal.pose.orientation.x, trans_goal.pose.orientation.y, trans_goal.pose.orientation.z, trans_goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    dest.theta=fmod(yaw+2*M_PI,2*M_PI);
}

Quaternion toQuaternion(double M_PItch, double roll, double yaw)
{
    Quaternion q;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(M_PItch * 0.5);
    double sp = sin(M_PItch * 0.5);

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
    return q;
}


int main(int argc,char **argv)
{ 

    int rows = 100, cols = 100;
    float scale = 6;

    ros::init(argc,argv,"hybrid_astar");
    ros::NodeHandle nh;

    ros::Subscriber map  = nh.subscribe("/obstacles",10,&mapCallback);   
    ros::Subscriber odom  = nh.subscribe("/base_pose_ground_truth",10,&odomCallback);
    ros::Subscriber goal  = nh.subscribe("/move_base_simple/goal",10,&goalCallback);

    ros::Publisher  pub = nh.advertise<nav_msgs::Path>("/astroid_path", 10);
    tf2_ros::TransformListener listener(tfBuffer);

    ros::Rate rate(1);
    while(ros::ok())
    {
        Planner astar;
        Vehicle car;
        GUI display(rows,cols,scale);

        nav_msgs::Path path_pub; 
        path_pub.header.frame_id = "/map";

        start_ch = false;
        while( !start_ch )
        {
            cout<<"Waiting for Start "<<endl;
            ros::spinOnce();
        }
        cout<<"Starting Received : "<<start.x<<" "<<start.y<<" "<<start.theta<<endl;

        dest_ch = false;
        while(!dest_ch)
        {
            cout<<"Waiting for Goal "<<endl;
            ros::spinOnce();
        }
        cout<<"Destination Received : "<<dest.x<<" "<<dest.y<<" "<<dest.theta<<endl;

        while( !map_ch )
        {
            cout<<"Waiting for Map "<<endl;
            ros::spinOnce();
        }
        map_ch = false;

        clock_t start_time=clock();
        vector<State> path = astar.plan(start, dest, car, obs, display, rows, cols);
        clock_t end_time=clock();

        geometry_msgs::PoseStamped  trans_pose;
        geometry_msgs::TransformStamped trans_msg;
    
        try{
            trans_msg = tfBuffer.lookupTransform("map", "hybrid_astar",ros::Time(0), ros::Duration(10));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }

        vector<State>::iterator ptr;
        for (ptr = path.begin(); ptr != path.end(); ptr++) 
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = (*ptr).x;
            pose.pose.position.y = (*ptr).y;
            pose.pose.position.z = 0;

            Quaternion myQuaternion = toQuaternion(0,0,(*ptr).theta);

            pose.pose.orientation.x = myQuaternion.x;
            pose.pose.orientation.y = myQuaternion.y;
            pose.pose.orientation.z = myQuaternion.z;
            pose.pose.orientation.w = myQuaternion.w;
           
            pose.header.frame_id = "hybrid_astar";
            pose.header.stamp = ros::Time::now();
            
            tf2::doTransform(pose,trans_pose,trans_msg);
            path_pub.poses.push_back(trans_pose);
        }
        cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
        cout<<"Got path of length "<<path.size()<<endl<<endl;

        
        if(DEBUG)
        {
            GUI dis(rows, cols, scale);
            dis.draw_obstacles(obs);
            dis.draw_car(start,car);
            for(int i=0;i<=path.size();i++)
            {
                dis.draw_car(path[i], car);
                dis.show(1);
            } 
            dis.show(200);
        }
       
        path_pub.header.stamp = ros::Time::now();
        pub.publish(path_pub);
        rate.sleep();

    }

}
