#include "../include/Planner.hpp"
#include "../include/GUI.hpp"

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/TransformStamped.h" 

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include <tf/transform_datatypes.h>
#include "tf2/convert.h" 
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

typedef struct _Quaternion
{
    float x;
    float y;
    float z;
    float w;
}Quaternion;

class ROSInterface
{
    public:

	State start;
    State destination;
    int** map;
    float map_origin_x;
    float map_origin_y;
    int map_grid_x;
    int map_grid_y;
    float map_grid_resolution;

    bool got_start;
    bool got_destination;
    bool got_map;

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;

    ros::Subscriber start_sub;
    ros::Subscriber destination_sub;
    ros::Subscriber map_sub;
    ros::Publisher path_pub;

    void startCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) 
    {
        if(got_start)
            return;
        
        cout<<"Getting start position."<<endl;

        start.x = odom_msg->pose.pose.position.x;
        start.y = odom_msg->pose.pose.position.y;

        tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        //due to orientation of base_link_frame wrt vehicle frame
        start.theta = fmod(yaw-M_PI/2+2*M_PI,2*M_PI);

        got_start = true;
        return;
    }

    void destinationCallback(const geometry_msgs::PoseStamped& goal_msg)
    {
        if(got_destination)
            return;

        cout<<"Getting destination"<<endl;
        
        geometry_msgs::PoseStamped transformed_goal;
        geometry_msgs::TransformStamped transform_msg;
        
        try{
            transform_msg = tfBuffer.lookupTransform("map", goal_msg.header.frame_id, ros::Time(0));
            tf2::doTransform(goal_msg, transformed_goal, transform_msg);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }
        
        destination.x= transformed_goal.pose.position.x;
        destination.y= transformed_goal.pose.position.y;
            
        tf::Quaternion q(transformed_goal.pose.orientation.x, transformed_goal.pose.orientation.y, transformed_goal.pose.orientation.z, transformed_goal.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        destination.theta=fmod(yaw+2*M_PI,2*M_PI);

        got_destination = true;
        return;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        if(got_map)
            return;

        cout<<"Getting map"<<endl;

        map_origin_x = msg->info.origin.position.x;
        map_origin_y = msg->info.origin.position.y;
        map_grid_x = msg->info.width;
        map_grid_y = msg->info.height;
        map_grid_resolution = msg->info.resolution;

        //costmap is stored in row-major format
        for(int j=0; j<map_grid_y; j++)
            for(int i=0; i<map_grid_x; i++)
                map[i][j] = msg->data[j*200 + i] != 0 ? 1 : 0;

        got_map = true;
        return;
    }

    void reset()
    {
        got_start = false;
        got_destination = false;
        got_map = false;

        return;
    }

    void transform_start_and_destination()
    {
        start.x = start.x - map_origin_x;
        start.y = start.y - map_origin_y;

        destination.x = destination.x - map_origin_x;
        destination.y = destination.y - map_origin_y;
    }

    void transform_back_destination()
    {
        destination.x = destination.x + map_origin_x;
        destination.y = destination.y + map_origin_y;
    }

    Quaternion toQuaternion(double pitch, double roll, double yaw)
    {
        Quaternion q;

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);

        q.w = cy * cr * cp + sy * sr * sp;
        q.x = cy * sr * cp - sy * cr * sp;
        q.y = cy * cr * sp + sy * sr * cp;
        q.z = sy * cr * cp - cy * sr * sp;
        return q;
    }

    nav_msgs::Path convert_to_path_msg(vector<State> path)
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "odom";

        geometry_msgs::PoseStamped pose, transformed_pose;
        geometry_msgs::TransformStamped transform_msg;

        try{
            transform_msg = tfBuffer.lookupTransform("odom", "map", ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }

        for(vector<State>::iterator ptr = path.begin(); ptr != path.end(); ptr++)
        {

            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();

            pose.pose.position.x = (*ptr).x + map_origin_x;
            pose.pose.position.y = (*ptr).y + map_origin_y;
            pose.pose.position.z = 0;

            Quaternion myQuaternion = toQuaternion(0,0,(*ptr).theta);

            pose.pose.orientation.x = myQuaternion.x;
            pose.pose.orientation.y = myQuaternion.y;
            pose.pose.orientation.z = myQuaternion.z;
            pose.pose.orientation.w = myQuaternion.w;

            tf2::doTransform(pose, transformed_pose, transform_msg);
            path_msg.poses.push_back(transformed_pose);
        }

        return path_msg;
    }

    void publish_path(nav_msgs::Path path_msg)
    {
        path_msg.header.stamp = ros::Time::now();
        path_pub.publish(path_msg);
        return;
    }

    ROSInterface(ros::NodeHandle nh) : listener(tfBuffer)
    {
        this->nh = nh;

        int* map_array=new int[200*200];
        this->map = new int*[200];
        for(int i=0;i<200;i++)
            (this->map)[i] = map_array + 200*i;

        start_sub = nh.subscribe("/base_pose_ground_truth", 1, &ROSInterface::startCallback, this);
        destination_sub = nh.subscribe("/move_base_simple/goal", 1, &ROSInterface::destinationCallback, this);
        map_sub = nh.subscribe("/costmap_node/costmap/costmap", 1, &ROSInterface::mapCallback, this);
        path_pub = nh.advertise<nav_msgs::Path>("/astroid_path", 10);

        got_start = true;
        got_destination = true;
        got_map = true;
        
        return;
    }

    ~ROSInterface()
    {
        delete[] (this->map)[0];
        delete[] this->map;
        
        return;
    }
};


void plan_once(ros::NodeHandle nh)
{ 
    ROSInterface interface(nh);
    interface.got_start = false;
    interface.got_destination = false;
    interface.got_map = false;

    ros::Rate wait_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();

        if(interface.got_start && interface.got_destination && interface.got_map)
            break;

        wait_rate.sleep();
    }

    cout<<"Got start, destination and map."<<endl;

    interface.transform_start_and_destination();

    State start = interface.start;
    State destination = interface.destination;
    int** map = interface.map;
    int map_x = interface.map_grid_x * interface.map_grid_resolution;
    int map_y = interface.map_grid_y * interface.map_grid_resolution;
    float map_grid_resolution = interface.map_grid_resolution;
    float planner_grid_resolution = 1;

    GUI display(map_x, map_y, 5); 
    Vehicle car;

    Planner astar(map_x, map_y, map_grid_resolution, planner_grid_resolution);
    vector<State> path = astar.plan(start, destination, car, map, display);

    while(ros::ok())
    {
        ros::spinOnce();

        nav_msgs::Path path_msg = interface.convert_to_path_msg(path);
        interface.publish_path(path_msg);

        wait_rate.sleep();
    }

    astar.path.clear();
    
    return;
}

void plan_repeatedly(ros::NodeHandle nh)
{ 
    ROSInterface interface(nh);
    interface.got_start = true;
    interface.got_destination = false;
    interface.got_map = true;

    ros::Rate wait_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();

        if(interface.got_destination)
            break;

        wait_rate.sleep();
    }

    cout<<"Got destination."<<endl;

    int map_x = 100;
    int map_y = 100;
    float map_grid_resolution = 0.5;
    float planner_grid_resolution = 1;

    Planner astar(map_x, map_y, map_grid_resolution, planner_grid_resolution);
    GUI display(100, 100, 5); 
    Vehicle car;

    while(ros::ok())
    {
        interface.got_start = false;
        interface.got_map = false;

        while(ros::ok())
        {
            ros::spinOnce();

            if(interface.got_start && interface.got_map)
                break;

            wait_rate.sleep();
        }

        interface.transform_start_and_destination();

        State start = interface.start;
        State destination = interface.destination;
        int** map = interface.map;

        vector<State> path = astar.plan(start, destination, car, map, display);

        nav_msgs::Path path_msg = interface.convert_to_path_msg(path);
        interface.publish_path(path_msg);

        astar.path.clear();
        interface.transform_back_destination();
    }
    
    return;
}


int main(int argc,char **argv)
{ 
    ros::init(argc,argv,"hybrid_astar");
    ros::NodeHandle nh;

    plan_repeatedly(nh);
    
    return 0;
}

