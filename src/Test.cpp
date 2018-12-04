#include "opencv/cv.h"
#include "../src/Planner.cpp"
#include "../include/GUI.hpp"
#include <vector>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/PoseStamped.h" 
#include <tf2/LinearMath/Quaternion.h>

#include <tf/transform_datatypes.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_converter/costmap_converter_interface.h>

typedef struct _Quaternion{
	float x;
	float y;
	float z;
	float w;
}Quaternion;


using namespace cv;

State start,target;
nav_msgs::OccupancyGrid obs_grid;
vector<vector<Point> > obs;
bool** obs_map;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
   	obs_grid=*msg;
   	obs_map = new bool*[obs_grid.info.width];
	for(int i=0; i<obs_grid.info.width; i++)
	{
	    obs_map[i] = new bool[obs_grid.info.height]; 
	    for(int j=0; j<obs_grid.info.height; j++)
<<<<<<< HEAD:src/Test.cpp
	        obs_map[i][j] = (obs_grid.data[i*obs_grid.info.width+j]>= 120);  
=======
	        obs_map[i][j] = (obs_grid.data[i*obs_grid.info.width+j]>= 90); 
>>>>>>> 4e1ae5cf44076d7051dc3614b4d5dc90f08e2575:src/Test.cpp
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
  cout<<"Inside OdomCallback"<<endl;
  start.x = odom_msg->pose.pose.position.x;
  start.y = odom_msg->pose.pose.position.y;

  tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  start.theta = yaw;

}

void goalCallback(const nav_msgs::Odometry::ConstPtr&  goal)
{
    target.x=goal->pose.pose.position.x;
    target.y=goal->pose.pose.position.x;
    
    tf::Quaternion q(goal->pose.pose.orientation.x, goal->pose.pose.orientation.y, goal->pose.pose.orientation.z, goal->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    target.theta=yaw;
}

void obstacleCallback(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst)
{
    for(int i=0;i<obst->obstacles.size();i++)
    {
        vector< Point > aur;
        for(int j=0;j<obst->obstacles[i].polygon.points.size();j++)
        {
            Point temp(obst->obstacles[i].polygon.points[j].x,obst->obstacles[i].polygon.points[j].y);
            aur.push_back(temp);
        }
        obs.push_back(aur);
    }
}

Quaternion toQuaternion(double pitch, double roll, double yaw)
{
	Quaternion q;
    // Abbreviations for the various angular functions
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


int main(int argc,char **argv)
{ 
<<<<<<< HEAD:src/Test.cpp
	start.x=5;
	start.y=4;
	start.theta=0;
=======
	State start(13,14,M_PI/2);
    State target(13,74,M_PI/2);

>>>>>>> 4e1ae5cf44076d7051dc3614b4d5dc90f08e2575:src/Test.cpp
    ros::init(argc,argv,"hybrid_astar");
    ros::NodeHandle nh;

    // ros::Subscriber sub1  = nh.subscribe("odometry/filtered",10,&odomCallback);
    ros::Subscriber sub2  = nh.subscribe("/map",10,&mapCallback);
<<<<<<< HEAD:src/Test.cpp
    ros::Subscriber goal  = nh.subscribe("/move_base_simple/goal",10,&goalCallback);
    ros::Subscriber obstacles = nh.subscribe("/costmap_converter/costmap_obstacles",10,&obstacleCallback);

    // ros::Publisher  pub = nh.advertise<geometry_msgs::PoseArray>("", 1000);
=======
    // ros::Subscriber goal  = nh.subscribe("/move_base_simple/goal",10,&goalCallback);
    // ros::Subscriber obstacles = nh.subscribe("/costmap_converter/costmap_obstacles",10,&obstacleCallback);

    ros::Publisher  pub = nh.advertise<geometry_msgs::PoseArray>("/waypoint", 1000);
>>>>>>> 4e1ae5cf44076d7051dc3614b4d5dc90f08e2575:src/Test.cpp

    geometry_msgs::PoseArray poseArray; 
    poseArray.header.frame_id = "/map";

<<<<<<< HEAD:src/Test.cpp
    cout<<"Started "<<obs_grid.info.width<<" "<<obs_grid.info.height<<endl;


=======
>>>>>>> 4e1ae5cf44076d7051dc3614b4d5dc90f08e2575:src/Test.cpp
    Vehicle car;
    Quaternion myQuaternion;
    
    ros::Rate rate(2);
    while(ros::ok())
    {
        poseArray.poses.clear(); 
        poseArray.header.stamp = ros::Time::now();

<<<<<<< HEAD:src/Test.cpp
        while(!obs_grid.info.width || obs.empty())
        	ros::spinOnce();

	    cout<<"Started "<<obs_grid.info.width<<" "<<obs_grid.info.height<<endl;

        cout<<obs.size()<<endl;
=======
        while(!obs_grid.info.width)
        	ros::spinOnce();

	    cout<<"Started "<<obs_grid.info.width<<" "<<obs_grid.info.height<<endl;
        float scale=1000.0/obs_grid.info.width;
        
        cout<<"Scale "<<scale<<endl;
        // GUI display(1000, 1000);
        // display.draw_obstacles(obs_map,scale);
        // display.draw_car(start,car,scale);
        // display.draw_car(target,car,scale);
        // display.show();
        Planner astar;
>>>>>>> 4e1ae5cf44076d7051dc3614b4d5dc90f08e2575:src/Test.cpp
        clock_t start_time=clock();
        vector<State> path = astar.plan(start, target, obs_map, car ,obs, scale);
        clock_t end_time=clock();

        vector<State>::iterator ptr;
        for (ptr = path.begin(); ptr != path.end(); ptr++) 
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = (*ptr).x;
            pose.pose.position.y = (*ptr).y;
            pose.pose.position.z = 0;

            myQuaternion=toQuaternion(0,0,(*ptr).theta);

            pose.pose.orientation.x = myQuaternion.x;
            pose.pose.orientation.y = myQuaternion.y;
            pose.pose.orientation.z = myQuaternion.z;
            pose.pose.orientation.w = myQuaternion.w;

            poseArray.poses.push_back(pose.pose);
        }
        cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
        cout<<"Got path of length "<<path.size()<<endl;

<<<<<<< HEAD:src/Test.cpp
        // pub.publish(poseArray);
        // ROS_INFO("poseArray size: %i", poseArray.poses.size()); 
=======
        pub.publish(poseArray);
>>>>>>> 4e1ae5cf44076d7051dc3614b4d5dc90f08e2575:src/Test.cpp

        // ROS_INFO("poseArray size: %i", poseArray.poses.size()); 
        // GUI display(1000, 1000);
        // display.draw_obstacles(obs_map,scale);
        // display.draw_car(start,car,scale);

        // for(int i=0;i<=path.size();i++)
        // {
        //     display.draw_car(path[i], car,scale);
        //     display.show(1);
        // } 
        // display.show();
        
        ros::spinOnce();
        rate.sleep();
    }

}