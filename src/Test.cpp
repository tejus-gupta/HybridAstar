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
 
typedef struct _Quaternion
{
	float x;
	float y;
	float z;
	float w;
}Quaternion;


using namespace cv;

State start,dest;
nav_msgs::OccupancyGrid obs_grid;
vector< vector<Point> > obs;
vector< vector< bool > > obs_map;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
   	obs_grid=*msg;
   	obs_map.resize(obs_grid.info.width);
	for(int i=0; i<obs_grid.info.width; i++)
	    obs_map[i].resize(obs_grid.info.height);
    
    for(int i=0; i<obs_grid.info.width; i++) 
	    for(int j=0; j < obs_grid.info.height; j++)
	        obs_map[obs_grid.info.width -1 -i][j] = (obs_grid.data[i*obs_grid.info.width+j]>= 90 || obs_grid.data[i*obs_grid.info.width+j]==-1); 

    // cout<<"Map "<<obs_grid.info.width<<" "<<obs_grid.info.height<<endl;
    // CANNNNY
    Mat A(obs_grid.info.height, obs_grid.info.width, CV_8UC1, Scalar(0));
    for(int i=0;i<A.rows;i++)
        for(int j=0;j<A.cols;j++)
            if(obs_map[i][j])
                A.at<uchar>(i,j) = 255;
      
    // cout<<"Before Canny"<<endl;

    int threshold=100;
    Canny(A,A,threshold,3*threshold,3);

    vector< vector< Point > > temp_obs;
    findContours(A, temp_obs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    obs.resize(temp_obs.size());
    
    for(int i=0;i<temp_obs.size();i++)
        convexHull(temp_obs[i],obs[i]);
    
    // cout<<"THE Found Hull"<<endl;
    // Mat imgp(obs_grid.info.height*5,obs_grid.info.width*5, CV_8UC1,Scalar(0));
    // for(int i=0;i<obs.size();i++)
    // {
    //     for(int j=0;j<obs[i].size();j++)
    //         imgp.at<uchar>(obs[i][j].y*5,obs[i][j].x*5)=255;
    // }
    // imshow("a",imgp);
    // waitKey(0);
    // exit(0);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) 
{
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
    dest.x=goal->pose.pose.position.x;
    dest.y=goal->pose.pose.position.x;
    
    tf::Quaternion q(goal->pose.pose.orientation.x, goal->pose.pose.orientation.y, goal->pose.pose.orientation.z, goal->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    dest.theta=yaw;
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
	 
    ros::init(argc,argv,"hybrid_astar");
    ros::NodeHandle nh;

    //ros::Subscriber sub1  = nh.subscribe("odometry/filtered",10,&odomCallback);
    ros::Subscriber sub2  = nh.subscribe("/map",10,&mapCallback);
    //ros::Subscriber goal  = nh.subscribe("/move_base_simple/goal",10,&goalCallback);

    ros::Publisher  pub = nh.advertise<geometry_msgs::PoseArray>("/waypoint", 10);
    
    // ros::Rate rate(1);
    while(ros::ok())
    {
        Planner astar;
        Vehicle car;
        geometry_msgs::PoseArray poseArray; 
        
        poseArray.header.frame_id = "/map";
         
        State start(5,7,M_PI/2);
        State target(25,154,M_PI/2);  

        ros::spinOnce();
        while(obs_map.empty())
            ros::spinOnce();

	    // cout<<"Started "<<obs_grid.info.width<<" "<<obs_grid.info.height<<endl;
        float scale=1000.0/obs_grid.info.width;

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

            Quaternion myQuaternion=toQuaternion(0,0,(*ptr).theta);

            pose.pose.orientation.x = myQuaternion.x;
            pose.pose.orientation.y = myQuaternion.y;
            pose.pose.orientation.z = myQuaternion.z;
            pose.pose.orientation.w = myQuaternion.w;

            poseArray.poses.push_back(pose.pose);
        }
        cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
        cout<<"Got path of length "<<path.size()<<endl<<endl;

        poseArray.header.stamp = ros::Time::now();
        pub.publish(poseArray);

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
        // exit(0);
        // ros::spinOnce();
        // rate.sleep();
    }

}