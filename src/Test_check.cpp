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
vector< vector<Point> > obs_copy;
bool map_ch = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ch = true;
    obs_grid=*msg;
    
    obs_map.resize(obs_grid.info.width);
    for(int i=0; i<obs_grid.info.width; i++)
        obs_map[i].resize(obs_grid.info.height);
    
    for(int i=0; i<obs_grid.info.width; i++) 
        for(int j=0; j < obs_grid.info.height; j++)
            obs_map[obs_grid.info.width -1 -i][j] = (obs_grid.data[i*obs_grid.info.width+j]>= 90 || obs_grid.data[i*obs_grid.info.width+j]==-1); 

    obs.clear();
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
/*
	For printing the output of Contour Detection
*/    

 //    vector<Vec4i> hierarchy;
 //    Mat drawing = Mat::zeros( A.size(), CV_8UC3 );
 //	   for( int i = 0; i< temp_obs.size(); i++ )
 //    {
 //       Scalar color = Scalar( 0,255,255);
 //       drawContours( drawing, temp_obs, i, color, 2, 8, hierarchy, 0, Point() );
 //    }

 //    imshow("Contours ",drawing);
 //    waitKey(0);

    for(int i=0;i<temp_obs.size();i++)
        convexHull(temp_obs[i],obs[i]);

/*
	For printing the output of Convex Hull
*/    
    // Mat drawing = Mat::zeros( A.size(), CV_8UC3 );
    // for( int i = 0; i< temp_obs.size(); i++ )
    // {
    //     Scalar color = Scalar( 0, 0, 255 );
    //     Scalar color1 = Scalar( 0, 255, 255 );
    //     drawContours( drawing, temp_obs, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    //     drawContours( drawing, obs, i, color1, 1, 8, vector<Vec4i>(), 0, Point() );
    // }

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

    int rows = 200, cols = 200;
    float scale = 3;
    
    ros::init(argc,argv,"hybrid_astar");
    ros::NodeHandle nh;
    
    ros::Subscriber map  = nh.subscribe("/map",10,&mapCallback);   

    ros::Rate rate(1);

    Planner astar(rows,cols);
    Vehicle car;
    GUI display(rows,cols,scale);
    
    start.x = start.y = 5;
    start.theta = dest.theta = 0;
    dest.x = dest.y = 195;
    
    while( !map_ch )
    {
        ros::spinOnce();
    }
    map_ch = false;
    
    clock_t start_time=clock();
    vector<State> path = astar.plan(start, dest, car, obs, display);
    clock_t end_time=clock();
    cout<<"Time: Overall= "<<double(end_time-start_time)/CLOCKS_PER_SEC<<endl;

    astar.path.clear();
    
    GUI dis(rows, cols, scale);
    dis.draw_obstacles(obs);
    dis.draw_car(start,car);
    for(int i=0;i<=path.size();i++)
    {
        dis.draw_car(path[i], car);
        dis.show(1);
    } 
    dis.show();

}

