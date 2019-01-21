#include "opencv/cv.h"
#include "../src/Planner.cpp"
#include "../include/GUI.hpp"
#include <vector>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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
bool dest_receive=false;

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
    obs.clear();

    Mat A(obs_grid.info.height, obs_grid.info.width, CV_8UC1, Scalar(0));
    for(int i=0;i<A.rows;i++)
        for(int j=0;j<A.cols;j++)
            if(obs_map[i][j])
                A.at<uchar>(i,j) = 255;


    // vector <Point> a;
    // vector <Point> b;
    // vector <Point> c;
    // a.push_back(Point(64,105));
    // a.push_back(Point(64,185));
    // a.push_back(Point(280,185));
    // a.push_back(Point(280,105));
    // obs.push_back(a);
    // b.push_back(Point(64,225));
    // b.push_back(Point(64,310));
    // b.push_back(Point(280,310));
    // b.push_back(Point(280,225));
    // obs.push_back(b);
    // c.push_back(Point(64,335));
    // c.push_back(Point(64,430));
    // c.push_back(Point(280,430));
    // c.push_back(Point(280,335));
    // obs.push_back(c );


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
 //    Mat drawing = Mat::zeros( A.size(), CV_8UC3 );
 //    for( int i = 0; i< temp_obs.size(); i++ )
 //    {
 //        Scalar color = Scalar( 0, 0, 255 );
 //        Scalar color1 = Scalar( 0, 255, 255 );
 //        drawContours( drawing, temp_obs, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
 //        drawContours( drawing, obs, i, color1, 1, 8, vector<Vec4i>(), 0, Point() );
 //    }
 //    imshow("Contours ",drawing);
 //    waitKey(0);

/*
	For printing the points of Convex Hull
*/
    obs_copy.resize(obs.size());
    for (int i = 0; i < obs.size(); ++i)
    {
        obs_copy[i].resize(obs[i].size());
    	for (int j = 0; j < obs[i].size(); ++j)
    	{
    		obs_copy[i][j].x=obs[i][j].y;
    		obs_copy[i][j].y=obs[i][j].x;
    	}
    }
    // Mat drawing = Mat::zeros( A.size(), CV_8UC3 );
    // for( int i = 0; i< temp_obs.size(); i++ )
    // {
    //     Scalar color = Scalar( 0, 0, 255 );
    //     Scalar color1 = Scalar( 0, 255, 255 );
    //     // drawContours( drawing, temp_obs, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    //     drawContours( drawing, obs_copy, i, color1, 1, 8, vector<Vec4i>(), 0, Point() );
    // }
    // imshow("Contours ",drawing);
    // waitKey(0);
    // exit(0);

    // cout<<"Found Hull"<<endl;

    
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

void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg) 
{
    cout<<"Inside OdomCallback"<<endl;
    start.x = obs_map.size() - odom_msg->pose.pose.position.y;
    start.y = odom_msg->pose.pose.position.x;

    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    start.theta = yaw;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr&  goal)
{
    dest.x= obs_map.size()- goal->pose.position.y;
    dest.y= goal->pose.position.x;
    
    tf::Quaternion q(goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    dest.theta=yaw;
    dest_change=true;
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

    ros::Subscriber sub1  = nh.subscribe("/initialpose",10,&odomCallback);
    ros::Subscriber sub2  = nh.subscribe("/map",10,&mapCallback);
    ros::Subscriber goal  = nh.subscribe("/move_base_simple/goal",10,&goalCallback);

    ros::Publisher  pub = nh.advertise<geometry_msgs::PoseArray>("/waypoint", 10);
    
    while(ros::ok())
    {
        Planner astar;
        Vehicle car;
        geometry_msgs::PoseArray poseArray; 
        
        poseArray.header.frame_id = "/map";
        
        State start(10,10,CV_PI/2);
        State target(341,460,CV_PI/2); 

        ros::spinOnce();
        while(obs_map.empty())
    	{
    		cout<<"Waiting for Map "<<endl;
            ros::spinOnce();
    	}

        while(!dest_receive)
        {
            cout<<"Waiting for Goal "<<endl;
            ros::spinOnce();
        }

        dest_receive = false;
        cout<<"Destination Received : "<<dest.x<<" "<<dest.y<<" "<<dest.theta<<endl;
        cout<<"Starting Received : "<<start.x<<" "<<start.y<<" "<<start.theta<<endl;
                
        cout<<"Started "<<obs_grid.info.width<<" "<<obs_grid.info.height<<endl;
        float scale=1000.0/obs_grid.info.width;

        clock_t start_time=clock();
        vector<State> path = astar.plan(start, dest, obs_map, car ,obs_copy, scale);
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
    }

}

