#include <fstream>
#include "ros/ros.h"
#include <hybrid_astar/polygonArray.h>

using namespace std;

int main(int argc,char **argv)
{ 
    ros::init(argc,argv,"obstacle_publisher");
    ros::NodeHandle nh;
    ros::Publisher  pub = nh.advertise<hybrid_astar::polygonArray>("/obstacles", 10);

    ifstream obsFile("/home/tejus/catkin_ws/src/HybridAstar/polygonPoints.txt");
    
    int totalObstacles, obsSize;
    hybrid_astar::polygonArray obs_msg;

    obsFile >> totalObstacles;
    cout<<"totalObstacles: "<<totalObstacles<<endl;
    obs_msg.obstacles.resize(totalObstacles);
    for (int i = 0; i < obs_msg.obstacles.size(); ++i)
    {
    	obsFile >> obsSize;
        cout<<"obsSize: "<<obsSize<<endl;
    	obs_msg.obstacles[i].polygon.resize(obsSize);
        for(int j = 0; j < obs_msg.obstacles[i].polygon.size(); j++)
        {
            geometry_msgs::PointStamped temp;
	    	obsFile >> temp.point.x >> temp.point.y; 
	    	cout<<temp.point.x <<" "<<temp.point.y<<endl;
	    	obs_msg.obstacles[i].polygon[j] = temp;
            
        }
    }
    obsFile.close();

    ros::Rate rate(1);
    while(ros::ok())
    {
    	pub.publish(obs_msg);
    	rate.sleep();
 	}       

    return 0;
}
  