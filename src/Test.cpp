#include "../src/Planner.cpp"
#include "../include/GUI.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <bits/stdc++.h>

using namespace cv;

int main()
{ 
	Mat obs_img = imread("../maps/map.jpg", 0);

    bool** obs_map = new bool*[obs_img.rows];
    for(int i=0; i<obs_img.rows; i++)
    {
        obs_map[i] = new bool[obs_img.cols]; 
        for(int j=0; j<obs_img.cols; j++)
            obs_map[i][j] = !(obs_img.at<uchar>(i,j) >= 120);  
    }

     /* SAT Points */
    vector<vector<Point> > obs; 
    Mat A(obs_img.rows,obs_img.cols,CV_8UC1,Scalar(255));
    for(int i=0;i<obs_img.rows;i++)
    {
        for(int j=0;j<obs_img.cols;j++)
        {
            if(obs_map[i][j])
                A.at<uchar>(i,j) = 0;
        }
    }
    
    // Edge detection
    int thresh=100;
    Canny(A, A, thresh, thresh*2, 3);

    // Finding Contours
    findContours(A, obs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    vector< vector<Point> > fin_obs(obs.size());
    
    // Convex Hulling
    for(int i=0;i<obs.size();i++)    
        convexHull(obs[i],fin_obs[i]);

    // Mat B(1000,1000,CV_8UC1,Scalar(0));
    // for(int i=0;i<fin_obs[0].size();i++)
    //     B.at<uchar>(fin_obs[0][i].y,fin_obs[0][i].x)=255;
    Vehicle car;
    Planner astar;

    float scale = 1000.0/obs_img.rows;
	State start(12, 13, 0);
	State target(64,104, 0);
    
	clock_t start_time=clock();
    vector<State> path = astar.plan(start, target, obs_map, car, fin_obs, scale);
	clock_t end_time=clock();
	
	cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
	cout<<"Got path of length "<<path.size()<<endl;
    
    GUI display(1000, 1000);
    display.draw_obstacles(obs_map, scale);
    for(int i=0;i<=path.size();i++)
    {
        display.draw_car(path[i], car, scale);
        display.show(1);
    } 

    display.show();
}
