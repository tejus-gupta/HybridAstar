#include "../src/Planner.cpp"
#include "../include/GUI.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <bits/stdc++.h>

using namespace cv;

int main()
{ 
	Mat obs_img = imread("../maps/map.jpg", 0);

    /* SAT Points */
    vector<vector<Point> > obs; 
    
    vector <Point> a;
    vector <Point> b;
    vector <Point> c;
    a.push_back(Point(64,105));
    a.push_back(Point(64,185));
    a.push_back(Point(280,185));
    a.push_back(Point(280,105));
    obs.push_back(a);
    b.push_back(Point(64,225));
    b.push_back(Point(64,310));
    b.push_back(Point(280,310));
    b.push_back(Point(280,225));
    obs.push_back(b);
    c.push_back(Point(64,335));
    c.push_back(Point(64,430));
    c.push_back(Point(280,430));
    c.push_back(Point(280,335));
    obs.push_back(c);
    
    bool** obs_map = new bool*[obs_img.rows];
    for(int i=0; i<obs_img.rows; i++)
    {
        obs_map[i] = new bool[obs_img.cols]; 
        for(int j=0; j<obs_img.cols; j++)
            obs_map[i][j] = !(obs_img.at<uchar>(i,j) >= 120);  
    }

    Vehicle car;
    Planner astar;

    // Calculating Dubin's cost
    clock_t start_time=clock();
    // astar.h_obj.Dubins(car.min_radius);
    clock_t end_time=clock();
    // cout<<"Dubin's time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;

    float scale = 1000.0/obs_img.rows;
	State start(12, 13, 0,scale);
	State target(604,804, 0,scale);
    
	start_time=clock();
    vector<State> path = astar.plan(start, target, obs_map, car ,obs,scale);
	end_time=clock();
	
	cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
	cout<<"Got path of length "<<path.size()<<endl;
    
    GUI display(1000, 1000);
    display.draw_obstacles(obs_map,scale);
    for(int i=0;i<=path.size();i++)
    {
        display.draw_car(path[i], car,scale);
        display.show(1);
    } 

    display.show();
}