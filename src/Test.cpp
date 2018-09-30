#include "../src/Planner.cpp"
#include "../include/GUI.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include<bits/stdc++.h>

using namespace cv;

int main(){ 
	Mat obs_img = imread("../maps/map.jpg", 0);
    int h = obs_img.rows, w = obs_img.cols;
 
    bool** obs_map = new bool*[h];
    for(int i=0; i<h; i++)
    {
        obs_map[i] = new bool[w];
        for(int j=0; j<w; j++)
            obs_map[i][j] = !(obs_img.at<uchar>(i,j) >= 120);  
    }

	State start(1, 90, 0);
	State target(1, 20, 3.14);
    Vehicle car;
	Planner astar;
	
	clock_t start_time=clock();
	vector<State> path = astar.plan(start, target, obs_map, car);
	clock_t end_time=clock();
	cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
	cout<<"Got path of length "<<path.size()<<endl;
    
    GUI display(1000, 1000);
    display.draw_obstacles(obs_map);
    for(int i=0;i<=path.size();i++)
    {
        display.draw_car(path[i], car);
        display.show(1);
    }
    display.show();
}