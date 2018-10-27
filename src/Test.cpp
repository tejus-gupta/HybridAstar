#include "../src/Planner.cpp"
#include "../include/GUI.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include<bits/stdc++.h>

using namespace cv;

int main(){ 
	Mat obs_img = imread("../maps/map.jpg", 0);
    int h = obs_img.rows, w = obs_img.cols;

    cout<<"h and w"<<h<<" "<<w<<endl;

   // Mat canny;
    vector<vector<Point> > obs;  
    vector <Point> a;
    vector <Point> b;
    vector <Point> c;
    //vector<Vec4i> hierarchy;
    // Canny( obs_img, canny, 100, 200, 3 );
    // findContours( canny,obs, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    // Mat drawing = Mat::zeros( canny.size(), CV_8UC3 );
    // for( int i = 0; i< obs.size(); i++ )
    // {
    //    Scalar color = Scalar( rand()%256, rand()%256, rand()%256);
    //    drawContours( drawing, obs, i, color, 2, 8, hierarchy, 0, Point() );
    // }

    // imshow( "Contours", drawing );
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
    obs.push_back(c );

    bool** obs_map = new bool*[h];
    for(int i=0; i<h; i++)
    {
        obs_map[i] = new bool[w]; 
        for(int j=0; j<w; j++)
            obs_map[i][j] = !(obs_img.at<uchar>(i,j) >= 120);  
    }

	State start(10, 9, 0);
	State target(99,90, 0);
    Vehicle car;
	Planner astar;
	
    clock_t start_time=clock();
    astar.h_obj.Dubins_write("Dubins.txt");
    clock_t end_time=clock();
	cout<<"Time: Calculated Dubins Cost= "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;

	start_time=clock();
    vector<State> path = astar.plan(start, target, obs_map, car ,obs);
	end_time=clock();
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