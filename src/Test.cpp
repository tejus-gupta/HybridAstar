#include "../include/Planner.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include "../include/GUI.hpp"

using namespace cv;

int main(){ 
	Mat obs_img = imread("../maps/map.jpg", 0);
    int h = obs_img.rows, w = obs_img.cols;

    bool** obs_map = new bool*[h];
    for(int i=0; i<h; i++)
    {
        obs_map[i] = new bool[w];
        for(int j=0; j<w; j++)
            obs_map[i][j] = (obs_img.at<uchar>(j, h-i) >= 120);
    }

	State start(700, 100, 0);
	State target(100, 600, 0);
    Vehicle car;
	Planner astar;
	stack<State> path = astar.plan(start, target, obs_map, car);

    GUI display(1000, 1000);
    display.draw_obstacles(obs_map);

    for(int i=0;i<=path.size();i++)
    {
        display.draw_car(path.top(), car);
        path.pop();
    }
    
    display.show();
}