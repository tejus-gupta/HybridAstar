#include "../include/Planner.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include "../include/GUI.hpp"

using namespace cv;

int main(){
	Mat obs_img = imread("../maps/map.jpg", 0);
    int h = obs_img.rows, w = obs_img.cols;

    int** obs_map = new int*[h];
    for(int i=0; i<h; i++)
    {
        obs_map[i] = new int[w];
        for(int j=0; j<c; j++)
            obs_map[i][j] = (obs_img.at<uchar>(j, h-i) >= 120);
    }

	State start(700, 100, 0);
	State target(100, 600, 0);
    Vehicle car;

	Planner astar;
	vector<state> path = astar.plan(start, target, obs_map, car);

    GUI display(1000, 1000);
    display.draw_obstacles(obs_map);

    for (auto it = begin (path); it != end (path); ++it)
        display.draw_car(*it, car);
    
    display.show();
}