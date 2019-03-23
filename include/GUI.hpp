#ifndef GUI_HPP
#define GUI_HPP

#include "Vehicle.hpp"

#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
class GUI{
public:
    int rows;
    int cols;
    int scale;
    Mat display;
    Vehicle car;

    GUI(int map_x, int map_y, float scale);
    void draw_obstacles(int** map, float map_grid_resolution);
    void draw_tree( State state, State next);
    void draw_dubins( vector<State> Path);
    void draw_car(State src, Vehicle car);
    void show(int t);
    void clear();
};

#endif