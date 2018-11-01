#ifndef GUI_HPP
#define GUI_HPP

#include "Vehicle.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
class GUI{
public:
    int rows;
    int cols;
    Mat display;

    GUI(int rows, int cols);
    void draw_obstacles(bool** obs_map,int res);
    void draw_car(State src, Vehicle car);
    void show();
    void show(int t);
};

#endif