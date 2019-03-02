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
    float scale;
    Mat display;
    Vehicle car;

    GUI(int rows, int cols, float scale);
    void draw_obstacles(vector< vector<Point> > polygon);
    void draw_tree( State state, State next);
    void draw_dubins( vector<State> Path);
    void draw_car(State src, Vehicle car);
    void show(int t);
};

#endif