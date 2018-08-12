#ifndef GUI_HPP
#define GUI_HPP

#include "State.hpp"

class GUI{
public:
    int rows;
    int cols;
    Mat display;

    GUI(int rows, int cols);
    void draw_obstacles(int** obs_map);
    void draw_car(State src, Vehicle car);
    void show();
};

#endif