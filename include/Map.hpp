#ifndef MAP_HPP
#define MAP_HPP

#include"../include/Vehicle.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

class Map
{
    public:
        
        State end;
        Vehicle car;

        int** obs;
        int** acc_obs_map;

        int map_x;
        int map_y;

        float map_grid_resolution;

        int map_grid_x;
        int map_grid_y;

        Map();

        Map(int** obs, int map_x, int map_y, float map_grid_resolution, State end, Vehicle car);

        bool isReached(State curr);
        bool isValid(Point);
        bool isValid(State);

        void initCollisionChecker();
        bool checkCollision(State pos);

};
#endif
