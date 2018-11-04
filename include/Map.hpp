#ifndef MAP_HPP
#define MAP_HPP

#include"../src/Vehicle.cpp"

class Map{
public:
    
    State end;
    bool** obs_map;
    int** acc_obs_map;
    float map_resolution;
    vector<vector<Point> > obs;
       
    int MAP_THETA;
    int MAPX;
    int MAPY;
    int VISX;
    int VISY;

    Vehicle car;

    Map()
    {
        
    }
    Map(bool**,State,vector<vector<Point>>,float);
    bool checkCollisionSat(State pos);
    bool helperSAT(vector <Point> v1,vector <Point> v2);
    void initCollisionChecker();
    bool checkCollision(State pos);
    bool isReached(State curr);
};


#endif