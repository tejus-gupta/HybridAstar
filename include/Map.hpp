#ifndef MAP_HPP
#define MAP_HPP

#include"../src/Vehicle.cpp"

class Map{
public:
    
    State end;
    bool** obs_map;
    int** acc_obs_map;
    float map_resolution;
    vector< vector<Point> > obs;
       
    int MAP_THETA=72;
    int MAPX=1000;
    int MAPY=1000;
    int VISX=100;
    int VISY=100;

    Vehicle car;

    Map()
    {
        
    }
    Map(bool**,State,vector<vector<Point>> );
    bool helperSAT(vector <Point> v1,vector <Point> v2);
    void initCollisionChecker();
    bool checkCollision(State pos);
    bool isReached(State curr);
    bool checkCollisionSat(State pos);

};


#endif