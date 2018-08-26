#ifndef MAP_HPP
#define MAP_HPP

#include"../src/Vehicle.cpp"

class Map{
public:
    
    State End;
    bool** obs_map;
    int** acc_obs_map;
       
    int MAP_THETA=72;
    int MAPX=1000;
    int MAPY=1000;
    int VISX=100;
    int VISY=100;

    Vehicle car;

    Map()
    {
        
    }
    Map(bool**,State);
    void initCollisionChecker();
    bool checkCollision(State pos);
    bool isReached(State curr);
};


#endif