#ifndef MAP_HPP
#define MAP_HPP

#include"../src/Vehicle.cpp"

typedef struct _border{
    int Xmax;
    int Xmin;
    int Ymax;
    int Ymin;
}border;

class Map{
public:
    
    State end;
    vector<vector<bool> > obs_map;
    int** acc_obs_map;
    float map_resolution;
    vector<border> bPoints; 
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
    Map(vector<vector<bool> >   ,State,vector<vector<Point>>,float);
    bool checkCollisionSat(State pos);
    bool helperSAT(vector <Point> v1,vector <Point> v2);
    void initCollisionChecker();
    void initCollisionCheckerSat();
    bool checkCollision(State pos);
    bool isReached(State curr);
};


#endif