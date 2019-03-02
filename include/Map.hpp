#ifndef MAP_HPP
#define MAP_HPP

#include"../src/Vehicle.cpp"

typedef struct _border
{
    int Xmax;
    int Xmin;
    int Ymax;
    int Ymin;
}border;

class Map
{
    public:
        
        State end;
        vector<border> bPoints; 
        vector< vector<Point> > obs;
        
        int MAP_THETA;
        int VISX;
        int VISY;

        Vehicle car;

        Map()
        {
            
        }

        Map(vector< vector<Point> > polygon, State , int, int);
        bool checkCollisionSat(State pos);
        bool helperSAT(vector <Point> v1,vector <Point> v2);
        void initCollisionCheckerSat();
        bool isReached(State curr);
	bool isValid(Point);
};


#endif
