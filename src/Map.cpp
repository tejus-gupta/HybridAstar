#include "../include/Map.hpp"
using namespace cv;

Map::Map()
{
	return;
}

Map::Map(int** obs, int map_x, int map_y, float map_grid_resolution, State end, Vehicle car)
{
	this->obs = obs;
	this->map_x = map_x;
	this->map_y = map_y;
	this->map_grid_resolution = map_grid_resolution;
	this->end = end;
	this->car = car;

	map_grid_x = toInt(map_x/map_grid_resolution);
	map_grid_y = toInt(map_y/map_grid_resolution);
	
	initCollisionChecker();
}

double cmod(double a, double b)
{
	a = fmod(a,b);
	if(a<0)
		a+=b;
	return a;
}

bool Map::isValid(Point p)
{
	if(p.x < 0 || p.y < 0 || p.x >= 100 || p.y >= 100)
		return false;
	return true; 
}

//used where?
bool Map::isValid(State current)
{
	if(current.x < 0 || current.y < 0 || current.x >= map_x || current.y >= map_y)
		return false;
	return true; 
}

bool Map::isReached(State current)
{ 	
	//possible bug
	//need to check from both side
	if( abs(current.x - end.x) < 1 && abs(current.y - end.y) < 1 && abs(cmod(current.theta,2*M_PI) - end.theta) < M_PI/6 )
		return true;
 	else 
 		return false;
}

void Map::initCollisionChecker()
{
	acc_obs_map=new int*[map_grid_x];
	for(int i=0;i<map_grid_x;i++)
	{
		acc_obs_map[i]=new int[map_grid_y];
		for(int j=0;j<map_grid_y;j++)
			acc_obs_map[i][j]=(obs[i][j]>0);
	}

	for(int i=0;i<map_grid_x;i++)
		for(int j=1;j<map_grid_y;j++)
			acc_obs_map[i][j]=acc_obs_map[i][j-1]+acc_obs_map[i][j];

	for(int j=0;j<map_grid_x;j++)
		for(int i=1;i<map_grid_y;i++)
			acc_obs_map[i][j]=acc_obs_map[i-1][j]+acc_obs_map[i][j];

	return;
}

bool Map::checkCollision(State pos)
{
	if(pos.x>=map_x || pos.x<0 || pos.y>=map_y || pos.y<0 )
		return true;
	
	//first use a bounding box around car to check for collision in O(1) time
	int max_x, min_x, max_y, min_y;
	max_x =  (pos.x+car.BOT_L*abs(cos(pos.theta))/2+car.BOT_W*abs(sin(pos.theta))/2) + 1;
	min_x =  (pos.x-car.BOT_L*abs(cos(pos.theta))/2-car.BOT_W*abs(sin(pos.theta))/2) - 1;

	max_y =  (pos.y+car.BOT_L*abs(sin(pos.theta))/2+car.BOT_W*abs(cos(pos.theta))/2) + 1;
	min_y =  (pos.y-car.BOT_L*abs(sin(pos.theta))/2-car.BOT_W*abs(cos(pos.theta))/2) - 1;

	//possible bug
	//need to do brute force check (with boundary checking) in this case
	if(max_x>=map_x || min_x<0 || max_y>=map_y || min_y<0)
		return true;

	// coordinates in meters to indices in map_grid
	max_x /= map_grid_resolution;
	max_y /= map_grid_resolution;
	min_y /= map_grid_resolution;
	min_x /= map_grid_resolution;
	
	if(acc_obs_map[max_x][max_y]+acc_obs_map[min_x][min_y]==acc_obs_map[max_x][min_y]+acc_obs_map[min_x][max_y])
		return false;

	// brute force check through the car
	for(float i = -car.BOT_L/2; i<=car.BOT_L/2 + 0.001; i+=0.25)
		for(float j=-car.BOT_W/2; j<=car.BOT_W/2 + 0.001; j+=0.25)
		{
			int s = (pos.x+i*cos(pos.theta)+j*sin(pos.theta))/map_grid_resolution + 0.001;
			int t = (pos.y+i*sin(pos.theta)+j*cos(pos.theta))/map_grid_resolution + 0.001;

     		if( obs[s][t]!=0 )
				return true;
		}
	return false;
}
