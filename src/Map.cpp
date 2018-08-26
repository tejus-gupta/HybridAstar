#ifndef MAP_CPP
#define MAP_CPP

#include "../include/Map.hpp"

Map::Map( bool **obs_map, State end ){

	this->obs_map = obs_map;
	this->end = end;
	this->map_resolution = 10;
	initCollisionChecker();
}

bool Map::isReached(State current){
 	
 	/* In this we could have int in place of bool which tells distance between them so 
 	thst we could act accordingly && fabs(Curr.theta-End.theta)<5*/
 	//
	if( abs(current.x - end.x) < 1 && abs(current.y - end.y) < 1 && (abs(current.theta - end.theta) < 3.14/9 || abs(current.theta - 72 + end.theta) < 3.14/9))
		return true;
 	else return false;
}

void Map::initCollisionChecker(){
	acc_obs_map=new int*[MAPX];
	for(int i=0;i<MAPX;i++)
	{
		acc_obs_map[i]=new int[MAPY];
		for(int j=0;j<MAPY;j++)
			acc_obs_map[i][j]=obs_map[i][j];
	}

	for(int i=0;i<MAPX;i++)
		for(int j=1;j<MAPY;j++)
			acc_obs_map[i][j]=acc_obs_map[i][j-1]+acc_obs_map[i][j];

	for(int j=0;j<MAPY;j++)
		for(int i=1;i<MAPX;i++)
			acc_obs_map[i][j]=acc_obs_map[i-1][j]+acc_obs_map[i][j];

	return;
}

bool Map::checkCollision(State pos){

	if(pos.x*map_resolution>=MAPX || pos.x*map_resolution<0 || pos.y*map_resolution>=MAPY || pos.y*map_resolution<0 )
		return true;

	//first use a bounding box around car to check for collision in O(1) time
	int max_x, min_x, max_y, min_y;
	max_x = map_resolution * (pos.x+car.BOT_L*abs(cos(pos.theta))/2+car.BOT_W*abs(sin(pos.theta))/2) + 1;
	min_x = map_resolution * (pos.x-car.BOT_L*abs(cos(pos.theta))/2-car.BOT_W*abs(sin(pos.theta))/2) - 1;

	max_y= map_resolution * (pos.y+car.BOT_L*abs(sin(pos.theta))/2+car.BOT_W*abs(cos(pos.theta))/2) + 1;
	min_y= map_resolution * (pos.y-car.BOT_L*abs(sin(pos.theta))/2-car.BOT_W*abs(cos(pos.theta))/2) - 1;
	
	if(max_x>=MAPX || min_x<0 || max_y>=MAPY || min_y<0)
		return true;

	if(acc_obs_map[max_x][max_y]+acc_obs_map[min_x][min_y]==acc_obs_map[max_x][min_y]+acc_obs_map[min_x][max_y]){
		return false;
	}

	// brute force check through the car
	for(float i=-car.BOT_L/2.0;i<=car.BOT_L/2.0+0.001;i+=0.25)
		for(float j=-car.BOT_W/2.0;j<=car.BOT_W/2.0+0.001;j+=0.25)
		{
			int s = map_resolution * (pos.x+i*cos(pos.theta*2.0*PI/MAP_THETA)+j*sin(pos.theta*2.0*PI/MAP_THETA)) + 0.001;
			int t = map_resolution * (pos.y+i*sin(pos.theta*2.0*PI/MAP_THETA)+j*cos(pos.theta*2.0*PI/MAP_THETA)) + 0.001;

     		if(obs_map[s][t])
				return true;
		}
	return false;
}
#endif
