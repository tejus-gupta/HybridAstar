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

	vector <Point> vertices;
	
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
bool Map::checkCollisionSat(vector <Point> v1,vector <Point> v2)
{
	double slope;
	double theta;
	double dis;
	double rmin1,rmax1,rmin2,rmax2;
	rmin1=INT_MAX;
	rmin2=INT_MAX;
	rmax1=INT_MIN;
	rmax2=INT_MIN;
	bool collide=0;
	for (int i=0;i<v1.size()-1;i++)
	{
		slope=(v1[i+1].y-v1[i].y)/(v1[i+1].x-v1[i].x);
		slope=-1*(1/slope);
		for (int j=0;j<v1.size();j++)
		{
			theta=arctan((v1[j].y)/(v1[j].x))-slope;
			dis=sqrt(v1[j].y*v1[j].y+v1[j].x*v1[j].x);
			rmin1=min(rmin1,dis*cos(theta));
			rmax1=max(rmax1,dis*cos(theta));
		}
		for (int j=0;j<v2.size();j++)
		{
			theta=arctan((v2[j].y)/(v2[j].x))-slope;
			dis=sqrt(v2[j].y*v2[j].y+v2[j].x*v2[j].x);
			rmin2=min(rmin2,dis*cos(theta));
			rmax2=max(rmax2,dis*cos(theta));
		}
		if (rmin2>=rmin1&&rmin2<=rmax1)
			collide=true;
		else if (rmin1>=rmin2&&rmin1<=rmax2)
			collide=true;
		if (!collide)
			return false;
		// we assume the line passes through origin and the slope is -1/slope
	}
	for (int i=0;i<v2.size()-1;i++)
	{
		slope=(v2[i+1].y-v2[i].y)/(v2[i+1].x-v2[i].x);
		slope=-1*(1/slope);
		for (int j=0;j<v1.size();j++)
		{
			theta=arctan((v1[j].y)/(v1[j].x))-slope;
			dis=sqrt(v1[j].y*v1[j].y+v1[j].x*v1[j].x);
			rmin1=min(rmin1,dis*cos(theta));
			rmax1=max(rmax1,dis*cos(theta));
		}
		for (int j=0;j<v2.size();j++)
		{
			theta=arctan((v2[j].y)/(v2[j].x))-slope;
			dis=sqrt(v2[j].y*v2[j].y+v2[j].x*v2[j].x);
			rmin2=min(rmin2,dis*cos(theta));
			rmax2=max(rmax2,dis*cos(theta));
		}
		if (rmin2>=rmin1&&rmin2<=rmax1)
			collide=true;
		else if (rmin1>=rmin2&&rmin1<=rmax2)
			collide=true;
		if (!collide)
			return false;
		// we assume the line passes through origin and the slope is -1/slope
	}
	return true;
}
#endif
