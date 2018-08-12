#include "../include/Map.hpp"


Map::Map( bool **obsmap, State end){

	End=end; 
	obs_map=new bool*[MAPX];
	for(int i=0;i<MAPX;i++)
	{
		obs_map[i]=new bool[MAPY];
		for(int j=0;j<MAPY;j++)
		{
			obs_map[i][j]=obsmap[i][j];
		}
	}
	cout<<"Cost Map Initialized!"<<endl;
}

bool Map::isReached(State Curr){
 	
 	/* In this we could have int in place of bool which tells distance between them so 
 	thst we could act accordingly */
 	if(  sqrt(pow(Curr.x-End.x,2)+ pow(Curr.y-End.y,2))<5 && fabs(Curr.theta-End.theta)<5 ) return true;
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

	cout<<"Collision checker initialized!"<<endl;
	return;
}

bool Map::checkCollision(State pos){

	if(pos.x>=MAPX || pos.x<0 || pos.y>=MAPY || pos.y<0 )
		return true;

	// if(pos.gx>=GX || pos.gx<0 || pos.gy>=GY || pos.gy<0 )
	// 	return true;

	//first use a bounding box around car to check for collision in O(1) time
	int max_x, min_x, max_y, min_y;
	max_x=pos.x+BOT_L*abs(cos(pos.theta*2*PI/MAP_THETA))/2+BOT_W*abs(sin(pos.theta*2*PI/MAP_THETA))/2+1;
	min_x=pos.x-BOT_L*abs(cos(pos.theta*2*PI/MAP_THETA))/2-BOT_W*abs(sin(pos.theta*2*PI/MAP_THETA))/2-1;

	max_y=pos.y+BOT_L*abs(sin(pos.theta*2*PI/MAP_THETA))/2+BOT_W*abs(cos(pos.theta*2*PI/MAP_THETA))/2+1;
	min_y=pos.y-BOT_L*abs(sin(pos.theta*2*PI/MAP_THETA))/2-BOT_W*abs(cos(pos.theta*2*PI/MAP_THETA))/2-1;

	if(max_x>=MAPX || min_x<0 || max_y>=MAPY || min_y<0)
		return true;

	if(acc_obs_map[max_x][max_y]+acc_obs_map[min_x][min_y]==acc_obs_map[max_x][min_y]+acc_obs_map[min_x][max_y]){
		return false;
	}

	// //brute force check through the car
	// for(float i=-BOT_L/2.0;i<=BOT_L/2.0+0.001;i++)
	// 	for(float j=-BOT_W/2.0;j<=BOT_W/2.0+0.001;j++)
	// 	{
	// 		int s=pos.x+i*cos(pos.theta*2.0*PI/MAP_THETA)+j*sin(pos.theta*2.0*PI/MAP_THETA)+0.001;
	// 		int t=pos.y+i*sin(pos.theta*2.0*PI/MAP_THETA)+j*cos(pos.theta*2.0*PI/MAP_THETA)+0.001;

 //     		if(obs_map[s][t] || obs_map[s+1][t] || obs_map[s][t+1] || obs_map[s+1][t+1] || obs_map[s-1][t-1] || obs_map[s-1][t] || obs_map[s][t-1] || obs_map[s-1][t+1] || obs_map[s+1][t-1]){
	// 			return true;
	// 		}
	// 	}
	// return false;
}
