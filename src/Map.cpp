#ifndef MAP_CPP
#define MAP_CPP

#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include "../include/Map.hpp"
using namespace cv;

Map::Map( vector< vector<Point> > obs, State end, int rows, int cols)
{
	MAP_THETA=72;
    VISX = rows;
    VISY = cols;

	this->obs = obs;
	this->end = end;
	
	// Converting the Obstacle Map in form of Polygon Array to Costmap 
	// so that point based collision detection can work.
	// Create a map with resolution 0.5m x 0.5m
	vector< vector<Point> > obs_copy;
	obs_map = Mat::zeros(Size(VISX, VISY), CV_8UC1);
	for(int i=0;i < obs.size();i++)
	{
		vector< Point > temp;
		for(int j=0;j < obs[i].size(); j++ )
			temp.push_back( Point {obs[i][j].x, obs[i][j].y});
		obs_copy.push_back(temp);
	}
	drawContours(obs_map, obs_copy, -1, Scalar(255), -1);
	transpose(obs_map,obs_map);
	
	
	initCollisionChecker();
	initCollisionCheckerSat();

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
	if(p.x < 0 || p.y < 0 || p.x >= VISX || p.y >= VISY)
		return false;
	return true; 
}

bool Map::isValid(State current)
{
	if(current.x < 0 || current.y < 0 || current.x >= VISX || current.y >= VISY)
		return false;
	return true; 
}

bool Map::isReached(State current)
{ 	
	if( abs(current.x - end.x) < 1 && abs(current.y - end.y) < 1 && abs(cmod(current.theta,2*M_PI) - end.theta) < M_PI/6 )
		return true;
 	else 
 		return false;
}

void Map::initCollisionChecker(){

	acc_obs_map=new int*[VISX];
	for(int i=0;i<VISX;i++)
	{
		acc_obs_map[i]=new int[VISY];
		for(int j=0;j<VISY;j++)
			acc_obs_map[i][j]=(obs_map.at<uchar>(i,j)==0);
	}

	for(int i=0;i<VISX;i++)
		for(int j=1;j<VISY;j++)
			acc_obs_map[i][j]=acc_obs_map[i][j-1]+acc_obs_map[i][j];

	for(int j=0;j<VISY;j++)
		for(int i=1;i<VISX;i++)
			acc_obs_map[i][j]=acc_obs_map[i-1][j]+acc_obs_map[i][j];

	return;
}

bool Map::checkCollision(State pos){

	if(pos.x>=VISX || pos.x<0 || pos.y>=VISY || pos.y<0 )
		return true;
	

	//first use a bounding box around car to check for collision in O(1) time
	int max_x, min_x, max_y, min_y;
	max_x =  (pos.x+car.BOT_L*abs(cos(pos.theta))/2+car.BOT_W*abs(sin(pos.theta))/2) + 1;
	min_x =  (pos.x-car.BOT_L*abs(cos(pos.theta))/2-car.BOT_W*abs(sin(pos.theta))/2) - 1;

	max_y =  (pos.y+car.BOT_L*abs(sin(pos.theta))/2+car.BOT_W*abs(cos(pos.theta))/2) + 1;
	min_y =  (pos.y-car.BOT_L*abs(sin(pos.theta))/2-car.BOT_W*abs(cos(pos.theta))/2) - 1;

	// Double the co-ordinates as the resolution is twice (0.5x0.5)
	// max_x*=2,max_y*=2,min_y*=2,min_x*=2;
	
	if(max_x>=VISX || min_x<0 || max_y>=VISY || min_y<0)
		return true;

	if(acc_obs_map[max_x][max_y]+acc_obs_map[min_x][min_y]==acc_obs_map[max_x][min_y]+acc_obs_map[min_x][max_y])
		return false;

	// brute force check through the car
	for(float i = -car.BOT_L; i<=car.BOT_L + 0.001; i+=0.5)
		for(float j=-car.BOT_W; j<=car.BOT_W + 0.001; j+=0.5)
		{
			int s = pos.x+i*cos(pos.theta)+j*sin(pos.theta) + 0.001;
			int t = pos.y+i*sin(pos.theta)+j*cos(pos.theta) + 0.001;

     		if( obs_map.at<uchar>(s,t) )
				return true;
		}
	return false;

}

/*
	Here we are trying to create a preliminary check for SAT collision checker .We first find the
	minimum and maximum projection of corner points on coordinate axis and then use coordinate axis
	as our axis to check for collision as in SAT.  
*/
void Map::initCollisionCheckerSat()
{
	bool DEBUG=false;
	for (int i = 0; i < obs.size(); ++i)
	{
		border temp;
		temp.Xmax=temp.Xmin=obs[i][0].x;
		temp.Ymax=temp.Ymin=obs[i][0].y;
		for (int j = 0; j < obs[i].size(); ++j)
		{
			if( obs[i][j].x<temp.Xmin )
				temp.Xmin=obs[i][j].x;
			if( obs[i][j].x>temp.Xmax )
				temp.Xmax=obs[i][j].x;
			if( obs[i][j].y<temp.Ymin )
				temp.Ymin=obs[i][j].y;
			if( obs[i][j].y>temp.Ymax )
				temp.Ymax=obs[i][j].y;
		}
		cout<<"obs.size(): "<<obs.size()<<endl;
		if(DEBUG)
			cout<<temp.Xmax<<" "<<temp.Xmin<<" "<<temp.Ymax<<" "<<temp.Ymin<<endl;
		bPoints.push_back(temp);
	}
}


bool Map::checkCollisionSat(State pos)
{
	bool DEBUG=false;

	if(DEBUG)
		cout<<"Inside checkCollisionSat "<<endl;

	if(pos.x >= VISX || pos.x<0 || pos.y >= VISY || pos.y<0 )
		return true;

	// We are trying to find the four corner points of our Bot 
	vector<Point> v1;

	Point p1;
	p1.x = pos.x-car.BOT_L*abs(cos(pos.theta))/2-car.BOT_W*abs(sin(pos.theta))/2 ;
	p1.y = pos.y-car.BOT_L*abs(sin(pos.theta))/2+car.BOT_W*abs(cos(pos.theta))/2 ;
	v1.push_back(p1);
	if(DEBUG)
		cout<<"Point : "<<p1.x<<" "<<p1.y<<endl;

	Point p2;
	p2.x = pos.x+car.BOT_L*abs(cos(pos.theta))/2-car.BOT_W*abs(sin(pos.theta))/2 ;
	p2.y = pos.y+car.BOT_L*abs(sin(pos.theta))/2+car.BOT_W*abs(cos(pos.theta))/2 ;
	v1.push_back(p2);
	if(DEBUG)
		cout<<"Point : "<<p2.x<<" "<<p2.y<<endl;
	

	Point p3;
	p3.x = pos.x+car.BOT_L*abs(cos(pos.theta))/2+car.BOT_W*abs(sin(pos.theta))/2 ;
	p3.y = pos.y+car.BOT_L*abs(sin(pos.theta))/2-car.BOT_W*abs(cos(pos.theta))/2 ;
	v1.push_back(p3);
	if(DEBUG)
		cout<<"Point : "<<p3.x<<" "<<p3.y<<endl;
	

	Point p4;
	p4.x = pos.x-car.BOT_L*abs(cos(pos.theta))/2+car.BOT_W*abs(sin(pos.theta))/2 ;
	p4.y = pos.y-car.BOT_L*abs(sin(pos.theta))/2-car.BOT_W*abs(cos(pos.theta))/2 ;
	v1.push_back(p4);
	if(DEBUG)
		cout<<"Point : "<<p4.x<<" "<<p4.y<<endl;
	
	/*
	 We are trying to find the maximum and minimum corner points of our bot 
	 corresponding to our coordinate axis(projection is on our coordinate axis).
	*/
	int Xmax,Xmin,Ymax,Ymin;
	Xmax=Xmin=v1[0].x;
	Ymax=Ymin=v1[0].y;
	for (int j = 1; j < v1.size(); ++j)
	{
		if( v1[j].x < Xmin )
			Xmin=v1[j].x;
		if( v1[j].x > Xmax )
			Xmax=v1[j].x;
		if( v1[j].y < Ymin )
			Ymin=v1[j].y;
		if( v1[j].y > Ymax )
			Ymax=v1[j].y;
	}
	if(DEBUG)
		cout<<Xmax<<" "<<Xmin<<" "<<Ymax<<" "<<Ymin<<endl;

	// cout<<"obs.size() "<<obs.size()<<endl;
	for (int i = 0; i < obs.size() ; ++i)
	{
		if(DEBUG)
		{
			cout<<(bPoints[i].Xmax<Xmin )<<" "<<( bPoints[i].Xmin>Xmax )<<" "<<( bPoints[i].Ymin>Ymax )<<" "<<(bPoints[i].Ymax<Ymin ) <<endl; 
			cout<<bPoints[i].Xmax<<" "<<Xmin<<" "<< bPoints[i].Xmin<<" "<<Xmax <<" "<< bPoints[i].Ymin<<" "<<Ymax <<" "<<bPoints[i].Ymax<<" "<<Ymin  <<endl; 
		}
		
		if( !(bPoints[i].Xmax<Xmin || bPoints[i].Xmin>Xmax || bPoints[i].Ymin>Ymax || bPoints[i].Ymax<Ymin ) ) 
		{
			if( helperSAT( v1 , obs[i] ) )
				return true;
		}
		else
		{
			if(DEBUG) 
				cout<<"Used extreme boundary checking"<<endl;
		}
	}

	return false;
}


// v1 is bot and v2 is obstacle
// we assume the line passes through origin and the slope is -1/slope
bool Map::helperSAT(vector <Point> v1,vector <Point> v2)
{
	bool DEBUG=false;
	
	if(DEBUG)
		cout<<"Inside Helper SAT "<<endl;
	
	double slope,theta,alpha;
	double dis;
	double rmin1,rmax1,rmin2,rmax2;

	// Covering all the axis of BOT 
	for (int i=0;i<v1.size();i++)
	{
		rmin1=rmin2=INT_MAX;
		rmax1=rmax2=INT_MIN;

		if( (v1[i+1].x==v1[i].x)) 
		{
			alpha=0;
			slope=0;
		}
		else if(v1[i+1].y-v1[i].y==0)
		{
			alpha=CV_PI/2;
			slope=INT_MAX;
		}
		else 
		{
			slope=(double)(v1[i+1].y-v1[i].y)/(v1[i+1].x-v1[i].x);
			slope=-1*(1/slope);
			if (slope<0)
				alpha=CV_PI+atan(slope);
			else
				alpha=atan(slope);
		}
		/*
			If we take slope is zero and alpha also zero we get projection of points on "our" X-axis .
		*/
		if(DEBUG)
			cout<<"Slope of the axis for this run : "<<slope<<" and corresponding angle is alpha : "<<alpha<<endl;

		int count=0;
		for (int j=0;j<v1.size();j++)
		{
			if(v1[j].x==0)
				theta=alpha-CV_PI/2;
			else
				theta=alpha-atan((double)v1[j].y/(double)v1[j].x);
			if( theta>CV_PI/2 )
				theta=atan((double)v1[j].y/(double)v1[j].x)-alpha+CV_PI;

			dis=sqrt(v1[j].y*v1[j].y+v1[j].x*v1[j].x);
			dis=( (v1[j].y-(-1/slope)*v1[j].x) >= 0 )? dis : -1*dis ;
			
			rmin1=min(rmin1,dis*cos(theta));
			rmax1=max(rmax1,dis*cos(theta));
		}
		if(DEBUG)
			cout<<"rmin1: "<<rmin1<<" rmax1: "<<rmax1<<endl;

		for (int j=0;j<v2.size();j++)
		{
			if(v2[j].x==0)
				theta=alpha-CV_PI/2;
			else
				theta=alpha-atan((double)v2[j].y/(double)v2[j].x);
			if( theta>CV_PI/2 )
				theta=atan((double)v2[j].y/(double)v2[j].x)-alpha+CV_PI;

			dis=sqrt(v2[j].y*v2[j].y+v2[j].x*v2[j].x);
			dis=( (v2[j].y-(-1/slope)*v2[j].x) >= 0 )? dis : -1*dis ;

			rmin2=min(rmin2,dis*cos(theta));
			rmax2=max(rmax2,dis*cos(theta));

		}
		if(DEBUG)
			cout<<"rmin2: "<<rmin2<<" rmax2: "<<rmax2<<endl;

		if ( rmin1>rmax2 || rmin2>rmax1 ) 
		{
			return false;
		}

	}

	// Covering all the axis of the obstacle in this function call .
	for (int i=0;i<v2.size();i++)
	{
		rmin1=rmin2=INT_MAX;
		rmax1=rmax2=INT_MIN;

		if( (v2[i+1].x==v2[i].x)) 
		{
			alpha=0;
			slope=0;
		}
		else if(v2[i+1].y-v2[i].y==0)
		{
			alpha=CV_PI/2;
			slope=INT_MAX;
		}
		else 
		{
			slope=(double)(v2[i+1].y-v2[i].y)/(v2[i+1].x-v2[i].x);
			slope=-1*(1/slope);
			if (slope<0)
				alpha=CV_PI+atan(slope);
			else
				alpha=atan(slope);
		}
		if(DEBUG)
			cout<<"slope of the axis for this run : "<<slope<<" and alpha : "<<alpha<<endl;

		int count=0;
		for (int j=0;j<v1.size();j++)
		{
			if(v1[j].x==0)
				theta=alpha-CV_PI/2;
			else
				theta=alpha-atan((double)v1[j].y/(double)v1[j].x);
			if( theta>CV_PI/2 )
				theta=atan((double)v1[j].y/(double)v1[j].x)-alpha+CV_PI;

			dis=sqrt(v1[j].y*v1[j].y+v1[j].x*v1[j].x);
			dis=( (v1[j].y-(-1/slope)*v1[j].x) >= 0 )? dis : -1*dis ;

			rmin1=min(rmin1,dis*cos(theta));
			rmax1=max(rmax1,dis*cos(theta));
		}
		if(DEBUG)
			cout<<"rmin1: "<<rmin1<<" rmax1: "<<rmax1<<endl;


		for (int j=0;j<v2.size();j++)
		{
			if(v2[j].x==0)
				theta=alpha-CV_PI/2;
			else
				theta=alpha-atan((double)v2[j].y/(double)v2[j].x);
			if( theta>CV_PI/2 )
				theta=atan((double)v2[j].y/(double)v2[j].x)-alpha+CV_PI;

			dis=sqrt(v2[j].y*v2[j].y+v2[j].x*v2[j].x);
			dis=( (v2[j].y-(-1/slope)*v2[j].x) >= 0 )? dis : -1*dis ;

			rmin2=min(rmin2,dis*cos(theta));
			rmax2=max(rmax2,dis*cos(theta));

		}
		if(DEBUG)		
			cout<<"rmin2: "<<rmin2<<" rmax2: "<<rmax2<<endl;


		if ( rmin1>rmax2 || rmin2>rmax1 ) 
		{
			return false;
		}
	}

	return true;

}

#endif
