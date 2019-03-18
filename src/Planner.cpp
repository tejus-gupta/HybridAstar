#include "../include/GUI.hpp"
#include "../include/Planner.hpp"

#include <limits.h>
#include <pthread.h>
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>

// For storing and using the value of Dijkstra Heuristic Cost inside the Planner Operator()
vector< vector< double > > H;

// For timing Planner::operator()
double t=0;

// These two are necessary for functioning of Planner::operator() 
Vehicle veh;
State target;


// We are calculating the values of Dubins Cost in two seperate threads:
bool PriQ::operator()(State a,State b)
{
	Heuristic h;
	double a_dubinsCost, b_dubinsCost;

	a_dubinsCost =  h.DubinCost(a, target, veh.min_radius);
	b_dubinsCost =  h.DubinCost(b, target, veh.min_radius);
		
	// Calculating max of Dubin's Cost and Djikstra's Cost  
	clock_t start_time=clock();
	double temp_a=max(H[(int)a.x][(int)a.y],a_dubinsCost);
	double temp_b=max(H[(int)b.x][(int)b.y],b_dubinsCost);
	clock_t end_time=clock();

	// In this conditional loop we are trying force the selection of node with equal gCost + fCost to the one
	// with greater gCost. By this the node expansion speeds up as it takes the node that is closer to the target.
	t+=(double)(end_time-start_time)/CLOCKS_PER_SEC;
	if(a.cost2d+temp_a >= b.cost2d+temp_b)
	{
		if(a.cost2d+temp_a > b.cost2d+temp_b)
			return true;
		else return (a.cost2d < b.cost2d);
	}
	else return false;

}

vector<State> Planner::plan(State start, State end, Vehicle car, vector<vector<Point> > obs, GUI display)
{

	bool DEBUG = false;
	Map map(obs, end, rows, cols);                         

	// This is done for functioning of Planner::Operator().
	veh = car;
	target = end;                         
	
	// DEBUG true will cause image of start pose, end pose and obstacles to display. 
	if(DEBUG)
	{
	    display.draw_obstacles(obs);
	    display.draw_car(start, car);
	    display.draw_car(end, car);
	    display.show(10);
	}   

	// Array of states allocation
	clock_t time_begin= clock();
	visited_state.resize(500,vector< vector< State > >(500,vector< State >(72)));	
	clock_t time_end= clock();
	cout<<"Time: Array of States Allocation = "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// Array of visited allocation
	time_begin= clock();
	visited.resize(500,vector< vector< bool > >(500,vector< bool >(72,false)));
	time_end= clock();
	cout<<"Time: Visited Array of States Allocation = "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// Djikstra Calculation
	time_begin= clock();
	h_obj.Dijkstra(map,end);
	time_end= clock();
	cout<<"Time: Dijkstra= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// Djikstra Copying
	time_begin= clock();
	H.resize(map.VISX, vector<double>(map.VISY));
	for(int i=0;i<map.VISX;i++)
	    for(int j=0;j<map.VISY;j++)
			H[i][j]=h_obj.h_vals[i][j].dis;
	time_end= clock();
	cout<<"Time: Dijkstra Copying= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;
			
	// This will cause display of Dijkstra Cost on an image with black representing negligible cost to 
	// white representing high Dijkstra Cost.		

	// if(DEBUG)
	// {		
	// 	Mat dijImage(rows*display.scale,cols*display.scale,CV_8UC1,Scalar(0));
	// 	for(int i=0;i<map.VISX*display.scale;i++)
	// 	{
	// 	    for(int j=0;j<map.VISY*display.scale;j++)
	// 		{
	// 			dijImage.at<uchar>(i,j)=((int)(H[(int)(i*1.0/display.scale)][(int)(j*1.0/display.scale)]))%256;
	// 		}	

	// 	}
	// 	imshow("Dijkstra Cost",dijImage);
	// 	waitKey(10);
	// }

	// Memory freeing
	time_begin = clock();
	for(int i=0;i<map.VISX;i++)
		delete[] h_obj.h_vals[i];
	delete[] h_obj.h_vals;	
	time_end = clock();
	cout<<"Time: Memory freeing= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// To mark the visited states. VISX, VISY and MAP_THETA are to be imported from the Map class
	time_begin = clock();
	for(int i=0; i < map.VISX; i++)
		for(int j=0; j < map.VISY; j++)
			for(int k=0; k < map.MAP_THETA; k++ )
				visited[i][j][k]=false;
	time_end = clock();
	cout<<"Time: Marking visited array false= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// Hybrid Astar Openlist Initiates:
	priority_queue <State, vector<State>, PriQ> pq;
	pq.push(start);

	double checkCollisionTime=0;
	double nextStatesTime=0;
	
	int count=0;
	while(!pq.empty())
	{
		if(DEBUG)
			cout<<"Inside While"<<endl;

		State current=pq.top();
		pq.pop();

		// Angles are taken from 0-360 in steps of 5 so MAP_THETA is 72 also 
		// theta attribute of State stores angle in radian form .
		int grid_theta=((int)(current.theta*map.MAP_THETA/(2*M_PI)))%map.MAP_THETA; 
		if( visited[(int)current.x][(int)current.y][grid_theta] )
			continue;
		visited[(int)current.x][(int)current.y][grid_theta] = true;
		visited_state[(int)current.x][(int)current.y][grid_theta] = current;

		if(DEBUG)
			cout<<"Current State "<<current.x<<" "<<current.y<<" "<<current.theta<<endl;	

		// Checks if it has reached the goal
		if(map.isReached(current))
		{
			cout<<"Time :CollisionChecker= "<<checkCollisionTime<<endl;
			cout<<"Time :nextStates= "<<nextStatesTime<<endl;
			cout<<"Time :Dubins on Spot = "<<t<<endl;
			cout<<"REACHED!"<<endl;
			
			State temp=current;
			while( temp.parent != NULL )
			{
				path.push_back(temp);
				temp =*(temp.parent);
			}
			reverse(path.begin(), path.end());			
			return path;
		}
		// else 
		// 	cout<<"Not Reached"<<endl;

		// This section finds the next states based on trajecory generation.
		time_begin=clock();
		vector<State> next=car.nextStates(&current);
		time_end=clock();
		nextStatesTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;

		for(vector<State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State nextS = *it;
			if( !map.isValid(nextS))
				continue;

			int next_theta=((int)(nextS.theta*180/(M_PI*5)))%72;
				
			if( visited[(int)nextS.x][(int)nextS.y][next_theta] )
				continue;
			
			time_begin=clock();
			if( !map.checkCollisionSat(nextS) )   //change
			{
				time_end=clock();
				it->parent = &(visited_state[(int)current.x][(int)current.y][grid_theta]);
				it->cost2d = current.cost2d+1;

				
				if(DEBUG)
			    {
					display.draw_tree(current,nextS);
		        	display.show(1);
			    }

				pq.push(*it);
			}
			else
				time_end=clock();

			checkCollisionTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
		}

		// At every few iterations we try to calculate the Dubins Path and add the states 
		// to the openlist. Once it collides we stop further iterations. 
		if( count%4==3 )
		{	
			
			time_begin=clock();
			vector<State> Path = h_obj.DubinShot(current,end,car.min_radius);
			time_end=clock();
			nextStatesTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;

			State prev=current,check=current;
			for(vector<State>::iterator it= Path.begin(); it!=Path.end();it++)
			{
				State nextS = *it;
				
				if( !map.isValid(nextS))
					continue;

				if(map.isReached(nextS))
				{
					cout<<"Time :CollisionChecker= "<<checkCollisionTime<<endl;
					cout<<"Time :nextStates= "<<nextStatesTime<<endl;
					cout<<"Time :Dubins on Spot = "<<t<<endl;
					cout<<"REACHED!"<<endl;

					State temp=check;
					while( temp.parent != NULL )
					{
						path.push_back(temp);
						temp= *(temp.parent);
					}
					reverse(path.begin(), path.end());
					return path;
				
				}

				// This is to insure that consecutive points are not very close .Because of being very close 
				// consecutive points were assigned same parents and caused problems while storing path.
				if( sqrt(pow(nextS.x-prev.x,2) + pow(nextS.y-prev.y,2)) < 2 )
					continue;
				
				time_begin=clock();
				if( !map.checkCollisionSat(nextS) ) //change
				{

					if(DEBUG)
						cout<<"Not collided"<<endl;
					
					time_end=clock();

					int prev_theta=((int)(prev.theta*map.MAP_THETA/(M_PI*2)))%map.MAP_THETA;
					
					it->parent = &(visited_state[(int)prev.x][(int)prev.y][(int)prev_theta]);
					it->cost2d = prev.cost2d+1;
					
					// it->gx=it->x,it->gy=it->y;

					int next_theta=((int)(nextS.theta*map.MAP_THETA/(M_PI*2)))%map.MAP_THETA;
					visited_state[(int)nextS.x][(int)nextS.y][(int)next_theta] = *it;

					if(DEBUG)
					{
						cout<<"Check prev : "<<prev.x<<" "<<prev.y<<" "<<prev.theta<<endl; 
						cout<<"Check nextS : "<<nextS.x<<" "<<nextS.y<<" "<<nextS.theta<<endl; 
						cout<<"Check parent : "<<(it->parent)->x<<" "<<(it->parent)->y<<" "<<(it->parent)->theta<<endl; 
					}

					check=*it;
					prev=nextS;
					checkCollisionTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
					pq.push(*it);
				}
				else
				{
				    if(DEBUG)
					{
						cout<<"Collided "<<endl;
						Path.erase(it,Path.end());
						display.draw_dubins( Path);
					    display.show(5);
					}

					time_end=clock();
					checkCollisionTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
					break;
				}

			}
		}

		if(DEBUG)
        	display.show(5);
		
		count++;
	}
	cout<<"Goal cannot be reached"<<endl;
	exit(0);
}



