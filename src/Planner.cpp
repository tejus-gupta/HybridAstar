#include "../include/Planner.hpp"
// #include "../include/GUI.hpp"

double **H;
double t=0;
State target;
Vehicle veh;

bool Planner::operator()(State a,State b)
{
	// Calculating max of Dubin's and Djikstra's
	clock_t start_time=clock();
	double temp_a=max(H[(int)a.x][(int)a.y],h_obj.Dubin_cost(a,target,veh.min_radius));
	double temp_b=max(H[(int)b.x][(int)b.y],h_obj.Dubin_cost(b,target,veh.min_radius));
	clock_t end_time=clock();

	t+=(double)(end_time-start_time)/CLOCKS_PER_SEC;
	return (a.cost2d+temp_a > b.cost2d+temp_b);
}

vector<State> Planner::plan(State start, State end, bool** obs_map, Vehicle car,vector<vector<Point> > obs,float scale)
{

	bool DEBUG = false;
	GUI display(1000, 1000);
	Map map(obs_map, end , obs, scale);                          //Object of Map class

	veh = car;
	target = end;                         
	if(DEBUG)
	{
	    display.draw_obstacles(obs_map, scale);
	    display.draw_car(start, car, scale);
	    display.draw_car(end, car, scale);
		display.show();
	}   

    // State stat(220,290,0);
	// display.draw_car(stat, car, scale);
    // cout<<map.checkCollisionSat(stat)<<endl;
	// exit(0);

	// Djikstra Calculation
	clock_t time_begin= clock();
	h_obj.Dijkstra(map,end);
	clock_t time_end= clock();
	cout<<"Time: Dijkstra= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	time_begin= clock();
	H=new double*[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		H[i]=new double[map.VISY];
	    for(int j=0;j<map.VISY;j++)
			H[i][j]=h_obj.h_vals[i][j].dis;		
	}
	time_end= clock();
	cout<<"Time: Dijkstra Copying= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// Array of states
	time_begin= clock();
	State*** visited_state=new State**[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		visited_state[i]=new State*[map.VISY];
		for(int j=0;j<map.VISY;j++)
		{
			visited_state[i][j]=new State[map.MAP_THETA];
		}
	}
	time_end= clock();
	cout<<"Time: Array of States Allocation = "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// To mark the visited states MAPX, MAPY and MAP_THETA are to be imported from the Map class
	time_begin= clock();
	bool*** visited=new bool**[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		visited[i]=new bool*[map.VISY];
		for(int j=0;j<map.VISY;j++)
		{
			visited[i][j]=new bool[map.MAP_THETA];
			for(int k=0;k<map.MAP_THETA;k++)
			{
				visited[i][j][k]=false;
			}
		}
	}
	time_end= clock();
	cout<<"Time: Visited Array of States Allocation = "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;


	priority_queue <State, vector<State>, Planner> pq;
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

		// grid_theta varies from 0-71 
		// current.theta varies from 0-CV_PI

		int grid_theta=((int)(current.theta*map.MAP_THETA/(2*PI)))%map.MAP_THETA; 
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
			cout<<"Time :Dubins on spot = "<<t<<endl;
			cout<<"REACHED!"<<endl;
			
			State temp=current;

			while( temp.parent != NULL )
			{
				path.push_back(temp);
				temp=*(temp.parent);
			}
			reverse(path.begin(), path.end());
			return path;
		}

		if( count%4!=0 )
		{

			time_begin=clock();
			vector<State> next=car.nextStates(&current,scale);
			time_end=clock();
			nextStatesTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;

			for(vector<State>::iterator it= next.begin(); it!=next.end();it++)
			{
				State nextS = *it;
				int next_theta=((int)(nextS.theta*180/(PI*5)))%72;
				
				if( visited[(int)nextS.x][(int)nextS.y][next_theta] )
					continue;
				
				time_begin=clock();
				if( !map.checkCollisionSat(nextS) )
				{
					time_end=clock();
					it->parent = &(visited_state[(int)current.x][(int)current.y][grid_theta]);
					it->cost2d = current.cost2d+1;
					
					if(DEBUG)
				    {
						display.draw_tree(current,nextS,1);
			        	display.show(5);
				    }
					
					pq.push(*it);
				}
				else
					time_end=clock();

				checkCollisionTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
			}

		}
		else{
			
			time_begin=clock();
			vector<State> Path = h_obj.DubinShot(current,end,car.min_radius);
			time_end=clock();
			nextStatesTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;

			if(DEBUG)
			{
				display.draw_dubins( Path,scale );
			    display.show(5000);
			}

			State prev=current,check=current;
			for(vector<State>::iterator it= Path.begin(); it!=Path.end();it++)
			{
				State nextS = *it;

				if(map.isReached(check))
				{
					cout<<"Time :CollisionChecker= "<<checkCollisionTime<<endl;
					cout<<"Time :nextStates= "<<nextStatesTime<<endl;
					cout<<"Time :Dubins on spot = "<<t<<endl;
					cout<<"REACHED!"<<endl;
					
					State temp=check;
					while( temp.parent != NULL )
					{
						path.push_back(temp);
						temp=*(temp.parent);
					}
					reverse(path.begin(), path.end());
					return path;
				
				}

				if( sqrt(pow(nextS.x-prev.x,2) + pow(nextS.y-prev.y,2)) < 2 )
					continue;
				
				time_begin=clock();
				if( !map.checkCollisionSat(nextS) )
				{
					if(DEBUG)
						cout<<"Not collided"<<endl;
					
					time_end=clock();

					int prev_theta=((int)(prev.theta*180/(PI*5)))%72;
					
					it->parent = &(visited_state[(int)prev.x][(int)prev.y][(int)prev_theta]);
					it->cost2d = prev.cost2d+1;
					
					int next_theta=((int)(nextS.theta*180/(PI*5)))%72;
					visited_state[(int)nextS.x][(int)nextS.y][(int)next_theta] = *it;

					if(DEBUG)
					{
						cout<<"Check nextS : "<<nextS.x<<" "<<nextS.y<<" "<<nextS.theta<<endl; 
						cout<<"Check prev : "<<prev.x<<" "<<prev.y<<" "<<prev.theta<<endl; 
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
						cout<<"Collided "<<endl;

					time_end=clock();
					checkCollisionTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
					break;
				}

			}
		}
		if(DEBUG)
        	display.show(1);
		
		count++;
	}
	cout<<"Goal cannot be reached"<<endl;
	exit(0);
}



