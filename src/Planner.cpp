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
	// double temp_a=H[a.gx][a.gy];
	// double temp_b=H[b.gx][b.gy];
	double temp_a=max(H[a.gx][a.gy],h_obj.Dubin_cost(a,target,veh.min_radius));
	double temp_b=max(H[b.gx][b.gy],h_obj.Dubin_cost(b,target,veh.min_radius));
	clock_t end_time=clock();

	t+=(double)(end_time-start_time)/CLOCKS_PER_SEC;
	return (a.cost2d+temp_a > b.cost2d+temp_b);
}

double dis (State a,State b)
{
	return (sqrt((b.gx-a.gx)*(b.gx-a.gx)+(b.gy-a.gy)*(b.gy-a.gy)));
}

vector<State> Planner::plan(State start, State end, bool** obs_map, Vehicle car,vector<vector<Point> > obs,float scale)
{

	bool DEBUG = false;
	Map map(obs_map, end , obs, scale);                          //Object of Map class

	veh = car;
	target = end;                         
	
	GUI display(1000, 1000);
    display.draw_obstacles(obs_map, scale);
    display.draw_car(start, car, scale);
    display.draw_car(end, car, scale);
	// display.show(0);

	// Djikstra
	clock_t time_begin= clock();
	h_obj.Dijkstra(map,end);
	clock_t time_end= clock();
	cout<<"Time: Dijkstra= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;
	
	H=new double*[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		H[i]=new double[map.VISY];
	    for(int j=0;j<map.VISY;j++)
			H[i][j]=h_obj.h_vals[i][j].dis;
	}
	// cout<<H[end.gx-100][end.gy]<<" "<<H[end.gx][end.gy-100]<<" "<<H[end.gx-100][end.gy-100]<<endl;
	// exit(0);
	// Array of states
	State*** visited_state=new State**[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		visited_state[i]=new State*[map.VISY];
		for(int j=0;j<map.VISY;j++)
			visited_state[i][j]=new State[map.MAP_THETA];
	}

	// To mark the visited states MAPX, MAPY and MAP_THETA are to be imported from the Map class
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

	priority_queue <State, vector<State>, Planner> pq;
	pq.push(start);

	double checkCollisionTime=0;
	double nextStatesTime=0;

	while(!pq.empty())
	{
		State current=pq.top();

		if(DEBUG)
			cout<<"current state "<<current.x<<" "<<current.y<<" "<<current.theta<<endl;	

		pq.pop();
		int grid_theta=((int)(current.theta*map.MAP_THETA/(2*PI)))%map.MAP_THETA; //grid_theta varies from 0-71 

		if( visited[(int)current.x][(int)current.y][grid_theta] )
			continue;

		visited[(int)current.x][(int)current.y][grid_theta] = true;
		visited_state[(int)current.x][(int)current.y][grid_theta] = current;

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
				// it->cost2d = current.cost2d+1;
				
				if(DEBUG)
					display.draw_tree(current, nextS, scale);
				
				pq.push(*it);
			}
			else
				time_end=clock();

			checkCollisionTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
		}
		if(DEBUG)
        	display.show(1);
	}
	cout<<"Goal cannot be reached"<<endl;
	exit(0);
}



