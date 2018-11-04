#include "../include/Planner.hpp"
// #include "../include/GUI.hpp"

double** H;
float scale_up;
bool Planner::operator()(State a,State b)
{
	return (a.cost2d+H[a.gx][a.gy]/scale_up > b.cost2d+H[b.gx][b.gy]/scale_up);
}

double dis (State a,State* b)
{
	return (sqrt((b->gx-a.gx)*(b->gx-a.gx)+(b->gy-a.gy)*(b->gy-a.gy)));
}

vector<State> Planner::plan(State start, State end, bool** obs_map, Vehicle car,vector<vector<Point> > obs,float scale)
{

	scale_up = scale;
	Map map(obs_map, end , obs, scale);                          //object of Map class
	clock_t time_begin= clock();
	map.initCollisionChecker();
	clock_t time_end= clock();
	cout<<"Time: InitCollisionChecker= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	time_begin= clock();
	h_obj.Dijkstra(map,end);
	time_end= clock();
	cout<<"Time: Dijkstra= "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	// time_begin= clock();
	// h_obj.Dubins_read("Dubins.txt");
	// time_end= clock();
	// cout<<"Time: Dubins Cost Stored = "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

	
	H=new double*[map.MAPX];
	for(int i=0;i<map.MAPX;i++)
	{
		H[i]=new double[map.MAPY];
	    for (int j=0;j<map.MAPY;j++)
			H[i][j]=h_obj.h_vals[i*DX/map.MAPX][j*DY/map.MAPY].dis;
	}
	

	State*** visited_state=new State**[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		visited_state[i]=new State*[map.VISY];
		for(int j=0;j<map.VISY;j++)
			visited_state[i][j]=new State[map.MAP_THETA];
	}

	bool*** visited=new bool**[map.VISX];
	//To mark the visited states MAPX, MAPY and MAP_THETA are to be imported from the Map class
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
		//cout<<"current state "<<current.x<<" "<<current.y<<" "<<current.theta<<endl;	
		pq.pop();
		int grid_theta=((int)(current.theta*map.MAP_THETA/(2*PI)))%72; //grid_theta varies from 0-71 
		if( visited[(int)current.x][(int)current.y][grid_theta] )
			continue;

		visited[(int)current.x][(int)current.y][grid_theta] = true;
		visited_state[(int)current.x][(int)current.y][grid_theta] = current;

		if(map.isReached(current))//checks if it has reached the goal
		{
			cout<<"Time :CollisionChecker= "<<checkCollisionTime<<endl;
			cout<<"Time :nextStates= "<<nextStatesTime<<endl;
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
		vector<State> next=car.nextStates(&current);
		time_end=clock();
		nextStatesTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
	//	cout<<"Next state of "<<current.x<<" "<<current.y<<" "<<current.theta<<endl;
		for(vector<State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State nextS = *it;
			int next_theta=((int)(nextS.theta*180/(PI*5)))%72;
			
			if( visited[(int)nextS.x][(int)nextS.y][next_theta] )
				continue;
	//		cout<<"   "<<nextS.x<<" "<<nextS.y<<" "<<nextS.theta<<endl;	
			//cout<<"check collision "<<map.checkCollision(nextS)<<endl;
			time_begin=clock();
			if( !map.checkCollision(nextS) )
			{
				time_end=clock();
				it->parent = &(visited_state[(int)current.x][(int)current.y][grid_theta]);
				it->cost2d = current.cost2d+1;
				pq.push(*it);
			}
			else
			{
				//cout<<"value of map check "<<map.checkCollisionSat(nextS)<<endl;
				time_end=clock();
				// if(nextS.x > 10 && nextS.y >50)
	//			cout<<"collided at "<<nextS.x<<" "<<nextS.y<<" "<<nextS.theta<<endl;	
			}
			//cout<<" time: "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;
			checkCollisionTime+=double(time_end-time_begin)/CLOCKS_PER_SEC;
		}
	}
	cout<<"Goal cannot be reached"<<endl;
	exit(0);
}



