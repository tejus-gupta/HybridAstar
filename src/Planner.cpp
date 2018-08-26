#include "../include/Planner.hpp"
double** H;
bool Planner::operator()(State a,State b)
{
	// cout<<"X "<<a.gx<<" Y "<<a.gy<<" Cost "<<H[a.gx][a.gy]<<endl;
	// cout<<"X "<<b.gx<<" Y "<<b.gy<<" Cost "<<H[b.gx][b.gy]<<endl;
	return (a.cost2d+H[a.gx][a.gy]/10 > b.cost2d+H[b.gx][b.gy]/10);
}

double dis (State a,State* b)
{
	return (sqrt((b->gx-a.gx)*(b->gx-a.gx)+(b->gy-a.gy)*(b->gy-a.gy)));
}

vector<State> Planner::plan(State start, State end, bool** obs_map, Vehicle car)
{
	Map map(obs_map, end);//object of Map class
	map.initCollisionChecker();
	h_obj.Dijkstra(map,end);
	
	H=new double*[DX];
	for(int i=0;i<DX;i++)
	{
		H[i]=new double[DY];
	    for (int j=0;j<DY;j++)
			H[i][j]=h_obj.h_vals[i][j].dis;
	}
	

	State*** visited_state=new State**[100];
	for(int i=0;i<100;i++)
	{
		visited_state[i]=new State*[100];
		for(int j=0;j<100;j++)
			visited_state[i][j]=new State[72];
	}

	bool*** visited=new bool**[map.VISX];
	//To mark the visited states MAPX, MAPY and MAP_THETA are to be imported from the Map class
	for(int i=0;i<map.VISX;i++)
	{
		visited[i]=new bool*[map.VISY];
		for(int j=0;j<map.VISY;j++)
			{
				visited[i][j]=new bool[map.MAP_THETA];
				for(int k=0;k<72;k++)
				{
					visited[i][j][k]=false;
				}
			}
	}
	priority_queue <State, vector<State>, Planner> pq;
	pq.push(start);

	while(!pq.empty())
	{
		State current=pq.top();
		pq.pop();
		int grid_theta=((int)(current.theta*180/(PI*5)))%72; //grid_theta varies from 0-71 
		if( visited[(int)current.x][(int)current.y][grid_theta] )
			continue;

		visited[(int)current.x][(int)current.y][grid_theta] = true;
		visited_state[(int)current.x][(int)current.y][grid_theta] = current;

		if(map.isReached(current))//checks if it has reached the goal
		{
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

		vector<State> next=car.nextStates(&current);
		
		for(vector<State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State nextS = *it;
			int next_theta=((int)(nextS.theta*180/(PI*5)))%72;
			
			if( visited[(int)nextS.x][(int)nextS.y][next_theta] )
				continue;
			
			if( !map.checkCollision(nextS) )
			{
				it->parent = &(visited_state[(int)current.x][(int)current.y][grid_theta]);
				it->cost2d = current.cost2d+1;
				pq.push(*it);
			}
		}
	}
	cout<<"Goal cannot be reached"<<endl;
	exit(0);
}


