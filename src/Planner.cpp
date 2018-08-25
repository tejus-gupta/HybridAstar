#include "../include/Planner.hpp"
double** H;

bool Planner::operator()(State a,State b)
{
	cout<<"X "<<a.gx<<" Y "<<a.gy<<" Cost "<<H[a.gx][a.gy]<<endl;
	cout<<"X "<<b.gx<<" Y "<<b.gy<<" Cost "<<H[b.gx][b.gy]<<endl;
	return (H[a.gx][a.gy] > H[b.gx][b.gy]);
}


vector<State> Planner::plan(State start, State end, bool** obs_map, Vehicle car)
{
	Map map(obs_map,end);//object of Map class
	map.initCollisionChecker();
	h_obj.Dijkstra(map,end);
	// int DX=1000,DY=1000;  //Please make them class member variables
	H=new double*[DX];
	for(int i=0;i<DX;i++)
	{
		H[i]=new double[DY];
	    for (int j=0;j<DY;j++)
			H[i][j]=h_obj.h_vals[i][j].dis;
	}
	bool*** visited=new bool**[map.MAPX];
	//To mark the visited states MAPX, MAPY and MAP_THETA are to be imported from the Map class
	for(int i=0;i<map.MAPX;i++)
	{
		visited[i]=new bool*[map.MAPY];
		for(int j=0;j<map.MAPY;j++)
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
		cout<<"Inside Queue"<<endl;
		State current=pq.top();
		pq.pop();
		int grid_theta=((int)(current.theta*180/(2*PI*5)))%72; //grid_theta varies from 0-71 
		if( visited[current.gx][current.gy][grid_theta] )
		{
			cout<<"Already Visited Queue"<<endl;
			continue;
		}


		cout<<"X "<<current.gx<<" Y "<<current.gy<<" Theta "<<grid_theta<<endl;
		int u;
		cin>>u;


		visited[current.gx][current.gy][grid_theta]=true;//current.theta has to be changed later on
		if(map.isReached(current))//checks if it has reached the goal
		{
			vector<State> path;
			State *temp=&current;
			while( temp!=NULL )
			{
				path.push_back(*temp);
				temp=temp->parent;

			}
			cout<<"Goal reached returning path"<<endl;
			return path;
		} 
		vector <State> next=car.nextStates(current);
		for(vector <State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State next;
			next=*it;
			int next_theta=((int)(next.theta*180/(2*PI*5)))%72;

			cout<<"x "<<next.gx<<" y "<<next.gy<<" theta "<<next_theta<<endl;
			cout<<"Cost "<<h_obj.h_vals[next.gx][next.gy].dis<<endl;
			int junk;
			cin>>junk;	
			
			if( visited[next.gx][next.gy][next_theta] )
			{
				cout<<"Already Visited"<<endl;
				continue;
			}
			if( !map.checkCollision(next) )
			{
				cout<<"Not Colliding"<<endl;
				pq.push(next);
			}
			
			// int jun;
			// cin>>jun;
		
		}
	}
	cout<<"Goal cannot be reached"<<endl;
	exit(0);
}


