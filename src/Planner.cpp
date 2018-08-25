#include "../include/Planner.hpp"

Planner p;
class Compare
{
public:
	bool  operator () (const State& a, const State& b) const
	{
		return (p.h_obj.h_vals[a.gx][a.gy].dis > p.h_obj.h_vals[b.gx][b.gy].dis);
	}
};
vector<State> Planner::plan(State start, State end, bool** obs_map, Vehicle car)
{
	Map map(obs_map,end);//object of Map class
	map.initCollisionChecker();
	p.h_obj.Dijkstra(map,end); 
	bool*** visited=new bool**[map.MAPX];//To mark the visited states. MAXX, MAXY and MAX_THETA are to be imported from the map class
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
	priority_queue <State, vector<State>, Compare> pq;
	pq.push(start);
	cout<<"starting size "<<pq.size()<<endl;
	Mat img=imread("../maps/map.jpg",1);

	while(1)
	{
		State current=pq.top();
		pq.pop();
		int t=current.theta*180/(PI*5);
		//cout<<"\n"<<t<<endl;
		if(visited[current.gx][current.gy][t])
				continue;
		img.at<Vec3b>(current.gx,current.gy)[0]=0;
		img.at<Vec3b>(current.gx,current.gy)[1]=0;		
		img.at<Vec3b>(current.gx,current.gy)[2]=255;
		imshow("a",img);
		waitKey(1);

		visited[current.gx][current.gy][t]=true;//current.theta has to be changed later on
		if(map.isReached(current))//checks if it has reached the goal
		{
			cout<<"Reached"<<endl;
			 vector<State> path;
			 State* temp=&current;
			// while( temp!=NULL )
			// {
			// 	path.push_back(*temp);
			// 	temp=temp->parent;

			// }
			return path;
		} 
		vector <State> next=car.nextStates(&current);
		for(vector <State>::iterator it= next.begin(); it!=next.end();it++)
		{
			cout<<"next state loop"<<endl;
			State s;
			s=*it;
			int t_temp=s.theta*180/(PI*5);
			cout<<"state:"<<s.gx<<","<<s.gy<<","<<s.theta<<","<<t_temp<<endl;
			
			
			if(visited[s.gx][s.gy][t_temp])
				continue;
			if(true)
			{
				// cout<<"NOT COLLIDING"<<endl;
				cout<<"state to push"<<s.gx<<","<<s.gy<<","<<s.theta<<endl;
				//cout<<"@@@@@@@@@@@@"<<p.h_obj.h_vals[s.gx][s.gy].dis<<endl;//","<<p.h_obj.h_vals[b.gx][b.gy].dis<<endl;
				pq.push(s);
				cout<<"pushed"<<endl;
				cout<<"Parent: "<<s.parent->gx<<","<<s.parent->gy<<","<<s.parent->theta<<endl;
				int x;
				cin>>x;
				// //cout<<"size after push "<<pq.size()<<endl;
			}

		}
	}
}