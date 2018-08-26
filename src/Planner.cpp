#include "../include/Planner.hpp"
double** H;
bool Planner::operator()(State a,State b)
{
	// cout<<"X "<<a.gx<<" Y "<<a.gy<<" Cost "<<H[a.gx][a.gy]<<endl;
	// cout<<"X "<<b.gx<<" Y "<<b.gy<<" Cost "<<H[b.gx][b.gy]<<endl;
	return (a.cost2d+H[a.gx][a.gy]/10 < b.cost2d+H[b.gx][b.gy]/10);
}

double dis (State a,State* b)
{
	return (sqrt((b->gx-a.gx)*(b->gx-a.gx)+(b->gy-a.gy)*(b->gy-a.gy)));
}

vector<State> Planner::plan(State start, State end, bool** obs_map, Vehicle car)
{

	//  State test_state(50, 50, 3.14159265359);
	//  while(true)
	// {
	// 	vector<State> next_states = car.nextStates(&test_state);
	// 	test_state = next_states[0];
	// 	cout<<"Left: "<<test_state.x<<","<<test_state.y<<","<<test_state.theta*180/3.14159265359<<endl;
	// 	test_state = next_states[1];
	// 	cout<<"Right: "<<test_state.x<<","<<test_state.y<<","<<test_state.theta*180/3.14159265359<<endl;

	// 	int t;
	// 	cin>>t;
	// }

	Map map(obs_map,end);//object of Map class
	map.initCollisionChecker();
	h_obj.Dijkstra(map,end);
	// int DX=1000,DY=1000;  //Please make them class member variables
	H=new double*[DX];
	for(int i=0;i<DX;i++)
	{
		
		H[i]=new double[DY];
		
	    for (int j=0;j<DY;j++)
			{
				
				H[i][j]=h_obj.h_vals[i][j].dis;
			}
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
	Mat img=imread("../maps/map.jpg",1);

	while(!pq.empty())
	{
		//cout<<"Inside Queue"<<endl;
		State current=pq.top();
		pq.pop();
		int grid_theta=((int)(current.theta*180/(PI*5)))%72; //grid_theta varies from 0-71 
		if( visited[(int)current.x][(int)current.y][grid_theta] )
		{
			//cout<<"Already Visited Queue"<<endl;
			continue;
		}

		visited[(int)current.x][(int)current.y][grid_theta]=true;//current.theta has to be changed later on
		if(map.isReached(current))//checks if it has reached the goal
		{
			cout<<"REACHED!"<<endl;
			
			State* temp=&current;
			while( temp!=NULL )
			{
				// img.at<Vec3b>(temp->gx,temp->gy)[0]=0;
				// img.at<Vec3b>(temp->gx,temp->gy)[1]=0;		
				// img.at<Vec3b>(temp->gx,temp->gy)[2]=255;
				// imshow("a",img);
				// waitKey(1);
				path.push_back(*temp);
				temp=temp->parent;
			}			
			return path;
		}

				img.at<Vec3b>(current.gx,current.gy)[0]=0;
				img.at<Vec3b>(current.gx,current.gy)[1]=0;		
				img.at<Vec3b>(current.gx,current.gy)[2]=255;
				namedWindow("w",WINDOW_NORMAL);
				imshow("w",img);
				waitKey(1);

		vector<State> next=car.nextStates(&current);
		cout<<"Current: X "<<current.x<<" Y "<<current.y<<" Theta "<<grid_theta<<endl;
		for(vector<State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State nextS;
			nextS=*it;
			int next_theta=((int)(nextS.theta*180/(PI*5)))%72;

			
			// int junk;
			// cin>>junk;	
			
			if( visited[(int)nextS.x][(int)nextS.y][next_theta] )
			{
				//cout<<"Already Visited"<<endl;
				continue;
			}
			if( !map.checkCollision(nextS) )
			{
				// cout<<"\nNot Colliding"<<endl;

				nextS.cost2d=nextS.parent->cost2d+1;//dis(nextS,nextS.parent);
				cout<<"Next to push: x "<<nextS.x<<" y "<<nextS.y<<" theta "<<(nextS.theta*180/PI)<<endl;
				cout<<"Parent: "<<nextS.parent->x<<","<<nextS.parent->y<<","<<(nextS.parent->theta*180/PI)<<endl;
				// cout<<"Cost "<<nextS.cost2d<<endl;
				cout<<"Cost: "<<nextS.cost2d+H[nextS.gx][nextS.gy]<<endl;
				pq.push(nextS);
				// char ch;
				// cin>>ch;

			}
			
			// int jun;
			// cin>>jun;
		
		}
		// int t;
		// cin>>t;
	}
	cout<<"Goal cannot be reached"<<endl;
	exit(0);
}


