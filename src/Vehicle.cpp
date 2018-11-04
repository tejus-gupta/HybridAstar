#include "../include/Vehicle.hpp"
vector <State> Vehicle::nextStates(State* n)//vector<Vehicle::State>
{
	vector<State> next;
	State t;
	float alpha,beta,r,d=1; //alpha=steering angle, beta = turning angle, r=turning radius, d= distanced travelled

	for(alpha=-BOT_MAX_ALPHA; alpha<=BOT_MAX_ALPHA+0.01; alpha+=BOT_MAX_ALPHA/2)
	{
		beta=abs(d*tan(alpha*PI/180)/BOT_L);
		if(abs(beta)>0.001)
		{
			r=abs(BOT_L/tan(alpha*PI/180));
			if(alpha<0)
			{
				t.x=n->x + sin(n->theta)*r - sin(n->theta-beta)*r;
				t.y=n->y - cos(n->theta)*r + cos(n->theta-beta)*r;
				t.theta=fmod(n->theta+beta,2*PI);
			}
			else
			{
				t.x=n->x - sin(n->theta)*r + sin(n->theta+beta)*r;
				t.y=n->y + cos(n->theta)*r - cos(n->theta+beta)*r;
				t.theta=fmod(n->theta-beta,2*PI);
			}
		
			if(t.theta<0)
				t.theta+=2*PI;
		}
		else
		{
			t.x=n->x + d*cos(n->theta); // if turning radius is very small we assume that the back tire has moved straight
			t.y=n->y + d*sin(n->theta);
			t.theta=n->theta;
		}
		t.gx=(int)(t.y*5);
		t.gy=(int)(t.x*5);
		t.steer_angle=alpha;
		//t.cost2d=n->cost2d+1;

		if(t.gx>=0&&t.gx<1000&&t.gy>=0&&t.gy<1000)//change upperbound according to the map size
		{	
			next.push_back(t);
		}
	}
	return next;		
}
