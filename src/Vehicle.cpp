#include "../include/Vehicle.hpp"

vector <State> Vehicle::nextStates(State* n)
{
	State t;
	vector<State> next;
	
	//alpha=steering angle, beta=turning angle, r=turning radius, d=distanced travelled
	float alpha, beta, r, d = 1; 

	for(alpha = -BOT_MAX_ALPHA; alpha <= BOT_MAX_ALPHA+0.01; alpha += BOT_MAX_ALPHA/2)
	{
		beta = abs(d*tan(alpha*M_PI/180)/BOT_L);
		if(abs(beta) > 0.001)
		{
			r = abs(BOT_L/tan(alpha*M_PI/180));
			if(alpha<0)
			{
				t.x = n->x + sin(n->theta)*r - sin(n->theta-beta)*r;
				t.y = n->y - cos(n->theta)*r + cos(n->theta-beta)*r;
				t.theta = fmod(n->theta+beta,2*M_PI);
			}
			else
			{
				t.x = n->x - sin(n->theta)*r + sin(n->theta+beta)*r;
				t.y = n->y + cos(n->theta)*r - cos(n->theta+beta)*r;
				t.theta = fmod(n->theta-beta,2*M_PI);
			}
		
			if(t.theta < 0)
				t.theta += 2*M_PI;
		}
		else
		{
			// if turning radius is very small we assume that the back tire has moved straight
			t.x = n->x + d*cos(n->theta); 
			t.y = n->y + d*sin(n->theta);
			t.theta = n->theta;
			t.g =  n->g + d;
		}

		next.push_back(t);
	}

	return next;		
}
