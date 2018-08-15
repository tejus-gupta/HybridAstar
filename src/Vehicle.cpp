#include "../include/Vehicle.hpp"


 vector<State> Vehicle::nextStates(State n)//vector<Vehicle::State>
{
	
	vector<State> next;
	State t;
	float alpha,beta,r,d=10; //alpha=steering angle, beta = turning angle, r=turning radius, d= distanced travelled

	for(alpha=-BOT_MAX_ALPHA; alpha<=BOT_MAX_ALPHA; alpha+=BOT_MAX_ALPHA/2)
	{
		beta= (d/BOT_L)*tan(alpha*PI/180);

		if(abs(beta)>0.001)
		{
			r=d/beta;
			t.x=n.x - sin(n.theta)*r + sin(n.theta+beta)*r;
			t.y=n.y + cos(n.theta)*r - cos(n.theta+beta)*r;
			t.theta=fmod(n.theta+beta,2*PI);
		}
		else
		{
			t.x=n.x + d*cos(n.theta); // if turning radius is very small we assume that the back tire has moved straight
			t.y=n.y + d*sin(n.theta);
			t.theta=n.theta;
		}
		t.steer_angle=alpha;
		t.parent=&n;
		next.push_back(t);

		//printf("%f,%f,%f\n",t.x,t.y,t.theta);

		return next;		

	}

}