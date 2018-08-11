#include "../include/Vehicle.hpp"

vector<Vehicle::State> Vehicle::nextStates(Vehicle::State n)
{
	vector<Vehicle::State> next;
	Vehicle::State t;
	float alpha,beta,R,d; //alpha=steering angle, beta = turning angle, R=turning radius, d= distanced travelled

	for(alpha=-CAR_MAX_ALPHA; alpha<=CAR_MAX_ALPHA; alpha+=CAR_MAX_ALPHA/2)
	{
		beta= (d/CAR_L)*tan(alpha*PI/180);

		if(abs(beta)>0.001)
		{
			R=d/beta;
			t.x=n.x - sin(n.theta*PI/180)*R + sin((n.theta*PI/180)+beta)*R;
			t.y=n.y + cos(n.theta*PI/180)*R - cos((n.theta*PI/180)+beta)*R;
			t.theta=fmod(n.theta+beta,2*PI);
		}
		else
		{
			t.x=n.x + d*cos(n.theta*PI/180); // if turning radius is very small we assume that the back tire has moved straight
			t.y=n.y + d*sin(n.theta*PI/180);
			t.theta=n.theta;
		}
		t.steer_angle=alpha;
		t.parent=&n;
		next.push_back(n);


	}


}

int main()
{
	
}