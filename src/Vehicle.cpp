#include "../include/Vehicle.hpp"

vector<State> nextStates(State n)
{
	vector<State> next;
	State t;
	float alpha,beta,R;

	for(alpha=-CAR_MAX_ALPHA; alpha<=CAR_MAX_ALPHA; alpha+=CAR_MAX_ALPHA/2)
	{
		beta= (CAR_W/CAR_L)*tan(alpha*PI/180);

		if(abs(beta)>0.001)
		{
			R=CAR_W/beta;
			t.x=n.x - sin(n.theta*PI/180)*R + sin((n.theta+beta)*PI/180)*R;
			t.y=n.y + cos(n.theta*PI/180)*R - cos((n.theta+beta)*PI/180)*R;
			t.theta=fmod(n.theta+beta,2*PI);
		}
		else
		{
			t.x=n.x + CAR_W*cos(n.theta*PI/180);
			t.y=n.y + CAR_W*sin(n.theta*PI/180);
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