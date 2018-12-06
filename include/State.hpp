#ifndef STATE_HPP
#define STATE_HPP

class State{

public:
	float x;
    float y;
	float theta;
	
	int gx;
	int gy;

	float cost2d;
	float cost3d;

	float velocity;
	float steer_angle;

	State* parent;
	State* next;

	State(float X,float Y,float THETA)
	{
		x=X;
		y=Y;
		gx=x;
		gy=y;
		theta=THETA;
		parent=NULL;
		cost2d=0;
	}

	State()
	{
		x=0;
		y=0;
		theta=0;
		parent=NULL;
		cost2d=0;
	}
};
#endif

