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
		theta=THETA;
		parent=NULL;
	}

	State()
	{
		x=0;
		y=0;
		theta=0;
		parent=NULL;
	}
};
#endif

