#ifndef STATE_HPP
#define STATE_HPP

class State{

public:
	float x;
    float y;
	float theta;

	float cost2d;
	State* parent;

	State(float X,float Y,float THETA)
	{
		x=X;
		y=Y;
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

