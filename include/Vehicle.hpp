#include <stdio.h>
#include <iostream>
#include "bits/stdc++.h"
//#include "ros/ros.h"


#define PI 3.14159265359

using namespace std;

class Vehicle{

public:

	int CAR_L;
	int CAR_W;
	int CAR_MAX_ALPHA;

typedef struct a{

	float x;
    float y;
	float theta;

	float cost2d;
	float cost3d;

	float velocity;
	float steer_angle;

	struct a* parent;
	struct a* next;

}State;

vector <State> nextStates(State);
};