#ifndef VEH_HPP
#define VEH_HPP

#include <stdio.h>
#include <iostream>
#include "bits/stdc++.h"
//#include "ros/ros.h"

#define CAR_L 60
#define CAR_W 40 
#define CAR_MAX_ALPHA 30
#define MIN_TURN_RADIUS 40
#define PI 3.14159265359

using namespace std;

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

#endif