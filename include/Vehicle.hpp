#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <stdio.h>
#include <iostream>
#include "State.hpp"
#include "bits/stdc++.h"

using namespace std;

class Vehicle{

public:

	float BOT_L=3;
	float BOT_W=2;
	float BOT_MAX_ALPHA=30;
	double min_radius = abs(BOT_L/tan(BOT_MAX_ALPHA*M_PI/180));
	vector<State> nextStates(State*);
};
#endif
