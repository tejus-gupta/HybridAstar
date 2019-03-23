#include "../include/GUI.hpp"

GUI::GUI(int map_x, int map_y, float scale)
{
	rows = map_x * scale;
	cols = map_y * scale;
	this->scale = scale;
	display=Mat(cv::Size(rows, cols), CV_8UC3, Scalar(220,220,220));
}

void GUI::draw_obstacles(int** map, float map_grid_resolution)
{
	float factor = 1/map_grid_resolution/scale;
	for(int i=0; i<rows; i++)
		for(int j=0; j<cols; j++)
		{
			if (map[roundDown(i*factor)][roundDown(j*factor)])
				display.at<Vec3b>(i, j) =  {128, 128, 128};
		}
	
	return;
}

void GUI::draw_car(State state, Vehicle car)
{
	RotatedRect rotRect = RotatedRect(Point2f(state.y*scale, state.x*scale), Size2f(car.BOT_W*scale, car.BOT_L*scale), 3.14159/2-state.theta*180/M_PI);
	Point2f vert[4];
	rotRect.points(vert);
	for(int i=0;i<4;i++)
		line(display, vert[i], vert[(i+1)%4], Scalar(200, 0, 0));
	
	circle(display, Point2f((state.y + car.BOT_W*sin(state.theta)/2)*scale, (state.x + car.BOT_L*cos(state.theta)/2)*scale), 5, Scalar(255, 0, 0));
	
	return;
}

void GUI::draw_dubins( vector<State> Path )
{
	for(vector<State>::iterator next= Path.begin(); next!=Path.end();next++)
		display.at<Vec3b>(next->x*scale,next->y*scale) = { 255, 0, 0 };		

	return ;
}

void GUI::draw_tree(State state, State next)
{
	Point root,child;
	root.x=state.y*scale,root.y=state.x*scale;
	child.x=next.y*scale,child.y=next.x*scale;
	line(display, root, child, Scalar(0,0,255), 1);
	
	return;
}

void GUI::show(int t)
{
	imshow("Display", display);
	waitKey(t);
	return;
}

void GUI::clear()
{
	for(int i=0; i<rows; i++)
		for(int j=0; j<cols; j++)
			display.at<Vec3b>(i, j) =  {220,220,220};

	return;
}


