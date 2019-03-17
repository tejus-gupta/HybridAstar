#include "../include/GUI.hpp"

GUI::GUI(int rows, int cols, float scale)
{
	this->rows = rows * scale;
	this->cols = cols * scale;
	this->scale = scale;
	display=Mat(cv::Size(this->rows, this->cols), CV_8UC3, Scalar(220,220,220));
}

void GUI::draw_obstacles(vector< vector<Point> > polygon)
{

	// Reversing the points as OpenCV has different frame for points
	for(int i=0; i < polygon.size(); i++)
		for(int j=0; j < polygon[i].size(); j++)
		{
			swap(polygon[i][j].x, polygon[i][j].y);
			polygon[i][j].x *= scale, polygon[i][j].y *= scale;
		}

	for(int i=0;i<polygon.size();i++)
	{
		for(int j=0;j<polygon[i].size()-1;j++)
			cv::line(display, polygon[i][j], polygon[i][j+1], Scalar(0, 0, 255), 1);

		cv::line(display, polygon[i][0], polygon[i][polygon[i].size()-1], Scalar(0, 0, 255), 1);
	}
	
	return;
}


void GUI::draw_car(State state, Vehicle car)
{
	RotatedRect rotRect = RotatedRect(Point2f(state.y*scale, state.x*scale), Size2f(car.BOT_W*scale, car.BOT_L*scale), 180-state.theta*180/M_PI);
	Point2f vert[4];
	rotRect.points(vert);
	for(int i=0;i<4;i++)
		line(display, vert[i], vert[(i+1)%4], Scalar(200, 0, 0));
	
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


