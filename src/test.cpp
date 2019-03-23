#include "../include/Planner.hpp"
#include "../include/GUI.hpp"
#include <boost/rational.hpp>

int** load_map_from_image(String map_name)
{
    Mat map_image = imread(map_name, 0);
    int size_x = map_image.rows, size_y = map_image.cols;

    int* map_array=new int[size_x*size_y];
    int** map = new int*[size_y];
    for(int i=0;i<size_y;i++)
        map[i] = map_array + size_x*i;
    
    for(int i=0;i<size_x;i++)
        for(int j=0;j<size_y;j++)
            map[i][j] = (map_image.at<uchar>(i, j) == 0);
    
    return map;
}

void deallocate_map(int** map)
{
    delete[] map[0];
    delete[] map;
    return;
}

int main(int argc,char **argv)
{
    int** map = load_map_from_image("/home/tejus/catkin_ws/src/HybridAstar/maps/map2.png");
    int map_x = 100;
    int map_y = 100;
    float map_grid_resolution = 0.5;
    float planner_grid_resolution = 1;

    State start(20, 20, 0);
    State destination(70, 85, 0);

    GUI display(100, 100, 5);
    Vehicle car;

    display.draw_car(start, car);
    display.draw_car(destination, car);
    display.draw_obstacles(map, map_grid_resolution);
    display.show(0);

    Planner astar(map_x, map_y, map_grid_resolution, planner_grid_resolution);

    const clock_t begin_time = clock();
    vector<State> path = astar.plan(start, destination, car, map, display);
    std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC;

    for(vector<State>::iterator state_ptr = path.begin(); state_ptr != path.end(); state_ptr++)
        display.draw_car(*state_ptr, car);
    display.show(0);

    astar.path.clear();
    deallocate_map(map);
    
    return 0;
}