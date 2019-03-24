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
    String base_path = "/home/tejus/catkin_ws/src/HybridAstar/maps/";
    String blank_line, map_path;

    int num_tests;
    cin>>num_tests;

    int map_x = 100;
    int map_y = 100;
    float map_grid_resolution = 0.5;
    float planner_grid_resolution = 1;

    float start_x, start_y, start_theta;
    float destination_x, destination_y, destination_theta;

    GUI display(100, 100, 5);
    Vehicle car;

    Planner astar(map_x, map_y, map_grid_resolution, planner_grid_resolution);

    clock_t begin_time, end_time;
    float avg_time = 0;

    for(int i=0;i<num_tests;i++)
    {
        cin>>map_path;
        cin>>start_x>>start_y>>start_theta;
        cin>>destination_x>>destination_y>>destination_theta;
        
        int** map = load_map_from_image(base_path+map_path);

        State start(start_x, start_y, start_theta);
        State destination(destination_x, destination_y, destination_theta);

        display.draw_car(start, car);
        display.draw_car(destination, car);
        display.draw_obstacles(map, map_grid_resolution);
        display.show(0);

        begin_time = clock();
        vector<State> path = astar.plan(start, destination, car, map, display);
        end_time = clock();

        cout<<"map init: "<<astar.map_init_time<<endl;
        cout<<"dijkstra: "<<astar.dijkstra_time<<endl;
        cout<<"planning: "<<(float(end_time-begin_time)/CLOCKS_PER_SEC - astar.map_init_time - astar.dijkstra_time)<<endl;
        cout<<"total: "<<float(end_time-begin_time)/CLOCKS_PER_SEC<<endl<<endl;

        avg_time += float(end_time-begin_time)/CLOCKS_PER_SEC;

        for(vector<State>::iterator state_ptr = path.begin(); state_ptr != path.end(); state_ptr++)
            display.draw_car(*state_ptr, car);
        display.show(0);

        display.clear();
        astar.path.clear();
        deallocate_map(map);

    }
    cout<<endl<<"Average time: "<<avg_time/num_tests<<endl;
    
    return 0;
}