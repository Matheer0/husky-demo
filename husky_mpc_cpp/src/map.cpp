#include "husky_mpc_cpp/map.hpp"
#include<cmath>

Obstacle::Obstacle(double x, double y, double radius) : x_(x), y_(y), radius_(radius)
{
}

Map::Map(double x_init, double y_init, double x_target, double y_target, double theta_target, double x_center, double y_center, double offset):
    x_init_(x_init), y_init_(y_init), x_target_(x_target), y_target_(y_target), theta_target_(theta_target), x_center_(x_center), y_center_(y_center), offset_(offset)
{
    y_upper_limit_ = y_center_ + offset_;
    y_lower_limit_ = y_center_ - offset_;

    
	/*
    // Map Parameters for Straight Rooms
    x_lower_limit_ = 0;
	x_upper_limit_ = x_center_;
    */

    // /*
    // Map Parameters for Square Rooms
    x_lower_limit_ = x_center_ - offset_;
	x_upper_limit_ = x_center_ + offset_+5;
    // */
    
}

// ob1 is on top left of ob2
Obstacle Map::compute_obstacle_grouping(Obstacle ob1, Obstacle ob2)
{
    double distance = sqrt(pow((ob2.y_ - ob1.y_), 2) + pow((ob2.x_ - ob1.x_), 2));
    double radius = (distance + ob1.radius_ + ob2.radius_)/2;
    double off_set = (distance/sqrt(2))/2;

    return Obstacle(ob1.x_+off_set, ob1.y_-off_set, radius);
}

		
void Map::generate_circular_obstacles()
{
    double default_radius = 1.0;
    /*
    Obstacle obs(15, 0, default_radius);
    obstacle_list.push_back(obs);
    */

    
    /*
    // ROOM 2
    Obstacle obs_1(6, 4, 1);
    obstacle_list.push_back(obs_1);
    //Obstacle obs_2(18, 8, 1);
    //obstacle_list.push_back(obs_2);
    Obstacle obs_3(15, -1, 1);
    obstacle_list.push_back(obs_3);
    Obstacle obs_4(6, -4, 1);
    obstacle_list.push_back(obs_4);
    Obstacle obs_5(2, -3, 1);
    obstacle_list.push_back(obs_5);
    Obstacle obs_6(3.5, -3, 1);
    obstacle_list.push_back(obs_6);
    Obstacle obs_7(4.5, -4, 1);
    obstacle_list.push_back(obs_7);
    Obstacle obs_8(5.5, -3, 1);
    obstacle_list.push_back(obs_8);
    Obstacle obs_9(1, -2, 1);
    obstacle_list.push_back(obs_9);
    */
    

    /*
    // ROOM test
    Obstacle obs_1(6, 4, 1);
    obstacle_list.push_back(obs_1);
    
    Obstacle obs_3(15, -1, 1);
    obstacle_list.push_back(obs_3);

    
    Obstacle obs_6(3.5, -3, 1);
    obstacle_list.push_back(obs_6);
    
    */

    // ROOM 7
    // target state: (17, 0, 0)
    // initial state: (2, -8, 1.6)
    Obstacle obs_1(11, 8, default_radius);
    obstacle_list.push_back(obs_1);
    //Obstacle obs_2(7, 5, default_radius);
    //obstacle_list.push_back(obs_2);
    
    Obstacle dummy_1(10, 0, default_radius);
    dummy_obstacles.push_back(dummy_1);
    Obstacle dummy_2(11.5, -1.5, default_radius);
    dummy_obstacles.push_back(dummy_2);
    Obstacle dummy_3(10, -1.5, default_radius);
    dummy_obstacles.push_back(dummy_3);
    // grouping of 2 obstacles above
    Obstacle grouping_1 = compute_obstacle_grouping(dummy_1, dummy_2);
    obstacle_list.push_back(grouping_1);
    
    Obstacle dummy_4(14, 6, default_radius);
    dummy_obstacles.push_back(dummy_4);
    Obstacle dummy_5(15.5, 4.5, default_radius);
    dummy_obstacles.push_back(dummy_5);
    Obstacle dummy_6(14, 4.5, default_radius);
    dummy_obstacles.push_back(dummy_6);
    // grouping of 2 obstacles above
    Obstacle grouping_2 = compute_obstacle_grouping(dummy_4, dummy_5);
    obstacle_list.push_back(grouping_2);

    Obstacle dummy_7(1, -2, default_radius);
    dummy_obstacles.push_back(dummy_7);
    Obstacle dummy_8(2, -3, default_radius);
    dummy_obstacles.push_back(dummy_8);
    // grouping of 2 obstacles above
    Obstacle grouping_3 = compute_obstacle_grouping(dummy_7, dummy_8);
    obstacle_list.push_back(grouping_3);
    
    /*
    Obstacle dummy_9(7, -4.5, default_radius);
    dummy_obstacles.push_back(dummy_9);
    Obstacle dummy_10(7, -6, default_radius);
    dummy_obstacles.push_back(dummy_10);
    Obstacle dummy_11(8.5, -6, default_radius);
    dummy_obstacles.push_back(dummy_11);
    // grouping of 2 obstacles above
    Obstacle grouping_4 = compute_obstacle_grouping(dummy_9, dummy_11);
    obstacle_list.push_back(grouping_4);
    */
    

}

	


