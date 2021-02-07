#pragma once
/*
Written by: Gabriel Arslan Waltersson, 2020, for the popoise project. 
*/
// ----------- structs ------------------------
struct Target
{
    float local_pos_x = 0;
    float local_pos_y = 0;
    float angle_from_y_axis = 0; // radians
    float distance = 0;
    float target_speed = 0; // m/s
    int mission_type = 0;
};

struct Pose_data
{
    //float init_pos_x = 0;
    //float init_pos_y = 0;
    float pos_x = 0; // easting in meters
    float pos_y = 0; // northing in meters
    //float last_pos_x = 0;
    //float last_pos_y = 0;
    //float init_yaw = 0; // 0 to north, radians, positive towords west.  
    float yaw = 0;
    //float last_yaw = 0;
    float speed = 0;
    int isValid = 0;
    int step_since_last_msg = 50;
};

struct Waypoints
{
    const static int max_num_elements = 30; // maximum number of waypoints
    float waypoints_x[max_num_elements]; // positive is to starbord of the boat.
    float waypoints_y[max_num_elements]; // positive is ahead of the boat. 
    float waypoints_x_init[max_num_elements]; // positive is to starbord of the boat.
    float waypoints_y_init[max_num_elements]; // positive is ahead of the boat.
    int num_waypoints = 0; // how mant waypoints there are
    int isValid = 0; // if the data is valid.
    int reciving = 0; // if the data is currently streamed in
    int last_index = 0; // the index of the last valid waypoint
    float offset_yaw = 0; // local offsets, i.e. the boat is always in origin, and the waypoints will be shifted. 
    float offset_pos_x = 0; 
    float offset_pos_y = 0; 
    float init_pos_x = 0;
    float init_pos_y = 0;
    float init_heading = 0;
};

struct Parameters
{
  float target_distance = 2.0; 
  float goal_distance = 0.5; 
  float p_rudders = 1;
  float i_rudders = 0;
  float d_rudders = 0;
  float rudder_error_integral = 0;
  float rudder_last_error = 0;
  float max_rudder_angle = 60; // degrees
  float p_thrusters = 1;
  float i_thrusters = 0;
  float d_thrusters = 0;
  float thruster_error_integral = 0;
  float thruster_last_error = 0;
};