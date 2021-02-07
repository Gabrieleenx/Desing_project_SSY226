#pragma once
/*
Written by: Gabriel Arslan Waltersson, 2020, for the popoise project. 
*/

// updates the offset for waypoints whn new posedata arrives
void uppdate_local_offset(Pose_data* pose_data, Waypoints* waypoints){
  float delta_yaw = pose_data->yaw - waypoints->init_heading;
  float delta_x = pose_data->pos_x - waypoints->init_pos_x;
  float delta_y = pose_data->pos_y - waypoints->init_pos_y;
  waypoints->offset_yaw = -delta_yaw; 
  float rot_m[2][2]; // rotation matrix
  rot_m[0][0] = cos(-waypoints->init_heading);
  rot_m[0][1] = -sin(-waypoints->init_heading);
  rot_m[1][0] = sin(-waypoints->init_heading);
  rot_m[1][1] = cos(-waypoints->init_heading);


  waypoints->offset_pos_x = -(rot_m[0][0]* delta_x + rot_m[0][1]*delta_y);
  waypoints->offset_pos_y = -(rot_m[1][0]* delta_x + rot_m[1][1]*delta_y);
}

// calculate waypoints in boat frame with current offsets 
void update_waypoints(Waypoints* waypoints){

  float rot_m[2][2]; // rotation matrix
  rot_m[0][0] = cos(waypoints->offset_yaw);
  rot_m[0][1] = -sin(waypoints->offset_yaw);
  rot_m[1][0] = sin(waypoints->offset_yaw);
  rot_m[1][1] = cos(waypoints->offset_yaw);

  for(int i = 0; i < waypoints->num_waypoints; i++){
    // translate
    waypoints->waypoints_x[i] =  waypoints->waypoints_x_init[i]+ waypoints->offset_pos_x;
    waypoints->waypoints_y[i] =  waypoints->waypoints_y_init[i]+ waypoints->offset_pos_y;
    // rotate
    float temp_x = rot_m[0][0] * waypoints->waypoints_x[i] + rot_m[0][1] * waypoints->waypoints_y[i];
    float temp_y = rot_m[1][0] * waypoints->waypoints_x[i] + rot_m[1][1] * waypoints->waypoints_y[i];
    waypoints->waypoints_x[i] = temp_x;
    waypoints->waypoints_y[i] = temp_y;
  }
}


void calc_target(Target* target, Waypoints* waypoints, Parameters* parameters)
{ 
  // if there is only 1 waypoint 
  if (waypoints->num_waypoints == 1){
    target->local_pos_x = waypoints->waypoints_x[0];
    target->local_pos_y =  waypoints->waypoints_y[0];
  }
  else  
  { 
    float distance = 0;
    float shortest_distance = 100000; // start with something big
    int shortest_index = 0;
    float coord_x = 0;
    float coord_y = 0;
    float short_coord_x = 0;
    float short_coord_y = 0;
    // find closest point on path .
    for (int i = 0; i < waypoints->num_waypoints - 1; i++){
      float A[2]; // linesegment start point, store in (x,y)
      float B[2]; // linesegment end point,
      A[0] = waypoints->waypoints_x[i];
      A[1] = waypoints->waypoints_y[i];
      B[0] = waypoints->waypoints_x[i+1];
      B[1] = waypoints->waypoints_y[i+1];

      float M[2];
      M[0] = B[0] - A[0];
      M[1] = B[1] - A[1];

      float t = (-A[0] * M[0] - A[1] * M[1])/((float)pow(M[0],2) + (float)pow(M[1],2));
      if (t<0){
        distance = sqrt((float)pow(A[0],2) + (float)pow(A[1],2));
        coord_x = A[0];
        coord_y = A[1];
      }
      else if (t>1){
        distance = sqrt((float)pow(B[0],2) + (float)pow(B[1],2));
        coord_x = B[0];
        coord_y = B[1];
      }
      else
      {
        float D[2];
        D[0] = A[0] + t*M[0];
        D[1] = A[1] + t*M[1];
        distance = sqrt((float)pow(D[0],2) + (float)pow(D[1],2));
        coord_x = D[0];
        coord_y = D[1];
      }

      if (distance < shortest_distance){
        shortest_distance = distance;
        shortest_index = i;
        short_coord_x = coord_x;
        short_coord_y = coord_y;
      }
    }
    
    // find point target_distance ahead of closes point. 
    float rem_segment_length = 0;
    int nxt_index = shortest_index;
    float dist_left = parameters->target_distance;
    float A_new[2]{0};
    float B_new[2]{0}; 

    while (dist_left > rem_segment_length){
      nxt_index += 1;
      // if it is the last waypoint
      if(nxt_index >= waypoints->num_waypoints){
        break;
      }      
      dist_left -= rem_segment_length;
      rem_segment_length = sqrt((float)pow((short_coord_x - waypoints->waypoints_x[nxt_index]),2) + 
                                 (float)pow((short_coord_y- waypoints->waypoints_y[nxt_index]),2));
      A_new[0] = short_coord_x;
      A_new[1] = short_coord_y;
      B_new[0] = waypoints->waypoints_x[nxt_index];
      B_new[1] = waypoints->waypoints_y[nxt_index];
      short_coord_x = B_new[0];
      short_coord_y = B_new[1];
    }

    // get new target position 
    float t_new = dist_left/sqrt((float)pow((A_new[0] - B_new[0]),2) +(float) pow((A_new[1]- B_new[1]),2));
    if (t_new>1){
      target->local_pos_x = B_new[0];
      target->local_pos_y = B_new[1];
    }
    else
    {
      target->local_pos_x = A_new[0] + t_new*(B_new[0] - A_new[0]);
      target->local_pos_y = A_new[1] + t_new*(B_new[1] - A_new[1]);
    }
  }
  // angle to target
  target->angle_from_y_axis = atan2f(target->local_pos_y, target->local_pos_x) - (float)M_PI/2.0f;
  
  if(target->angle_from_y_axis < -M_PI){
    target->angle_from_y_axis = 2*(float)M_PI + target->angle_from_y_axis;
  }
  // distance to target
  target->distance = sqrt((float)pow(target->local_pos_x, 2) + (float)pow(target->local_pos_y, 2));

}

void calc_control_signals(opendlv::proxy::ControlBoat *controlBoat_msg, Target *target, Parameters *parameters, Pose_data *pose_data){
  // rudder control 
  float error = target->angle_from_y_axis * 180/((float)M_PI); // in degrees 
  parameters->rudder_error_integral += error;
  // PID controller
  float rudder_angle  = parameters->p_rudders * error + 
                      parameters->i_rudders * parameters->rudder_error_integral +
                      parameters->d_rudders * (error - parameters->rudder_last_error); 
  parameters->rudder_last_error = error;

  // max and min rudder actuation 
  if (rudder_angle > parameters->max_rudder_angle){
    rudder_angle = parameters->max_rudder_angle;
  }
  else if(rudder_angle < -parameters->max_rudder_angle){
    rudder_angle = -parameters->max_rudder_angle;
  }
  controlBoat_msg->rudderSB(rudder_angle);
  controlBoat_msg->rudderPS(rudder_angle);

  // Not needed for bow thruster in this mission
  controlBoat_msg->tunnelThruster(0);

  // thrust controll 

  error = target->target_speed - pose_data->speed;
  parameters->thruster_error_integral += error;
  // PID controller
  float thrust = 100*(parameters->p_thrusters * error + 
          parameters->i_thrusters * parameters->thruster_error_integral +
          parameters->d_thrusters * (error - parameters->thruster_last_error));
  parameters->thruster_last_error = error;

  if(thrust > 100){
    thrust = 100;
  }
  else if(thrust < -100){
    thrust = -100;
  }

  controlBoat_msg->motorSB(thrust);
  controlBoat_msg->motorPS(thrust);

}
