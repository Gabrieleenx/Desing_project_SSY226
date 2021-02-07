/*
Written by: Gabriel Arslan Waltersson, 2021, for the popoise project. 
*/

#include <poll.h>
#include <thread>
#include <iostream>
#include <cstring>
#include <string>
#include <sys/stat.h> // checking if file/dir exist
#include <fstream>
#include <string>
#include <math.h>


#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"


// ---------- mutex for shared memory ------------------

std::mutex gps_file_mtx;
std::mutex pose_mtx;
std::mutex mission_mtx;
std::mutex map_mtx;
// ------------ Variables -----------------------------
struct Parameter
{
  float gps_update_distance = 1.0;
  int send_recorded_iter = 30;
  int iter = 30;
};

struct Pose_data
{
  float pos_x = 0; // easting in meters
  float pos_y = 0; // northing in meters
  float yaw = 0;
  int isValid = 0;
  int step_since_last_msg = 50;
};

struct Waypoints
{
  const static int max_num_elements = 30; // maximum number of waypoints
  float waypoints_x[max_num_elements]; // positive is to starbord of the boat.
  float waypoints_y[max_num_elements]; // positive is ahead of the boat. 
  int num_waypoints = 0; // how mant waypoints there are
  int isValid = 0; // if the data is valid.
  
};

struct Gps_points
{
  const static int max_num_elements = 1000; // maximum number of gps points
  float easting[max_num_elements]; // positive is east in meters.
  float northing[max_num_elements]; // positive is north in meters. 
  int num_points = 0;
  int is_valid = 0;
};

struct Map_data
{
  const static int max_num_elements = 1000;
  float coord_x[max_num_elements]; // local, positive to right of boat 
  float coord_y[max_num_elements]; // local, positive forward of boat 
  int object_type[max_num_elements]; // 1: target (sent in the order to be followed) 2: obstacle 
  int num_objects = 0; // how mant waypoints there are
  int isValid = 0;
  int is_reciving = 0;
  int last_index = 0;
};


// ----------- log gps data ----------------------------
class Gps_logger{
  private: 
    //gps file 
    //std::fstream gps_file;
    std::fstream gps_file;
    float last_easting = 0;
    float last_northing = 0;
    int exist_last_point = 0;
  public:
    Gps_logger(){
      gps_file.open("gps_data.txt", std::ios::out);
      gps_file.close();
    }

    void read_gps(Gps_points *gps_points){
      gps_file_mtx.lock();
      gps_file.open("gps_data.txt");
      if (gps_file.is_open()){
        std::string line;
        gps_points->num_points = 0;
        while (std::getline(gps_file, line))
        {
          //split line
          std::string str_easting = "";
          std::string str_northing = "";
          int word_num = 0;
          for (auto x : line)
          {
            if (x == ' '){
              word_num += 1;
            }
            else if (word_num == 0){
              str_easting += x;
            }
            else if (word_num == 1){
              str_northing += x;
              gps_points->is_valid=1;
            }
          }

          //convert back to float and add to struct 
          gps_points->easting[gps_points->num_points] = std::stof(str_easting);
          gps_points->northing[gps_points->num_points] = std::stof(str_northing);
          gps_points->num_points += 1;
          if (gps_points->num_points > gps_points->max_num_elements){
            break;
          }

        }
        std::cout << "read file" << std::endl;
        gps_file.close();
      }
      gps_file_mtx.unlock();

    }

    void save_gps(float easting, float northing, float update_distance){

      if (exist_last_point == 0){
        last_easting = easting;
        last_northing = northing;
        exist_last_point = 1;

        gps_file_mtx.lock();
        gps_file.open("gps_data.txt", std::ios::app);
        if (gps_file.is_open()){
          gps_file << std::to_string(easting) << " " << std::to_string(northing) << std::endl;
          gps_file.close();
        }
        gps_file_mtx.unlock();
      }
      else {
        float dx_2 = (float)std::pow(last_easting - easting, 2);
        float dy_2 = (float)std::pow(last_northing - northing, 2);
        float distance = (float)std::sqrt(dx_2+ dy_2);

        if (update_distance < distance){
          gps_file_mtx.lock();
          gps_file.open("gps_data.txt", std::ios::app);
          if (gps_file.is_open()){
            gps_file << std::to_string(easting) << " " << std::to_string(northing) << std::endl;
            gps_file.close();
          }
          gps_file_mtx.unlock();

          last_easting = easting;
          last_northing = northing;
          std::cout << "save gps" << std::endl;
        }        
      }
    }
    
    void delete_gps(){
      gps_file_mtx.lock();
        //delete all contents of txt file. 
        gps_file.open("gps_data.txt", std::ios::out | std::ios::trunc);
        gps_file.close();
      gps_file_mtx.unlock();
      exist_last_point = 0;
    }

};

// --------------- gps to local waypoints ----------------------
void gps_to_waypoints(Gps_points *gps_points, Waypoints  *waypoints, Pose_data *pose){
  // find gps point closest and suse as starting index
  pose_mtx.lock();
  float smallest_distance = 10000000.0; //well if your off more then this then you have bigger problems...
  int smallest_idx = 0;
  for (int i = 0; i < gps_points->num_points; i++){
    float dx_2 = (float)std::pow(pose->pos_x - gps_points->easting[i], 2);
    float dy_2 = (float)std::pow(pose->pos_y - gps_points->northing[i], 2);
    float distance = (float)std::sqrt(dx_2+ dy_2);
    if(distance < smallest_distance){
      smallest_distance = distance;
      smallest_idx = i;
    }
  }
  // take out 30 from that idx or as many as there are and turn to local coord
  int end_idx = ((gps_points->num_points-smallest_idx)<waypoints->max_num_elements) ? 
                    gps_points->num_points:(smallest_idx+waypoints->max_num_elements);
  
  float rot_m[2][2]; // rotation matrix
  rot_m[0][0] = cos(-pose->yaw);
  rot_m[0][1] = -sin(-pose->yaw);
  rot_m[1][0] = sin(-pose->yaw);
  rot_m[1][1] = cos(-pose->yaw);

  int j = 0;
  // turn into local coord
  for (int i = smallest_idx; i < end_idx; i++){
    float east_T = gps_points->easting[i] - pose->pos_x;
    float north_T = gps_points->northing[i] - pose->pos_y;
    
    waypoints->waypoints_x[j] = rot_m[0][0] * east_T + rot_m[0][1] * north_T;
    waypoints->waypoints_y[j] = rot_m[1][0] * east_T + rot_m[1][1] * north_T;
    waypoints->isValid = 1;
    j += 1;
    waypoints->num_waypoints = j;
  }
  pose_mtx.unlock();
}

// ----------- perception to waypoints ------------------------

void perception_to_waypoints(Map_data *map_data,  Waypoints  *waypoints){
  int j = 0;
  for (int i = 0; i < map_data->num_objects; i++){
    if (map_data->object_type[i] == 1){
      waypoints->waypoints_x[j] = map_data->coord_x[i];
      waypoints->waypoints_y[j] = map_data->coord_y[i];
      j += 1;
    }
    if (j > waypoints->max_num_elements){
      break;
    }
  }
  waypoints->isValid = 1;
  waypoints->num_waypoints = j;
  map_data->isValid = 0;
}

// ----------- main loop --------------------
int32_t main(int32_t argc, char **argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  // if not att parameters are available 
  if (0 == commandlineArguments.count("cid") ||
      0 == commandlineArguments.count("verbose") ||
      0 == commandlineArguments.count("updata_dist")) {
    std::cerr << argv[0] << " boat Controller " << std::endl;
    std::cerr << "Example: " << argv[0] << " docker run --rm -ti --init --net=host  pathplanner:latest --cid='112' --verbose  --updata_dist=1" << std::endl;
    return 1;
  } 
  else {
    // Setup
    int32_t VERBOSE{commandlineArguments.count("verbose") != 0};
    if (VERBOSE) {
      VERBOSE = std::stoi(commandlineArguments["verbose"]);

  }
    
    // set up variables
    Waypoints  *waypoints;
    Waypoints waypoints_obj;
    waypoints = & waypoints_obj;

    Gps_points *gps_points;
    Gps_points gps_points_obj;
    gps_points = &gps_points_obj;

    Pose_data *pose_data;
    Pose_data pose_data_obj;
    pose_data = &pose_data_obj;

    Parameter *parameter;
    Parameter parameter_obj;
    parameter = &parameter_obj;

    Map_data *map_data;
    Map_data map_data_obj;
    map_data = &map_data_obj;

    const float FREQ = 100;
    parameter->gps_update_distance = std::stof(commandlineArguments["updata_dist"]);

    Gps_logger gps_logger;
    
    int pathplanner_mission = 0;
    
    opendlv::proxy::Waypoints waypoints_message_send;
    // recive pose and mission 
    auto recive_pose_data{[pose_data ](cluon::data::Envelope &&envelope)
    {
      opendlv::proxy::Pose const pose_data_message = cluon::extractMessage<opendlv::proxy::Pose>(std::move(envelope));
      pose_mtx.lock();

      pose_data->pos_x = (float)pose_data_message.easting();
      pose_data->pos_y = (float)pose_data_message.northing(); 
      pose_data->yaw = (float)pose_data_message.yaw();
      pose_data->step_since_last_msg = 0;

      
      pose_mtx.unlock();
    }};

    auto recive_mission_data{[&pathplanner_mission](cluon::data::Envelope &&envelope)
    {
      opendlv::proxy::Mission const mission_data_message = cluon::extractMessage<opendlv::proxy::Mission>(std::move(envelope));
      mission_mtx.lock();
      pathplanner_mission = mission_data_message.mission_type_pathplanner();
      mission_mtx.unlock();

    }};

    auto recive_map{[map_data](cluon::data::Envelope &&envelope)
    {
      opendlv::perception::Map const map_message = cluon::extractMessage<opendlv::perception::Map>(std::move(envelope));
      
      int idx = map_message.list_index();
      // if first ellement 
      map_mtx.lock();
      if (idx == 0)
      { 
        map_data->is_reciving = 1;
      }

      if (idx >= map_data->max_num_elements && map_data->is_reciving == 1)
      {
        map_data->isValid = 1;
        map_data->is_reciving = 0;
        map_data->num_objects = idx;
      }

      if (map_data->is_reciving == 1)
      { 
        // if message has been skipped or lost.  
        if(idx != map_data->last_index + 1 && idx != 0)
        {
          map_data->isValid = 0;
          map_data->is_reciving = 0;
        }

        map_data->coord_x[idx] = map_message.coord_x();
        map_data->coord_y[idx] = map_message.coord_y();
        map_data->object_type[idx] = map_message.object_type();
        map_data->last_index = idx;

        if (map_data->is_reciving == 1){ 
          if (map_message.is_last_message() == 1){
            map_data->isValid = 1;
            map_data->is_reciving = 0;
            map_data->num_objects = idx+1; 
          }
        }
      }
      map_mtx.unlock();
    }};

    // cid is imprtant. i.e. cid must match sender. 
    // create opendlv object. 
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    // tiggers function to be called when message arrives. 
    od4.dataTrigger(opendlv::proxy::Pose::ID(), recive_pose_data);
    od4.dataTrigger(opendlv::proxy::Mission::ID(), recive_mission_data);
    od4.dataTrigger(opendlv::perception::Map::ID(), recive_map);

    // function to do something at a frequenzy, aslo to keep program running

    auto main_logic{[&waypoints_message_send, waypoints, pose_data, map_data, &gps_logger, gps_points, parameter, &pathplanner_mission, &VERBOSE, &od4]() -> bool
    { 
      mission_mtx.lock();
      int i = pathplanner_mission;
      mission_mtx.unlock();

      switch (i)
      {
      case 0: // no mission
        break;
      case 1: // record gps waypoints
        pose_mtx.lock();
        gps_logger.save_gps(pose_data->pos_x, pose_data->pos_y, parameter->gps_update_distance);
        pose_mtx.unlock();
        break;    
      case 2: // replay/follow recored gps waypoints, but only update every 3 sec
        if (parameter->send_recorded_iter <= parameter->iter){
          gps_logger.read_gps(gps_points);
          gps_to_waypoints(gps_points, waypoints, pose_data);
          //send waypoints
          for (int j=0; j < waypoints->num_waypoints; j++)
          {
            waypoints_message_send.list_index(j);
            waypoints_message_send.coord_x(waypoints->waypoints_x[j]);
            waypoints_message_send.coord_y(waypoints->waypoints_y[j]);
            if (VERBOSE) {
              //std::cout << waypoints->waypoints_x[j] << " x y " << waypoints->waypoints_y[j] << std::endl; 
              std::stringstream buffer;
              waypoints_message_send.accept([](uint32_t, const std::string &, const std::string &) {},
                        [&buffer](uint32_t, std::string &&, std::string &&n, auto v) { buffer << n << " = " << v << '\n'; },
                        []() {});
              std::cout << buffer.str() << std::endl;
            }
            if (j == waypoints->num_waypoints-1){
              waypoints_message_send.is_last_message(1);
            }
            else{
              waypoints_message_send.is_last_message(0);
            }
            od4.send(waypoints_message_send);
          }
          parameter->iter = 0;
        }
        parameter->iter += 1;
        break;    
      case 3: // delete recorded gps points 
        gps_logger.delete_gps();
        break;    
      case 4: // follow perception map
        /*code*/
        map_mtx.lock();
        if (map_data->isValid == 1){
          perception_to_waypoints(map_data,  waypoints);

          //send waypoints
          for (int j=0; j < waypoints->num_waypoints; j++)
          {
            waypoints_message_send.list_index(j);
            waypoints_message_send.coord_x(waypoints->waypoints_x[j]);
            waypoints_message_send.coord_y(waypoints->waypoints_y[j]);
            if (VERBOSE) {
              //std::cout << waypoints->waypoints_x[j] << " x y " << waypoints->waypoints_y[j] << std::endl; 
              std::stringstream buffer;
              waypoints_message_send.accept([](uint32_t, const std::string &, const std::string &) {},
                        [&buffer](uint32_t, std::string &&, std::string &&n, auto v) { buffer << n << " = " << v << '\n'; },
                        []() {});
              std::cout << buffer.str() << std::endl;
            }
            if (j == waypoints->num_waypoints-1){
              waypoints_message_send.is_last_message(1);
            }
            else{
              waypoints_message_send.is_last_message(0);
            }
            od4.send(waypoints_message_send);
          }

        }
        map_mtx.unlock();
        break;    
      // add more cases for other mission types here. 
      default:
        break;
      }

      return true;
    }};
    
    // call that function a frequency 
    od4.timeTrigger(FREQ, main_logic);
  }
  return 0;
}
