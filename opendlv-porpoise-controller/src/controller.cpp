/*
Written by: Gabriel Arslan Waltersson, 2020, for the popoise project. 
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

#include "Variables.hpp"
#include "helper_functions.hpp"

// ---------- mutex for shared memory ------------------

std::mutex read_wapoints_mtx;
std::mutex read_pose_mtx;
std::mutex read_target_mtx;

int32_t main(int32_t argc, char **argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  // if not att parameters are available 
  if (0 == commandlineArguments.count("cid") ||
      0 == commandlineArguments.count("verbose") ||
      0 == commandlineArguments.count("p_rudder") ||
      0 == commandlineArguments.count("i_rudder") ||
      0 == commandlineArguments.count("d_rudder") ||
      0 == commandlineArguments.count("p_thruster") ||
      0 == commandlineArguments.count("i_thruster") ||
      0 == commandlineArguments.count("d_thruster") ||
      0 == commandlineArguments.count("target_dist") ||
      0 == commandlineArguments.count("goal_dist") ||
      0 == commandlineArguments.count("freq")) {
    std::cerr << argv[0] << " boat Controller " << std::endl;
    std::cerr << "Example: " << argv[0] << " docker run --rm -ti --init --net=host  controller:latest --cid='112' --verbose --p_rudder=1 --i_rudder=0 --d_rudder=0 --p_thruster=1 --i_thruster=0 --d_thruster=0 --target_dist=2 --goal_dist=1 --freq=50 " << std::endl;
    return 1;
  } else {
    // Setup
    int32_t VERBOSE{commandlineArguments.count("verbose") != 0};
    if (VERBOSE) {
      VERBOSE = std::stoi(commandlineArguments["verbose"]);
    }
    
    // set up variables
    Waypoints  *waypoints;
    Waypoints waypoints_obj;
    waypoints = & waypoints_obj;

    Pose_data *pose_data;
    Pose_data pose_data_obj;
    pose_data = &pose_data_obj;

    Target *target;
    Target target_obj;
    target = & target_obj;
     
    Parameters *parameters;
    Parameters parameters_obj;
    parameters = & parameters_obj;
    const float FREQ = std::stof(commandlineArguments["freq"]);
    parameters->p_rudders = std::stof(commandlineArguments["p_rudder"]);
    parameters->i_rudders = std::stof(commandlineArguments["i_rudder"]);
    parameters->d_rudders = std::stof(commandlineArguments["d_rudder"]);
    parameters->p_thrusters = std::stof(commandlineArguments["p_thruster"]);
    parameters->i_thrusters = std::stof(commandlineArguments["i_thruster"]);
    parameters->d_thrusters = std::stof(commandlineArguments["d_thruster"]);
    parameters->target_distance = std::stof(commandlineArguments["target_dist"]);
    parameters->goal_distance = std::stof(commandlineArguments["goal_dist"]);
    opendlv::proxy::ControlBoat controlBoat_msg;

    // --------- callback functions for when messages arrive. ----------------  

    // recive waypoints that are sent out in a squential order. 
    auto recive_waypoints{[waypoints, pose_data](cluon::data::Envelope &&envelope)
    {
      opendlv::proxy::Waypoints const waypoints_message = cluon::extractMessage<opendlv::proxy::Waypoints>(std::move(envelope));
      
      int idx = waypoints_message.list_index();
      // if first ellement 
      read_wapoints_mtx.lock();
      if (idx == 0)
      { 
        read_pose_mtx.lock();
        waypoints->reciving = 1;
        waypoints->init_pos_x = pose_data->pos_x;
        waypoints->init_pos_y = pose_data->pos_y;
        waypoints->init_heading = pose_data->yaw;
        read_pose_mtx.unlock();
        waypoints->offset_pos_x = 0;
        waypoints->offset_pos_y = 0;
        waypoints->offset_yaw = 0;
      }

      if (idx >= waypoints->max_num_elements && waypoints->reciving == 1)
      {
        waypoints->isValid = 1;
        waypoints->reciving = 0;
        waypoints->num_waypoints = idx;
      }

      if (waypoints->reciving == 1)
      { 
        // if message has been skipped or lost.  
        if(idx != waypoints->last_index + 1 && idx != 0)
        {
          waypoints->isValid = 0;
          waypoints->reciving = 0;
        }

        waypoints->waypoints_x_init[idx] = waypoints_message.coord_x();
        waypoints->waypoints_y_init[idx] = waypoints_message.coord_y();
        waypoints->last_index = idx;
        if (waypoints->reciving == 1){ 
          if (waypoints_message.is_last_message() == 1){
            waypoints->isValid = 1;
            waypoints->reciving = 0;
            waypoints->num_waypoints = idx+1; 
          }
        }
      }
      read_wapoints_mtx.unlock();

    }};

    auto recive_pose_data{[pose_data, waypoints](cluon::data::Envelope &&envelope)
    {
      opendlv::proxy::Pose const pose_data_message = cluon::extractMessage<opendlv::proxy::Pose>(std::move(envelope));
      read_wapoints_mtx.lock();
      read_pose_mtx.lock();

      pose_data->pos_x = (float)pose_data_message.easting();
      pose_data->pos_y = (float)pose_data_message.northing(); 
      pose_data->yaw = (float)pose_data_message.yaw();
      pose_data->speed = (float)pose_data_message.velocity();
      pose_data->isValid = 1;
      pose_data->step_since_last_msg = 0;

      uppdate_local_offset(pose_data, waypoints);
      
      read_pose_mtx.unlock();
      read_wapoints_mtx.unlock();
    }};

    auto recive_mission_data{[target](cluon::data::Envelope &&envelope)
    {
      opendlv::proxy::Mission const mission_data_message = cluon::extractMessage<opendlv::proxy::Mission>(std::move(envelope));
      read_target_mtx.lock();
      target->target_speed = mission_data_message.target_speed();
      target->mission_type = mission_data_message.mission_type_controller();

      read_target_mtx.unlock();
    }};

    // cid is imprtant. i.e. cid must match sender. 
    // create opendlv object. 
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    // tiggers function to be called when message arrives. 
    od4.dataTrigger(opendlv::proxy::Waypoints::ID(), recive_waypoints);
    od4.dataTrigger(opendlv::proxy::Pose::ID(), recive_pose_data);
    od4.dataTrigger(opendlv::proxy::Mission::ID(), recive_mission_data);

    // function to do something at a frequenzy, aslo to keep program running
    
    auto CalcControlsignals{[&controlBoat_msg, pose_data, waypoints, &VERBOSE, &od4, parameters, target]() -> bool
    { 

      read_target_mtx.lock();
      int mission_type = target->mission_type;
      read_target_mtx.unlock();
      read_pose_mtx.lock();
      pose_data->step_since_last_msg += 1;
      int time_out = pose_data->step_since_last_msg;
      read_pose_mtx.unlock();

      // mission 0 is reserved for no actuation 
      if (mission_type == 0 || time_out > 50){          
        controlBoat_msg.motorSB(0);
        controlBoat_msg.motorPS(0);
        controlBoat_msg.rudderSB(0);
        controlBoat_msg.rudderPS(0);
        controlBoat_msg.tunnelThruster(0);
        controlBoat_msg.isValid(1);
      }

      // ----------- follow way points mission ----------------------
      else if (mission_type == 1){
        read_wapoints_mtx.lock();
        int waypoints_valid = waypoints->isValid;
        int reciving = waypoints->reciving;
        read_wapoints_mtx.unlock();
        if(waypoints_valid == 1 && reciving == 0){
          // update way points
          read_wapoints_mtx.lock();
          update_waypoints(waypoints);
          // calc target
          read_target_mtx.lock();
          calc_target(target, waypoints, parameters);
          read_target_mtx.unlock();
          read_wapoints_mtx.unlock();
          // calc control signals.

          read_pose_mtx.lock();
          read_target_mtx.lock();
          calc_control_signals(&controlBoat_msg, target, parameters, pose_data);
          read_target_mtx.unlock();
          read_pose_mtx.unlock();
          // end mission
          read_target_mtx.lock();
          if (target->distance < parameters->goal_distance)
          {
            std::cout << "target reached " << std::endl; 
            target->mission_type = 0; 
          }
          read_target_mtx.unlock();

        }
        else
        {
          controlBoat_msg.motorSB(0);
          controlBoat_msg.motorPS(0);
          controlBoat_msg.rudderSB(0);
          controlBoat_msg.rudderPS(0);
          controlBoat_msg.tunnelThruster(0);
          controlBoat_msg.isValid(1);
        }
      }
      else if (mission_type == 2){
        /* add other mission type here. 
        For example, hold position. 
        */
      }

      // send message
      od4.send(controlBoat_msg);
      // display msg if verbose 
      if (VERBOSE) {
      std::stringstream buffer;
      controlBoat_msg.accept([](uint32_t, const std::string &, const std::string &) {},
                [&buffer](uint32_t, std::string &&, std::string &&n, auto v) { buffer << n << " = " << v << '\n'; },
                []() {});
      std::cout << buffer.str() << std::endl;
      }
      
      return true;
    }};
    
    // call that function a frequency 
    od4.timeTrigger(FREQ, CalcControlsignals);
  }
  return 0;
}
