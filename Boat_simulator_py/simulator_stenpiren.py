import porpoiseMessageSet_pb2 as porpoise_message_set
import OD4Session
import time
import sys
import getopt
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.transforms as mtransforms
import utm
from datetime import datetime

import numpy as np
import threading

img = mpimg.imread('Test_map.png')

# ---------- Simulation Parameters --------
# boat parameters
delta_T = 0.01 # simlation step time
boat_mass = 5.0 # in kg
boat_forward_thrust = 80 # max in N per proppeler. 
boat_forward_thrust_waterspeed = 10 # the maximum ecjection speed from the proppellers in m/s
boat_dist_between_prop = 0.1 # distance betweem propelers.
boat_bow_thrust = 10 # bow thrust in N
#boat_bow_thrust_waterspeed = 1 # the maximum ecjection speed from the proppellers in m/s
boat_length = 1.2 # in meters
boat_width = 0.3 # in meters
boat_bow_to_pivot = 0.4 # in meters moment arm from bowthruster to hypothectial pivot point
boat_rudder_force_const = 0.02 # force = boat_rudder_force_const * angle(deg) * waterspeed, per rudder
boat_rudder_to_pivot = 0.5 # in meters moment arm from rudders to hypothectial pivot point
boat_forward_friction = 5 # force = boat_forward_friction * forwardSpeed(m/2)**2 
boat_side_friction = 50 # force = boat_side_friction * sideSpeed(m/2)**2
boat_moment_friction = 10 # rotional friction = boat_moment_friction * rotationSpeed(rad/s)**2

# initial boat pose
easting_, northing_, gps_zone_num,gps_zone_letter  = utm.from_latlon(57.705904, 11.956531)

boat_pos_y = northing_ # in meters, northing
boat_pos_x = easting_ # in meters, esting
boat_heading = 0.7*np.pi # headig in radians. 0 north, + gives west, i.e. right hand rule. 
boat_ang_vel = 0 # angular velocity rad/s
boat_speed_y = 0; # in m/s
boat_speed_x = 0; # in m/s

local_pos_x = boat_pos_x
local_pos_y = boat_pos_y


# initial boat control signals

motor_values = np.array([0,0,0,0,0])
target_speed = 5
mission_type_controller = 1 # 0: do nothing, 1: follow waypoints
mission_type_pathplanner = 1 # 0: do nothing, 1: record waypoints, 2:replay waypoints, 3:follow perception map

# noise and disturbances
wind_noise_y_scalar = 0 # amplitude of wind noise profile 
wind_noise_x_scalar = 0
off_wind = 0.5
water_y_stream = 0 # constant stream
water_x_stream = 0
wave_amplitude = 0 # scalar not in units 

time_n = 0
time_w_x = 0
time_w_y = 0

#pose noise
gps_north_std = 0
gps_est_std = 0
heading_std = 0
speed_std = 0

# waypoints 

waypoints_latlong = np.array([[57.705904, 57.704927, 57.704434, 57.703929, 57.703517, 57.70354,\
         57.703677, 57.703861, 57.704428, 57.705504],[11.956531, 11.95437, 11.952953,11.950378,\
        11.947417, 11.945271, 11.943555, 11.941795, 11.94069, 11.939844]])
num_waypoints = 10

waypoints_global = np.zeros((2, num_waypoints))

target_treashold = 20 # meters from final waypoint 

#from lonlat to easting and northing
for i in range(num_waypoints):
    easting_, northing_, gps_zone_num,gps_zone_letter  = utm.from_latlon(waypoints_latlong[0,i], waypoints_latlong[1,i])
    waypoints_global[0,i] = easting_
    waypoints_global[1,i] = northing_

# mutex

control_signal_mtx = threading.Lock()

# 
# moment of inertia... aproximated as rectangle

I_z = (1.0/12.0) * boat_mass * (boat_width**2 + boat_length**2) 

# boat for plotting
y1 = -boat_length/2.0
x1 = -boat_width/2.0
y2 = boat_length/2.5
x2 = -boat_width/2.0
y3 = boat_length/2.0
x3 = 0
y4 = boat_length/2.5
x4 = boat_width/2.0
y5 = -boat_length/2.0
x5 = boat_width/2.0

scale_boat = 100 # visual scale for plotting 
boat_points = np.array([[x1, x2, x3, x4, x5, x1],[y1,y2,y3,y4,y5,y1]])*scale_boat

# --------- Funcitons ----------------
 
def control_signals(msg, senderStamp, timeStamps):
    control_signal_mtx.acquire()
    motor_values[0] = msg.motorSB
    motor_values[1] = msg.motorPS
    motor_values[2] = msg.rudderSB
    motor_values[3] = msg.rudderPS
    motor_values[4] = msg.tunnelThruster
    control_signal_mtx.release()

def force_calc(local_speed_x,local_speed_y,rot_speed):
    control_signal_mtx.acquire()
    thruster_SB = boat_forward_thrust*motor_values[0]/100
    thruster_PS = boat_forward_thrust*motor_values[1]/100
    forward_force = thruster_SB +thruster_PS - np.sign(local_speed_y)*boat_forward_friction * local_speed_y**2

    force_rudder_SB =  -boat_rudder_force_const*motor_values[2] * (local_speed_y + (boat_forward_thrust_waterspeed - local_speed_y)*(boat_forward_thrust_waterspeed * motor_values[0] / 100))
    force_rudder_PS =  -boat_rudder_force_const*motor_values[3] * (local_speed_y + (boat_forward_thrust_waterspeed - local_speed_y)*(boat_forward_thrust_waterspeed * motor_values[1] / 100))
    bow_thrust = boat_bow_thrust*motor_values[4]
    side_force = bow_thrust - force_rudder_SB - force_rudder_PS -  np.sign(local_speed_x)*boat_side_friction * local_speed_x**2

    control_signal_mtx.release()
    rot_fric  =  np.sign(rot_speed)*boat_moment_friction * rot_speed**2
    momentum =   -boat_rudder_to_pivot*(force_rudder_SB + force_rudder_PS) - np.sign(rot_speed)*boat_moment_friction * rot_speed**2 + (thruster_SB -thruster_PS)*boat_dist_between_prop/2

    return forward_force, side_force, momentum


def sim_step():
    global boat_heading, boat_speed_y, boat_speed_x, boat_ang_vel, boat_heading
    global boat_pos_y, boat_pos_x, time_n, time_w_x, time_w_y

    # calc local speed
    local_speed_y = np.cos(-boat_heading)*boat_speed_y + np.sin(-boat_heading)*boat_speed_x
    local_speed_x = np.cos(-boat_heading)*boat_speed_x - np.sin(-boat_heading)*boat_speed_y

    # calc local forces
    forward_force, side_force, torque = force_calc(local_speed_x,local_speed_y,boat_ang_vel)

    # forces from local to global coord. 
    force_y = np.cos(boat_heading)*forward_force + np.sin(boat_heading)*side_force
    force_x = np.cos(boat_heading)*side_force - np.sin(boat_heading)*forward_force

    # calc distrubances
   
    time_n += delta_T
    wind_noise_y = wind_noise_y_scalar*(np.sin(0.25*time_n) - np.cos(0.5*time_n) + 2 + 0.8*np.sin(0.5*time_n) \
                    - 1*np.sin(1*time_n) + 0.2*np.sin(5*time_n) + 0.5*np.sin(1.5*time_n)+ 0.5*np.cos(2*time_n))/5.3
    wind_noise_x = wind_noise_y_scalar*(np.sin(0.25*time_n+off_wind) - np.cos(0.5*time_n+off_wind) + 2 + 0.8*np.sin(0.5*time_n+off_wind) \
                    - 1*np.sin(1*time_n+off_wind) + 0.2*np.sin(5*time_n+off_wind) + 0.5*np.sin(1.5*time_n+off_wind)+ 0.5*np.cos(2*time_n+off_wind))/5.3
    
    time_w_y += (local_speed_y+1)*delta_T 
    time_w_x += (local_speed_x+1)*delta_T 

    rot_noise = 0.5*wave_amplitude*np.sin(time_w_y)

    water_noise_y =  wave_amplitude*np.sin(time_w_y) + water_y_stream
    water_noise_x =  wave_amplitude*np.sin(time_w_x) + water_y_stream
    # calc acc

    ang_acc =  (torque + rot_noise) / I_z 
    y_acc = (force_y + wind_noise_y + water_noise_y)/boat_mass
    x_acc = (force_x + wind_noise_x + water_noise_x)/boat_mass

    # update speed and position
    boat_speed_y += y_acc * delta_T
    boat_speed_x += x_acc * delta_T
    boat_pos_y += boat_speed_y * delta_T
    boat_pos_x += boat_speed_x * delta_T
    boat_ang_vel += ang_acc * delta_T
    boat_heading += boat_ang_vel * delta_T

    return local_speed_y

# calculates the closest distance to the path
def closet_dist_to_lines(line_segments, point):
    line_segments = line_segments - point # change problem to closest to origo
    shortest_dist = 1000000000
    for i in range(np.size(line_segments,1)-1):
        A0 = line_segments[0,i]
        A1 = line_segments[1,i]
        B0 = line_segments[0,i+1]
        B1 = line_segments[1,i+1]

        M0 = B0-A0
        M1 = B1-A1

        t = (-A0*M0 - A1*M1)/(M0**2+ M1**2)
        if t<0:
            dist = np.sqrt(A0**2+A1**2)
        elif t>1:
            dist = np.sqrt(B0**2+B1**2)
        else:
            dist = np.sqrt((A0+t*M0)**2+(A1+t*M1)**2)

        if dist < shortest_dist:
            shortest_dist = dist
    return shortest_dist

def start_microservice():
    global boat_pos_y, boat_pos_x, boat_heading, boat_ang_vel, boat_speed_y, boat_speed_x

    verbose = True
    # create a default connection id for libcluon
    cid = 112
    # Create a session to send and receive messages from a running OD4Session;
    session = OD4Session.OD4Session(cid=112)
    # recive 
    session.registerMessageCallback(5321, control_signals, porpoise_message_set.opendlv_proxy_ControlBoat)
    # Connect to the network session.
    session.connect()

    if verbose:
        print("Simulator")
    
    # define messages
    msg_waypoints = porpoise_message_set.opendlv_proxy_Waypoints()
    msg_pose_data = porpoise_message_set.opendlv_proxy_Pose()
    msg_mission = porpoise_message_set.opendlv_proxy_Mission()

    # send mission
    msg_mission.target_speed = target_speed
    msg_mission.mission_type_controller = 0
    msg_mission.mission_type_pathplanner = 3
    session.send(5325, msg_mission.SerializeToString())

    msg_mission.target_speed = target_speed
    msg_mission.mission_type_controller = mission_type_controller
    msg_mission.mission_type_pathplanner = mission_type_pathplanner
    session.send(5325, msg_mission.SerializeToString())

    # send first batch of pose data

    msg_pose_data.easting = boat_pos_x+ np.random.normal(0, gps_est_std)
    msg_pose_data.northing = boat_pos_y+ np.random.normal(0, gps_est_std)
    msg_pose_data.yaw = boat_heading+ np.random.normal(0, heading_std)
    msg_pose_data.pitch = 0
    msg_pose_data.roll = 0
    msg_pose_data.velocity = 0 

    # Send message on id 
    session.send(5324, msg_pose_data.SerializeToString())

    # plot
    fig1, ax1 = plt.subplots()
    x_L, y_L, gps_zone_num,gps_zone_letter  = utm.from_latlon(57.698356, 11.932705)
    y_L +=50
    x_L -=20

    scale_img=2.53
    imgplot = ax1.imshow(img, aspect='equal', origin='upper', extent=[x_L, x_L+915*scale_img, y_L, y_L+539*scale_img])
    ax1.plot(waypoints_global[0,:], waypoints_global[1,:], '-*k')

    plt.pause(delta_T)
  
    iter = 0
    #path_save = np.array([[boat_pos_x],[boat_pos_y], [boat_heading], [0]])
    error_dist = np.array([])

    # sim loop 
    while True:
        start = time.clock()

        # take sim step
        local_speed_y = sim_step()
        
        print("sim running: speed forward", local_speed_y)
        # calc noisy poistion from true position + noise
        boat_pos_x_noise = boat_pos_x + np.random.normal(0, gps_est_std)
        boat_pos_y_noise = boat_pos_y + np.random.normal(0, gps_north_std)
        boat_heading_noise = boat_heading + np.random.normal(0, heading_std)

        #pose msg
        msg_pose_data.easting =boat_pos_x_noise
        msg_pose_data.northing = boat_pos_y_noise
        msg_pose_data.yaw = boat_heading_noise
        msg_pose_data.pitch = 0
        msg_pose_data.roll = 0
        msg_pose_data.velocity = local_speed_y + np.random.normal(0, speed_std)

        # Send message on id 
        session.send(5324, msg_pose_data.SerializeToString())
        new_pose = np.array([[boat_pos_x],[boat_pos_y], [boat_heading], [local_speed_y]])
        #path_save = np.append(path_save,new_pose, axis=1)

        if iter%(5/delta_T) == 0:
            # turn waypoints to local 
            rot_m_ = np.array([[np.cos(-boat_heading_noise), - np.sin(-boat_heading_noise)],[np.sin(-boat_heading_noise), np.cos(-boat_heading_noise)]])
            waypoints_local = waypoints_global - np.array([[boat_pos_x_noise],[boat_pos_y_noise]])
            waypoints_local = rot_m_.dot(waypoints_local)

            # send waypoints 
            for i in range(num_waypoints):
                msg_waypoints.list_index = i
                msg_waypoints.coord_x = waypoints_local[0,i]
                msg_waypoints.coord_y = waypoints_local[1,i]
                msg_waypoints.is_last_message = 0
                if i == num_waypoints -1 :
                    msg_waypoints.is_last_message = 1
                if mission_type_pathplanner == 1 or mission_type_pathplanner == 0:
                    session.send(5323, msg_waypoints.SerializeToString())
        
        # plot 
        if iter%(1/delta_T) == 0:

            rot_m_boat = np.array([[np.cos(boat_heading), - np.sin(boat_heading)],[np.sin(boat_heading), np.cos(boat_heading)]])
            new_boat = rot_m_boat.dot(boat_points) + np.array([[boat_pos_x],[boat_pos_y]])

            boat_plt = ax1.plot(new_boat[0,:], new_boat[1,:], '-r')
            # save path into array if needed 
            new_pos = np.array([[boat_pos_x],[boat_pos_y]])
            #old_path = np.append(old_path,new_pos, axis=1)
            if iter%(10/delta_T) == 0:
                ax1.plot(boat_pos_x, boat_pos_y,'*b')

            plt.pause(delta_T/10)
            for i in range(len(boat_plt)):
                boat_plt[i].remove()

            # save error
            distance_ = closet_dist_to_lines(waypoints_global, new_pos)
            error_dist = np.append(error_dist,distance_)

        time_diff =  time.clock() - start ;
        sleep_t = delta_T-time_diff
        if sleep_t > 0:
            time.sleep(sleep_t)
        iter += 1
        if np.sqrt((boat_pos_x-waypoints_global[0,num_waypoints-1])**2 + (boat_pos_y-waypoints_global[1,num_waypoints-1])**2) < target_treashold:
            boat_plt = ax1.plot(new_boat[0,:], new_boat[1,:], '-r')
            print('Mission done')
            print('mean error' , np.mean(error_dist))
            print('max error' , np.max(error_dist))
            #np.save('Path_data', path_save)
            plt.show()

            break

if __name__ == "__main__":
    start_microservice()
