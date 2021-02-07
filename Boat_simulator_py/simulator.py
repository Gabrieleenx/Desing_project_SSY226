import porpoiseMessageSet_pb2 as porpoise_message_set
import OD4Session
import time
import sys
import getopt
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import threading
## --- bugs ---
# nan from sim step

# ---------- Simulation Parameters --------
# boat parameters
delta_T = 0.01 # simlation step time
boat_mass = 5.0 # in kg
boat_forward_thrust = 20 # max in N per proppeler. 
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

boat_pos_y = 2344 # in meters, northing
boat_pos_x = 1452 # in meters, esting
boat_heading = 0.5*np.pi # headig in radians. 0 north, + gives west, i.e. right hand rule. 
boat_ang_vel = 0 # angular velocity rad/s
boat_speed_y = 0; # in m/s
boat_speed_x = 0; # in m/s

local_pos_x = boat_pos_x
local_pos_y = boat_pos_y
local_pos_h = boat_heading


# plot window 
x_min = boat_pos_x - 10
x_max = boat_pos_x + 10
y_min = boat_pos_y - 15
y_max = boat_pos_y + 10

# initial boat control signals

motor_values = np.array([0,0,0,0,0])
target_speed = 1
mission_type_controller = 1
mission_type_pathplanner = 1

# noise and disturbances

# waypoints 
waypoints_global = np.array([[0,1,3,6,9,9,2], [2,2.5,3,2,0,-4,-10]])+np.array([[1452],[2344]]) # x , y 
num_waypoints = 7
# mutex

control_signal_mtx = threading.Lock()

# Variables

I_z = (1.0/12.0) * boat_mass * (boat_width**2 + boat_length**2) # moment of inertia... aproximated as rectangle

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

boat_points = np.array([[x1, x2, x3, x4, x5, x1],[y1,y2,y3,y4,y5,y1]])

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
    global boat_pos_y, boat_pos_x
    # calc local speed
    local_speed_y = np.cos(-boat_heading)*boat_speed_y + np.sin(-boat_heading)*boat_speed_x
    local_speed_x = np.cos(-boat_heading)*boat_speed_x - np.sin(-boat_heading)*boat_speed_y
    # calc local forces
    forward_force, side_force, torque = force_calc(local_speed_x,local_speed_y,boat_ang_vel)
    # forces from local to global coord. 
    force_y = np.cos(boat_heading)*forward_force + np.sin(boat_heading)*side_force
    force_x = np.cos(boat_heading)*side_force - np.sin(boat_heading)*forward_force
    # calc distrubances
    rot_noise = 0
    wind_noise_y = 0
    wind_noise_x = 0
    water_noise_y = 0
    water_noise_x = 0
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

def start_microservice():
    global local_pos_h, local_pos_x, local_pos_y

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
    msg_mission.mission_type_controller = mission_type_controller
    msg_mission.mission_type_pathplanner = mission_type_pathplanner
    session.send(5325, msg_mission.SerializeToString())

    # turn waypoints to local 
    rot_m = np.array([[np.cos(-boat_heading), - np.sin(-boat_heading)],[np.sin(-boat_heading), np.cos(-boat_heading)]])
    waypoints_local = waypoints_global - np.array([[boat_pos_x],[boat_pos_y]])
    waypoints_local = rot_m.dot(waypoints_local)

    # send first batch of pose data

    init_boat_pos_y = boat_pos_y
    init_boat_pos_x = boat_pos_x
    init_boat_heading = boat_heading
    #msg_pose_data.init_pos_x = init_boat_pos_x
    #msg_pose_data.init_pos_y = init_boat_pos_y
    #msg_pose_data.init_yaw = init_boat_heading
    #msg_pose_data.init_roll = 0
    #msg_pose_data.init_pitch = 0
    msg_pose_data.easting = boat_pos_x
    msg_pose_data.northing = boat_pos_y
    msg_pose_data.yaw = boat_heading
    msg_pose_data.pitch = 0
    msg_pose_data.roll = 0
    msg_pose_data.velocity = 0 

    # Send message on id 
    session.send(5324, msg_pose_data.SerializeToString())
    
    # send waypoints 
    for i in range(num_waypoints):
        msg_waypoints.list_index = i
        msg_waypoints.coord_x = waypoints_local[0,i]
        msg_waypoints.coord_y = waypoints_local[1,i]
        msg_waypoints.is_last_message = 0
        if i == num_waypoints -1 :
            msg_waypoints.is_last_message = 1
        session.send(5323, msg_waypoints.SerializeToString())
        time.sleep(delta_T)
    
    fig, ax = plt.subplots() 
    ax.axis([x_min, x_max, y_min, y_max])
  
    iter = 0
    old_path = np.array([[init_boat_pos_x],[init_boat_pos_y]])

    # sim loop 
    while True:
        iter += 1

        # take sim step
        local_speed_y = sim_step()
        print("sim running: speed forward", local_speed_y)

        #if verbose:   
        #msg_pose_data.init_pos_x = init_boat_pos_x
        #msg_pose_data.init_pos_y = init_boat_pos_y
        #msg_pose_data.init_yaw = init_boat_heading
        #msg_pose_data.init_roll = 0
        #msg_pose_data.init_pitch = 0
        msg_pose_data.easting = boat_pos_x
        msg_pose_data.northing = boat_pos_y
        msg_pose_data.yaw = boat_heading
        msg_pose_data.pitch = 0
        msg_pose_data.roll = 0
        msg_pose_data.velocity = local_speed_y 

        # Send message on id 
        session.send(5324, msg_pose_data.SerializeToString())
        
        if iter%(5/delta_T) == 0:
            # turn waypoints to local 
            rot_m_ = np.array([[np.cos(-boat_heading), - np.sin(-boat_heading)],[np.sin(-boat_heading), np.cos(-boat_heading)]])
            waypoints_local = waypoints_global - np.array([[boat_pos_x],[boat_pos_y]])
            waypoints_local = rot_m_.dot(waypoints_local)

            local_pos_x = boat_pos_x
            local_pos_y = boat_pos_y
            local_pos_h = boat_heading

            # send waypoints 
            for i in range(num_waypoints):
                msg_waypoints.list_index = i
                msg_waypoints.coord_x = waypoints_local[0,i]
                msg_waypoints.coord_y = waypoints_local[1,i]
                msg_waypoints.is_last_message = 0
                if i == num_waypoints -1 :
                    msg_waypoints.is_last_message = 1
                session.send(5323, msg_waypoints.SerializeToString())
        
        # plot 
        if iter%(0.1/delta_T) == 0:
            ax.plot(waypoints_global[0,:], waypoints_global[1,:], '-*k')
            plt.xlim(x_min, x_max)
            plt.ylim(y_min, y_max)
            #ax.plot(waypoints_local[0,:], waypoints_local[1,:], '-*k')
            ax.axis('equal')

            ax.plot(boat_pos_x, boat_pos_y, '*b')

            rot_m_boat = np.array([[np.cos(boat_heading), - np.sin(boat_heading)],[np.sin(boat_heading), np.cos(boat_heading)]])
            new_boat = rot_m_boat.dot(boat_points) + np.array([[boat_pos_x],[boat_pos_y]])

            #rot_m_boat = np.array([[np.cos(boat_heading - local_pos_h), - np.sin(boat_heading-local_pos_h)],[np.sin(boat_heading-local_pos_h), np.cos(boat_heading-local_pos_h)]])
            #rot_m_boat_ = np.array([[np.cos(-local_pos_h), - np.sin(-local_pos_h)],[np.sin(-local_pos_h), np.cos(-local_pos_h)]])
            #new_boat = rot_m_boat.dot(boat_points) + rot_m_boat_.dot(np.array([[boat_pos_x-local_pos_x],[boat_pos_y-local_pos_y]]))


            ax.axis('equal')
            ax.plot(new_boat[0,:], new_boat[1,:], '-r')
            # Wait delta_T seconds before next reading
            new_pos = np.array([[boat_pos_x],[boat_pos_y]])
            old_path = np.append(old_path,new_pos, axis=1)
            ax.plot(old_path[0,:], old_path[1,:], '*b')

            plt.pause(delta_T)
            ax.clear()
        time.sleep(delta_T)

if __name__ == "__main__":
    start_microservice()
