/**
 * @file offboard_test.cpp
 * @author Bryce Hopkins (bryceh@clemson.edu)
 * @brief This file contains a basic waypoint based flight routine to fly in a 2m square, 2m above initialization point.
 *
 *      Top down view of flight
 *            1 _____ 2 
 *             |     |
 *             |_____|
 *             ^      3
 *          start
 *          
 * @version 1.0
 * @date 2023-31-10
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "std_msgs/String.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include <math.h>

#define FLIGHT_ALTITUDE 2.0 // alt in m 
#define FA              FLIGHT_ALTITUDE
#define RATE            20  // loop rate hz
#define MAX_STEPS       10000 // Maximum allowed steps for path

#define PI  3.14159265358979323846264338327950
#define RAD2DEG         180 / PI

// Waypoint arrays
    // note this is in ENU coordinates since mavros will convert to NED
    // x right, y forward, z up.
    // yaw 0 = along x axis (right)
        //  yaw pi/2 = along y axis (forward)
struct Waypoint {
    double x;
    double y;
    double z;
    double yaw;
    double hold_time;
    double transition_time;
};

#define NUM_WAYPOINTS 9

// NOTE: Timings have been found to be shorter than 1 second per. unsure why at the moment.
                                  // x        Y       Z                   YAW                 HOLD    TRANSITION
Waypoint waypoints[NUM_WAYPOINTS]= {{0,       0,      FA,                 PI/2,               5,      10},
                                    {0,       2.00,   FA,                 PI/2,               0,      5},
                                    {0,       2.00,   FA,                 0,                  0,      10},
                                    {2.00,    2.00,   FA,                 0,                  0,      5},
                                    {2.00,    2.00,   FA,                 3*PI/2,             0,      10},
                                    {2.00,    0,      FA,                 3*PI/2,             0,      5},
                                    {2.00,    0,      FA,                 2*PI/2,             0,      10},
                                    {0,       0,      FA,                 2*PI/2,             0,      5},
                                    {0,       0,      FA,                 PI/2,               5,      0}};


mavros_msgs::State              current_state;
mavros_msgs::PositionTarget     path[MAX_STEPS];   // STEPS plus 8 second hover
geometry_msgs::PoseStamped      pose;
double                          starting_R, starting_P, starting_Y;
bool                            get_pose_flag;             
bool                            got_pose;

// Calculates the total number of steps required for a given set of waypoints at a given rate
int calc_steps(Waypoint waypoints[], int num_waypoints, int rate)
{
    int total_steps = 0;

    for(int i=0; i<num_waypoints; i++)
    {
        total_steps += (waypoints[i].hold_time + waypoints[i].transition_time) * rate;
    }

    return total_steps;
}

// computes shortest angular difference between two input angles [rad, floats].
// + = ccw, - = cw
float shortest_angle(float start, float end)
{
    return -((start - end + PI - (2*PI) * floor((start - end + PI)/(2*PI))) - PI);
}

int init_path()
{
    int total_steps;

    // Validate that the number/timings of waypoints doesn't exceed the allocated path size
    total_steps = calc_steps(waypoints, NUM_WAYPOINTS, RATE);
    if (total_steps > MAX_STEPS)
    {
        printf("FATAL ERROR: Waypoint steps exceed max steps. Increase MAX_STEPS or decrease mission time.");
        ROS_INFO("FATAL ERROR: Waypoint steps exceed max steps. Increase MAX_STEPS or decrease mission time.");
        exit(1);
    }
    

    int i, j, step_counter;
    step_counter = 0;

    // Create path traveling linearly from waypoint to waypoint, with designated hold timings and transition timings
    // note this is in ENU coordinates since mavros will convert to NED
    // +x right, +y forward, +z up.
    for(i=0;i<NUM_WAYPOINTS;i++)
    {

        // hold each waypoint for the designated time
        for(j=step_counter; j<step_counter + (waypoints[i].hold_time * RATE); j++)
        {
            path[j].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            path[j].type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            path[j].position.x = waypoints[i].x;
            path[j].position.y = waypoints[i].y;
            path[j].position.z = waypoints[i].z;

            path[j].yaw = waypoints[i].yaw;
        }
        // Keep track of total steps assigned
        step_counter += waypoints[i].hold_time * RATE;

        // if there isn't another waypoint, don't path to the next waypoint.
        if (i + 1 >= NUM_WAYPOINTS)
        {
            continue;
        }

        // Path to next waypoint over transition_time seconds.
        for(j=step_counter; j<step_counter+(waypoints[i].transition_time * RATE); j++)
        {
            path[j].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            path[j].type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            path[j].position.x = waypoints[i].x + (waypoints[i+1].x - waypoints[i].x) * (j-step_counter)/(waypoints[i].transition_time * RATE);
            path[j].position.y = waypoints[i].y + (waypoints[i+1].y - waypoints[i].y) * (j-step_counter)/(waypoints[i].transition_time * RATE);
            path[j].position.z = waypoints[i].z + (waypoints[i+1].z - waypoints[i].z) * (j-step_counter)/(waypoints[i].transition_time * RATE);
    
            // Path starting yaw -> starting yaw + shortest offset to target yaw, letting mavros do 360 deg conversions as necessary.
            path[j].yaw = waypoints[i].yaw + (shortest_angle(waypoints[i].yaw, waypoints[i+1].yaw)) * (j-step_counter)/(waypoints[i].transition_time * RATE);
        }
        step_counter += waypoints[i].transition_time * RATE;
    }

    // step_counter now holds the total number of intialized steps. Be sure not to index path[] further than step_counter
    return step_counter;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void get_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (get_pose_flag && ros::ok())
    {
        // XYZ stored as .pose.position.x/y/z
        // RPY stored as quaternion, so conversion to euler is necesary
    pose = *msg;
    tf::Quaternion q(pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(starting_R, starting_P, starting_Y);
    // stores R, P, Y in radians w/ 0 = right, along x-axis.
    // pose is stored in pose.pose.position.x/y/z
    // rotation stored in starting_R, starting_P, starting_Y [rad]
    
    get_pose_flag = false;
    got_pose = true;
    }
}


int main(int argc, char **argv)
{
    int i;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State>
                                        ("mavros/state", 10, state_cb);
    ros::Publisher target_local_pub     = nh.advertise<mavros_msgs::PositionTarget>
                                        ("mavros/setpoint_raw/local", 10);
    ros::Subscriber local_pose_sub      = nh.subscribe<geometry_msgs::PoseStamped>
                                        ("mavros/local_position/pose", 30, get_pose_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCT...");
    }

    // Grab starting point
        // set flags
    got_pose = false;
    get_pose_flag = true;
        // wait until pose is stored in starting_local_pose (pose.pose.x/y/z)
    while (!got_pose)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // print pose for debug purposes:
    ROS_INFO("current pos: x= %f, y = %f, z = %f", pose.pose.position.x,
                                                   pose.pose.position.y,
                                                   pose.pose.position.z);
    ROS_INFO("current orient: r = %f, p = %f, y = %f [deg]", starting_R * RAD2DEG, starting_P * RAD2DEG, starting_Y * RAD2DEG);
    
    ROS_INFO("NOTE: first waypoint is: x = %f, y = %f, z = %f, yaw = %f", waypoints[0].x,
                                                                          waypoints[0].y,
                                                                          waypoints[0].z,
                                                                          waypoints[0].yaw * RAD2DEG);

    // keep this pose constant, home position (initialized relative to starting_local_pose)
    mavros_msgs::PositionTarget position_home;
    position_home.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_home.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    position_home.position.x = pose.pose.position.x;
    position_home.position.y = pose.pose.position.y;
    position_home.position.z = pose.pose.position.z + FLIGHT_ALTITUDE;


    ROS_INFO("Initializing Path...");
    int NUM_STEPS = init_path();
    ROS_INFO("Path Successfully Initialized!");


    ROS_INFO("Sending initial setpoints...");
    //send a few setpoints before starting
    for(i = 100; ros::ok() && i > 0; --i){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

HOME:
    ROS_INFO("Waiting for Offboard Mode. Flight starts once Offboard is enabled...");
    while(ros::ok()){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
        if(current_state.mode == "OFFBOARD") break;

        if (!ros::ok())
        {
            ROS_INFO("ROS is NOT ok. Shutting down...");
            ros::shutdown();
        }
    }

    // give the system 2 seconds to get to home position
    i = RATE * 2;
    ROS_INFO("Moving to home position");
    while(ros::ok() && i>0){
        // return to home position if px4 falls out of offboard mode or disarms
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            goto HOME;
        }
        i--;
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

    // Now begin path
    i=0;
    ROS_INFO("following path");
    while(ros::ok()){
        // return to home position if px4 falls out of offboard mode or disarms
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            goto HOME;
        }
        target_local_pub.publish(path[i]);
        i++;
        if (i>=NUM_STEPS)
        {
            goto END;
        }
        ros::spinOnce();
        rate.sleep();
    }

END:
    // Path complete!
    ROS_INFO("Path completed! Now exiting");
    return 0;
}

