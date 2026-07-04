#ifndef HEADERCLASS_H
#define HEADERCLASS_H

#include <vector>
#include <deque>

//=====================================================
// Basic coordinate class
//=====================================================

class Coordinates
{
public:

    double x = 0.0;
    double y = 0.0;
    double mod = 0.0;

};



//=====================================================
// Circular obstacle
//=====================================================

class Obstacle
{
public:

    int id = 0;

    double x = 0.0;
    double y = 0.0;

    double radius = 0.0;
double theta = 0.0;
    // Static obstacle
    double vel_x = 0.0;
    double vel_y = 0.0;

    // Used in VGA cluster formation
    int clusterNo = -1;

};



//=====================================================
// Variable Goal
//=====================================================

class VariableGoal
{
public:

    double x = 0.0;
    double y = 0.0;

    // 0 = left
    // 1 = right
    int sideChosen = -1;

};



//=====================================================
// Cluster information
//=====================================================
class Cluster
{
public:

    int id = 0;

    int direct = -1;

    int length = 0;
};



//=====================================================
// Pedestrian
//=====================================================

class Pedestrian
{
public:

    //----------------------------------------
    // Identification
    //----------------------------------------

    int id = 0;

    //----------------------------------------
    // Position
    //----------------------------------------

    double pos_x = 0.0;
    double pos_y = 0.0;
//----------------------------------------
// Initial position
//----------------------------------------

double initial_pos_x = 0.0;
double initial_pos_y = 0.0;
    //----------------------------------------
    // Velocity
    //----------------------------------------

    double vel_x = 0.0;
    double vel_y = 0.0;

    double speed = 0.0;

    //----------------------------------------
    // Half-step velocity (Velocity Verlet)
    //----------------------------------------

    double vel_half_x = 0.0;
    double vel_half_y = 0.0;

    //----------------------------------------
    // Radius
    //----------------------------------------

    double radius = 0.23;

    //----------------------------------------
    // Desired speed
    //----------------------------------------

    double desired_speed = 1.47;

    //----------------------------------------
    // Desired velocity
    //----------------------------------------

    double desired_vel_x = 0.0;
    double desired_vel_y = 0.0;

    //----------------------------------------
    // Final destination
    //----------------------------------------

    double finalGoal_x = 0.0;
    double finalGoal_y = 0.0;

    //----------------------------------------
    // Variable goal
    //----------------------------------------

    VariableGoal Vgoal;

    //----------------------------------------
    // Distance quantities
    //----------------------------------------

    double dist_from_goal = 0.0;
    double dist_from_finalGoal = 0.0;

    //----------------------------------------
    // Self force
    //----------------------------------------

    double Fs_x = 0.0;
    double Fs_y = 0.0;

    //----------------------------------------
    // UPL force
    //----------------------------------------

    double Fi_UPL_x = 0.0;
    double Fi_UPL_y = 0.0;

    //----------------------------------------
    // Contact force (future)
    //----------------------------------------

    double Fi_contact_x = 0.0;
    double Fi_contact_y = 0.0;

    //----------------------------------------
    // Total force
    //----------------------------------------

    double Ftotal_x = 0.0;
    double Ftotal_y = 0.0;

    double Ftotal_mag = 0.0;

    //----------------------------------------
    // Previous force (Velocity Verlet)
    //----------------------------------------

    double Fold_x = 0.0;
    double Fold_y = 0.0;

    //----------------------------------------
    // New force
    //----------------------------------------

    double Fnew_x = 0.0;
    double Fnew_y = 0.0;

    //----------------------------------------
    // Acceleration
    //----------------------------------------

    double acc_x = 0.0;
    double acc_y = 0.0;

    double acc_mag = 0.0;

    //----------------------------------------
    // Collision time (UPL)
    //----------------------------------------

    double tau = 1e9;

    //----------------------------------------
    // VGA information
    //----------------------------------------

    int clusterNo = -1;

    //----------------------------------------
    // Goal status
    //----------------------------------------

    bool reachedGoal = false;

    //----------------------------------------
    // Memory for recurrence detection
    //----------------------------------------

    std::deque<Coordinates> previousGoals;

};



//=====================================================
// Boundary information
//=====================================================

class Boundary
{
public:

    double width = 0.0;
    double height = 0.0;

    bool leftWall = true;
    bool rightWall = true;
    bool topWall = true;
    bool bottomWall = true;

};



//=====================================================
// Simulation parameters
//=====================================================

class Parameters
{
public:

    //----------------------------------------
    // Time
    //----------------------------------------

    double dt = 0.01;

    double Tmax = 150.0;

    //----------------------------------------
    // Self force
    //----------------------------------------

    double tau_alpha = 0.635;

    //----------------------------------------
    // UPL parameters
    //----------------------------------------

    double tau0 = 3.0;

    double k = 1.5;

    //----------------------------------------
    // Acceleration limit
    //----------------------------------------

    double maxAcceleration = 7.0;

    //----------------------------------------
    // Field of view
    //----------------------------------------

    double fieldOfView = 130.0;

    //----------------------------------------
    // VGA
    //----------------------------------------

    int VGAinterval = 6;

};

#endif