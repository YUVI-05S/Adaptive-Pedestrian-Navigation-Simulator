#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include "headerClass.h"
#include "utils.h"
//-----------------------------------------------------
// Position update (Velocity Verlet)
//-----------------------------------------------------

void updatePosition(Pedestrian& ped,
                    const Parameters& param)
{
    ped.pos_x =
            ped.pos_x
            +
            ped.vel_x*param.dt
            +
            0.5*ped.Fold_x*param.dt*param.dt;

    ped.pos_y =
            ped.pos_y
            +
            ped.vel_y*param.dt
            +
            0.5*ped.Fold_y*param.dt*param.dt;
}//-----------------------------------------------------
// Half-step velocity
//-----------------------------------------------------

void updateHalfVelocity(Pedestrian& ped,
                        const Parameters& param)
{
    ped.vel_half_x =
            ped.vel_x
            +
            0.5*ped.Fold_x*param.dt;

    ped.vel_half_y =
            ped.vel_y
            +
            0.5*ped.Fold_y*param.dt;
}
//-----------------------------------------------------
// Complete velocity update
//-----------------------------------------------------

void updateVelocity(Pedestrian& ped,
                    const Parameters& param)
{
    ped.vel_x =
            ped.vel_half_x
            +
            0.5*ped.Fnew_x*param.dt;

    ped.vel_y =
            ped.vel_half_y
            +
            0.5*ped.Fnew_y*param.dt;

    ped.speed =
            magnitude(ped.vel_x,
                      ped.vel_y);
}
//-----------------------------------------------------
// Limit acceleration magnitude
//-----------------------------------------------------

void limitAcceleration(Pedestrian& ped,
                       const Parameters& param)
{
    double acc_mag =
            magnitude(ped.Fnew_x,
                      ped.Fnew_y);

    if(acc_mag > param.maxAcceleration)
    {
        double scale =
                param.maxAcceleration
                /
                acc_mag;

        ped.Fnew_x *= scale;
        ped.Fnew_y *= scale;
    }
}
//-----------------------------------------------------
// Store new force as old force
//-----------------------------------------------------

void updateOldForce(Pedestrian& ped)
{
    ped.Fold_x = ped.Fnew_x;

    ped.Fold_y = ped.Fnew_y;
}
//-----------------------------------------------------
// Limit pedestrian speed
//-----------------------------------------------------

void limitSpeed(Pedestrian& ped,
                double maxSpeed = 3.0)
{
    double speed =
            magnitude(ped.vel_x,
                      ped.vel_y);

    if(speed > maxSpeed)
    {
        double scale =
                maxSpeed
                /
                speed;

        ped.vel_x *= scale;
        ped.vel_y *= scale;

        ped.speed =
                magnitude(ped.vel_x,
                          ped.vel_y);
    }
}
//-----------------------------------------------------
// Reset forces before each iteration
//-----------------------------------------------------

void resetForces(Pedestrian& ped)
{
    ped.Fnew_x = 0.0;
    ped.Fnew_y = 0.0;
}
//-----------------------------------------------------
// Update total force magnitude
//-----------------------------------------------------

void updateForceMagnitude(Pedestrian& ped)
{
    ped.Ftotal_mag =
            magnitude(ped.Fnew_x,
                      ped.Fnew_y);
}
//-----------------------------------------------------
// Update acceleration
//-----------------------------------------------------

void updateAcceleration(Pedestrian& ped)
{
    ped.acc_x = ped.Fnew_x;

    ped.acc_y = ped.Fnew_y;

    ped.acc_mag =
            magnitude(ped.acc_x,
                      ped.acc_y);
}
//-----------------------------------------------------
// Check if final goal has been reached
//-----------------------------------------------------

void checkGoalReached(Pedestrian& ped,
                      double tolerance = 0.20)
{
    double dx =
            ped.finalGoal_x
            -
            ped.pos_x;

    double dy =
            ped.finalGoal_y
            -
            ped.pos_y;

    double distance =
            magnitude(dx,dy);

    if(distance < tolerance)
    {
        ped.reachedGoal = true;
    }
}
//-----------------------------------------------------
// Stop pedestrian after reaching destination
//-----------------------------------------------------

void stopPedestrian(Pedestrian& ped)
{
    if(ped.reachedGoal)
    {
        ped.vel_x = 0.0;
        ped.vel_y = 0.0;

        ped.speed = 0.0;

        ped.vel_half_x = 0.0;
        ped.vel_half_y = 0.0;

        ped.Fnew_x = 0.0;
        ped.Fnew_y = 0.0;

        ped.Fold_x = 0.0;
        ped.Fold_y = 0.0;
    }
}
//-----------------------------------------------------
// Update distance from final goal
//-----------------------------------------------------

void updateDistanceFromFinalGoal(Pedestrian& ped)
{
    double dx =
            ped.finalGoal_x
            -
            ped.pos_x;

    double dy =
            ped.finalGoal_y
            -
            ped.pos_y;

    ped.dist_from_finalGoal =
            magnitude(dx,dy);
}
//-----------------------------------------------------
// Update distance from current variable goal
//-----------------------------------------------------

void updateDistanceFromVariableGoal(Pedestrian& ped)
{
    double dx =
            ped.Vgoal.x
            -
            ped.pos_x;

    double dy =
            ped.Vgoal.y
            -
            ped.pos_y;

    ped.dist_from_goal =
            magnitude(dx,dy);
}
//-----------------------------------------------------
// Update desired velocity components
//-----------------------------------------------------

void updateDesiredVelocity(Pedestrian& ped)
{
    Coordinates direction =
            normalize(
                ped.Vgoal.x - ped.pos_x,
                ped.Vgoal.y - ped.pos_y);

    ped.desired_vel_x =
            ped.desired_speed
            *
            direction.x;

    ped.desired_vel_y =
            ped.desired_speed
            *
            direction.y;
}
//-----------------------------------------------------
// Update pedestrian speed
//-----------------------------------------------------

void updateSpeed(Pedestrian& ped)
{
    ped.speed =
            magnitude(
                ped.vel_x,
                ped.vel_y);
}
//-----------------------------------------------------
// Reset goal status
//-----------------------------------------------------

void resetGoalStatus(Pedestrian& ped)
{
    ped.reachedGoal = false;
}
//-----------------------------------------------------
// Reset accelerations
//-----------------------------------------------------

void resetAcceleration(Pedestrian& ped)
{
    ped.acc_x = 0.0;
    ped.acc_y = 0.0;
    ped.acc_mag = 0.0;
}
//-----------------------------------------------------
// Reset force magnitude
//-----------------------------------------------------

void resetForceMagnitude(Pedestrian& ped)
{
    ped.Ftotal_mag = 0.0;
}
//-----------------------------------------------------
// Reset half-step velocity
//-----------------------------------------------------

void resetHalfVelocity(Pedestrian& ped)
{
    ped.vel_half_x = ped.vel_x;
    ped.vel_half_y = ped.vel_y;
}
//-----------------------------------------------------
// Initialize pedestrian state
//-----------------------------------------------------

void initializePedestrianState(Pedestrian& ped)
{
    //-------------------------------------
    // Speed
    //-------------------------------------

    ped.speed =
            magnitude(
                ped.vel_x,
                ped.vel_y);

    //-------------------------------------
    // Half velocities
    //-------------------------------------

    ped.vel_half_x = ped.vel_x;

    ped.vel_half_y = ped.vel_y;

    //-------------------------------------
    // Forces
    //-------------------------------------

    ped.Fold_x = 0.0;
    ped.Fold_y = 0.0;

    ped.Fnew_x = 0.0;
    ped.Fnew_y = 0.0;

    //-------------------------------------
    // Accelerations
    //-------------------------------------

    ped.acc_x = 0.0;
    ped.acc_y = 0.0;
    ped.acc_mag = 0.0;

    //-------------------------------------
    // Total force
    //-------------------------------------

    ped.Ftotal_mag = 0.0;

    //-------------------------------------
    // Goal status
    //-------------------------------------

    ped.reachedGoal = false;

    //-------------------------------------
    // Distances
    //-------------------------------------

    updateDistanceFromFinalGoal(ped);

    updateDistanceFromVariableGoal(ped);

    //-------------------------------------
    // Desired velocity
    //-------------------------------------

    updateDesiredVelocity(ped);
}
#endif