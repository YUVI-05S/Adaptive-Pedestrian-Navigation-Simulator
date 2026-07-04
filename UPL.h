#ifndef UPL_H
#define UPL_H
#include <cmath>
#include "headerClass.h"
#include "utils.h"

//-----------------------------------------------------
// Magnitude of a vector
//-----------------------------------------------------

//-----------------------------------------------------
// Dot product
//-----------------------------------------------------



//-----------------------------------------------------
// Normalize vector
//-----------------------------------------------------

//-----------------------------------------------------
// Self Force
//-----------------------------------------------------

Coordinates self_force(const Pedestrian& ped,
                       const Parameters& param)
{
    Coordinates Fs;

    Fs.x = (ped.desired_vel_x - ped.vel_x)/param.tau_alpha;

    Fs.y = (ped.desired_vel_y - ped.vel_y)/param.tau_alpha;

    Fs.mod = magnitude(Fs.x,Fs.y);

    return Fs;
}
//-----------------------------------------------------
// Relative position vector
//-----------------------------------------------------

Coordinates relativePosition(const Pedestrian& ped,
                             const Obstacle& obst)
{
    Coordinates xij;

    xij.x = ped.pos_x - obst.x;
    xij.y = ped.pos_y - obst.y;

    xij.mod = magnitude(xij.x,xij.y);

    return xij;
}



//-----------------------------------------------------
// Relative velocity vector
//-----------------------------------------------------

Coordinates relativeVelocity(const Pedestrian& ped,
                             const Obstacle& obst)
{
    Coordinates vij;

    vij.x = ped.vel_x - obst.vel_x;
    vij.y = ped.vel_y - obst.vel_y;

    vij.mod = magnitude(vij.x,vij.y);

    return vij;
}
//-----------------------------------------------------
// Compute a,b,c,d for Pedestrian-Obstacle interaction
//-----------------------------------------------------

void abcd(const Pedestrian& ped,
          const Obstacle& obst,
          double& a,
          double& b,
          double& c,
          double& d)
{
    Coordinates xij = relativePosition(ped,obst);

    Coordinates vij = relativeVelocity(ped,obst);

    a = vij.mod*vij.mod;

    b = -dotProduct(xij.x,xij.y,
                    vij.x,vij.y);

    c = xij.mod*xij.mod
        -
        (ped.radius + obst.radius)
        *
        (ped.radius + obst.radius);

    d = b*b - a*c;
}
//-----------------------------------------------------
// Collision time τ for Pedestrian-Obstacle interaction
//-----------------------------------------------------

double tau(const Pedestrian& ped,
           const Obstacle& obst)
{
    double a,b,c,d;

    abcd(ped,obst,a,b,c,d);

    // Relative speed almost zero
    if(a < 1e-12)
        return -1.0;

    // No real collision
    if(d <= 0.0)
        return -1.0;

    double collision_time =
            (b - sqrt(d))/a;

    // Collision already occurred in past
    if(collision_time <= 0.0)
        return -1.0;

    // Avoid singularity
    if(collision_time < 1e-8)
        collision_time = 1e-8;

    return collision_time;
}
//-----------------------------------------------------
// Check if obstacle lies inside Field Of View
//-----------------------------------------------------

bool inFieldOfView(const Pedestrian& ped,
                   const Obstacle& obst,
                   const Parameters& param)
{
    Coordinates viewDirection;
    Coordinates targetDirection;

    //-----------------------------------------
    // Determine viewing direction
    //-----------------------------------------

    if(ped.speed > 1e-6)
    {
        viewDirection =
                normalize(ped.vel_x,
                          ped.vel_y);
    }
    else
    {
        viewDirection =
                normalize(ped.Vgoal.x - ped.pos_x,
                          ped.Vgoal.y - ped.pos_y);
    }

    //-----------------------------------------
    // Direction toward obstacle
    //-----------------------------------------

    targetDirection =
            normalize(obst.x - ped.pos_x,
                      obst.y - ped.pos_y);

    //-----------------------------------------
    // Compute angle between them
    //-----------------------------------------

    double cosineTheta =
            dotProduct(viewDirection.x,
                       viewDirection.y,
                       targetDirection.x,
                       targetDirection.y);

    // Numerical safety
    if(cosineTheta > 1.0)
        cosineTheta = 1.0;

    if(cosineTheta < -1.0)
        cosineTheta = -1.0;

    double angle =
            acos(cosineTheta)
            *180.0/M_PI;

    //-----------------------------------------
    // Half angle = 65 degrees
    //-----------------------------------------

    if(angle <= param.fieldOfView/2.0)
        return true;

    return false;
}
//-----------------------------------------------------
// UPL force : Pedestrian - Obstacle
//-----------------------------------------------------

Coordinates UPL(const Pedestrian& ped,
                const Obstacle& obst,
                const Parameters& param)
{
    Coordinates Fi;

    Fi.x = 0.0;
    Fi.y = 0.0;
    Fi.mod = 0.0;

    //-----------------------------------------
    // FOV check
    //-----------------------------------------

    if(!inFieldOfView(ped,obst,param))
        return Fi;

    //-----------------------------------------
    // Collision time
    //-----------------------------------------

    double collision_time = tau(ped,obst);

    if(collision_time < 0.0)
        return Fi;

    //-----------------------------------------
    // Relative vectors
    //-----------------------------------------

    Coordinates xij = relativePosition(ped,obst);

    Coordinates vij = relativeVelocity(ped,obst);

    //-----------------------------------------
    // ABCD coefficients
    //-----------------------------------------

    double a,b,c,d;

    abcd(ped,obst,a,b,c,d);

    if(a < 1e-12)
        return Fi;

    //-----------------------------------------
    // Scalar coefficient A
    //-----------------------------------------

    double A;

    A = param.k
        *
        exp(-collision_time/param.tau0)
        /
        a
        /
        collision_time
        /
        collision_time
        *
        (
            (2.0/collision_time)
            +
            (1.0/param.tau0)
        );

    //-----------------------------------------
    // Vector coefficient B
    //-----------------------------------------

    Coordinates B;

    B.x =
        vij.x
        -
        (
            a*xij.x
            +
            b*vij.x
        )
        /
        sqrt(d);

    B.y =
        vij.y
        -
        (
            a*xij.y
            +
            b*vij.y
        )
        /
        sqrt(d);

    //-----------------------------------------
    // Final force
    //-----------------------------------------

    Fi.x = -A*B.x;

    Fi.y = -A*B.y;

    Fi.mod = magnitude(Fi.x,Fi.y);

    return Fi;
}
//-----------------------------------------------------
// UPL force : Pedestrian - Pedestrian
//-----------------------------------------------------

Coordinates UPL(const Pedestrian& ped1,
                const Pedestrian& ped2,
                const Parameters& param)
{
    Coordinates Fi;

    //-----------------------------------------
    // Same pedestrian
    //-----------------------------------------

    if(ped1.id == ped2.id)
    {
        Fi.x = 0.0;
        Fi.y = 0.0;
        Fi.mod = 0.0;

        return Fi;
    }

    //-----------------------------------------
    // Convert ped2 into obstacle
    //-----------------------------------------

    Obstacle tempObst;

    tempObst.x = ped2.pos_x;
    tempObst.y = ped2.pos_y;

    tempObst.vel_x = ped2.vel_x;
    tempObst.vel_y = ped2.vel_y;

    tempObst.radius = ped2.radius;

    //-----------------------------------------
    // Reuse Pedestrian-Obstacle UPL
    //-----------------------------------------

    Fi = UPL(ped1,tempObst,param);

    return Fi;
}
//-----------------------------------------------------
// Contact Force : Pedestrian - Obstacle
//-----------------------------------------------------

Coordinates ContactForce(const Pedestrian& ped,
                         const Obstacle& obst)
{
    Coordinates Fc;

    Fc.x = 0.0;
    Fc.y = 0.0;
    Fc.mod = 0.0;

    return Fc;
}



//-----------------------------------------------------
// Contact Force : Pedestrian - Pedestrian
//-----------------------------------------------------

Coordinates ContactForce(const Pedestrian& ped1,
                         const Pedestrian& ped2)
{
    Coordinates Fc;

    Fc.x = 0.0;
    Fc.y = 0.0;
    Fc.mod = 0.0;

    return Fc;
}
//-----------------------------------------------------
// Left Wall
//-----------------------------------------------------

Obstacle leftWallObstacle(const Boundary& boundary)
{
    Obstacle wall;

    wall.radius = 1e9;

    wall.x = -wall.radius;
    wall.y = boundary.height/2.0;

    wall.vel_x = 0.0;
    wall.vel_y = 0.0;

    return wall;
}



//-----------------------------------------------------
// Right Wall
//-----------------------------------------------------

Obstacle rightWallObstacle(const Boundary& boundary)
{
    Obstacle wall;

    wall.radius = 1e9;

    wall.x = boundary.width + wall.radius;
    wall.y = boundary.height/2.0;

    wall.vel_x = 0.0;
    wall.vel_y = 0.0;

    return wall;
}



//-----------------------------------------------------
// Bottom Wall
//-----------------------------------------------------

Obstacle bottomWallObstacle(const Boundary& boundary)
{
    Obstacle wall;

    wall.radius = 1e9;

    wall.x = boundary.width/2.0;
    wall.y = -wall.radius;

    wall.vel_x = 0.0;
    wall.vel_y = 0.0;

    return wall;
}



//-----------------------------------------------------
// Top Wall
//-----------------------------------------------------

Obstacle topWallObstacle(const Boundary& boundary)
{
    Obstacle wall;

    wall.radius = 1e9;

    wall.x = boundary.width/2.0;
    wall.y = boundary.height + wall.radius;

    wall.vel_x = 0.0;
    wall.vel_y = 0.0;

    return wall;
}
#endif