#include <iostream>
#include <fstream>
#include <vector>

#include "headerClass.h"
#include "utils.h"
#include "UPL.h"
#include "VGA.h"
#include "Integrator.h"

using namespace std;

int main()
{
    //-----------------------------------------------------
    // Simulation objects
    //-----------------------------------------------------

    Parameters param;

    Boundary field;

    vector<Pedestrian> pedestrians;

    vector<Obstacle> staticObstacles;

    vector<Obstacle> obstacles;

    //-----------------------------------------------------
    // User input variables
    //-----------------------------------------------------

    int numberOfPedestrians;

    int numberOfStaticObstacles;

    int useWalls;

    int stochasticVGA;

    int leftWall;
    int rightWall;
    int topWall;
    int bottomWall;

    //-----------------------------------------------------
    // Output file
    //-----------------------------------------------------

    ofstream trajectory("trajectory.csv");
      trajectory
            << "Time,"
            << "PedestrianID,"
            << "PositionX,"
            << "PositionY,"
            << "VelocityX,"
            << "VelocityY,"
            << "Speed,"
            << "VariableGoalX,"
            << "VariableGoalY,"
            << "ReachedGoal"
            << endl;
    //-----------------------------------------------------
    // Field dimensions
    //-----------------------------------------------------

    cout << "----------------------------------------" << endl;
    cout << "PEDESTRIAN DYNAMICS SIMULATION" << endl;
    cout << "----------------------------------------" << endl;

    cout << endl;

    cout << "Enter field width (m): ";
    cin >> field.width;

    cout << "Enter field height (m): ";
    cin >> field.height;

    //-----------------------------------------------------
    // Walls
    //-----------------------------------------------------

    cout << endl;

    cout << "Use walls?" << endl;
    cout << "1 = Yes" << endl;
    cout << "0 = No" << endl;
    cout << "Choice : ";

    cin >> useWalls;

    if(useWalls == 1)
    {
        cout << endl;

        cout << "Left wall (1 = Yes, 0 = No): ";
        cin >> leftWall;

        cout << "Right wall (1 = Yes, 0 = No): ";
        cin >> rightWall;

        cout << "Top wall (1 = Yes, 0 = No): ";
        cin >> topWall;

        cout << "Bottom wall (1 = Yes, 0 = No): ";
        cin >> bottomWall;

        field.leftWall = leftWall;
        field.rightWall = rightWall;
        field.topWall = topWall;
        field.bottomWall = bottomWall;
    }
    else
    {
        field.leftWall = false;
        field.rightWall = false;
        field.topWall = false;
        field.bottomWall = false;
    }

    //-----------------------------------------------------
    // Static obstacles
    //-----------------------------------------------------

    cout << endl;

    cout << "Enter number of static obstacles : ";

    cin >> numberOfStaticObstacles;

    staticObstacles.resize(numberOfStaticObstacles);

    for(int i=0;i<numberOfStaticObstacles;i++)
    {
        cout << endl;

        cout << "Obstacle " << i+1 << endl;

        staticObstacles[i].id = i;

        cout << "x : ";
        cin >> staticObstacles[i].x;

        cout << "y : ";
        cin >> staticObstacles[i].y;

        cout << "radius (m): ";
        cin >> staticObstacles[i].radius;
    }
        //-----------------------------------------------------
    // Pedestrians
    //-----------------------------------------------------

    cout << endl;
//-----------------------------------------------------
// Convert selected walls into obstacles
//-----------------------------------------------------

if(field.leftWall)
{
    staticObstacles.push_back(
            leftWallObstacle(field));
}

if(field.rightWall)
{
    staticObstacles.push_back(
            rightWallObstacle(field));
}

if(field.topWall)
{
    staticObstacles.push_back(
            topWallObstacle(field));
}

if(field.bottomWall)
{
    staticObstacles.push_back(
            bottomWallObstacle(field));
}

    cout << "Enter number of pedestrians : ";

    cin >> numberOfPedestrians;

    pedestrians.resize(numberOfPedestrians);

    for(int i=0;i<numberOfPedestrians;i++)
    {
        cout << endl;

        cout << "----------------------------------------" << endl;
        cout << "Pedestrian " << i+1 << endl;
        cout << "----------------------------------------" << endl;

        pedestrians[i].id = i + 1;

        //---------------------------------
        // Initial Position
        //---------------------------------

        cout << "Initial x (m): ";
        cin >> pedestrians[i].pos_x;

        cout << "Initial y (m): ";
        cin >> pedestrians[i].pos_y;

        pedestrians[i].initial_pos_x =
                pedestrians[i].pos_x;

        pedestrians[i].initial_pos_y =
                pedestrians[i].pos_y;

        //---------------------------------
        // Initial Velocity
        //---------------------------------

        cout << "Initial velocity in x (m/s): ";
        cin >> pedestrians[i].vel_x;

        cout << "Initial velocity in y (m/s): ";
        cin >> pedestrians[i].vel_y;

        //---------------------------------
        // Desired Speed
        //---------------------------------

        cout << "Desired speed (Enter 0 for default 1.47 m/s): ";

        cin >> pedestrians[i].desired_speed;

        if(pedestrians[i].desired_speed == 0.0)
        {
            pedestrians[i].desired_speed = 1.47;
        }

        //---------------------------------
        // Final Destination
        //---------------------------------

        cout << "Final goal x (m): ";
        cin >> pedestrians[i].finalGoal_x;

        cout << "Final goal y (m): ";
        cin >> pedestrians[i].finalGoal_y;

        //---------------------------------
        // Initially Variable Goal
        //---------------------------------

        pedestrians[i].Vgoal.x =
                pedestrians[i].finalGoal_x;

        pedestrians[i].Vgoal.y =
                pedestrians[i].finalGoal_y;

        pedestrians[i].Vgoal.sideChosen = -1;

        //---------------------------------
        // Initialize
        //---------------------------------

        initializePedestrianState(
                pedestrians[i]);
    }
        //-----------------------------------------------------
    // VGA Option
    //-----------------------------------------------------

    cout << endl;

    cout << "Variable Goal Algorithm" << endl;
    cout << "0 = Deterministic" << endl;
    cout << "1 = Stochastic" << endl;
    cout << "Choice : ";

    cin >> stochasticVGA;

    //-----------------------------------------------------
    // Simulation time
    //-----------------------------------------------------

    double time = 0.0;

    int iteration = 0;

    //-----------------------------------------------------
    // Main Simulation Loop
    //-----------------------------------------------------

    while(time <= param.Tmax)
    {
        //-------------------------------------------------
        // Check whether everyone has reached destination
        //-------------------------------------------------

        bool simulationFinished = true;

        for(int i=0;i<numberOfPedestrians;i++)
        {
            if(!pedestrians[i].reachedGoal)
            {
                simulationFinished = false;
                break;
            }
        }

        if(simulationFinished)
        {
            break;
        }

        //-------------------------------------------------
        // Loop over pedestrians
        //-------------------------------------------------

        for(int i=0;i<numberOfPedestrians;i++)
        {
            if(pedestrians[i].reachedGoal)
            {
                continue;
            }

            //-------------------------------------------------
            // Build obstacle list
            //-------------------------------------------------

            obstacles.clear();

            //-----------------------------------------
            // Static obstacles
            //-----------------------------------------

            for(int j=0;
    j<(int)staticObstacles.size();
    j++)
            {
                obstacles.push_back(
                        staticObstacles[j]);
            }

            //-----------------------------------------
            // Every other pedestrian is an obstacle
            //-----------------------------------------

            for(int j=0;j<numberOfPedestrians;j++)
            {
                if(j==i)
                {
                    continue;
                }

                Obstacle currentObstacle;

                currentObstacle.id =
                        pedestrians[j].id;

                currentObstacle.x =
                        pedestrians[j].pos_x;

                currentObstacle.y =
                        pedestrians[j].pos_y;

                currentObstacle.radius =
                        pedestrians[j].radius;

                currentObstacle.vel_x =
                        pedestrians[j].vel_x;

                currentObstacle.vel_y =
                        pedestrians[j].vel_y;

                obstacles.push_back(
                        currentObstacle);
                }
           

            //-------------------------------------------------
            // Reset all forces
            //-------------------------------------------------

            resetForces(
                    pedestrians[i]);

            resetAcceleration(
                    pedestrians[i]);

            resetForceMagnitude(
                    pedestrians[i]);

                         //-------------------------------------------------
            // Construct corridor for VGA
            //-------------------------------------------------

            Coordinates direction =
                    normalize(
                            pedestrians[i].finalGoal_x
                            -
                            pedestrians[i].pos_x,
                            pedestrians[i].finalGoal_y
                            -
                            pedestrians[i].pos_y);

            Coordinates perpendicular;

            perpendicular.x =
                    -direction.y;

            perpendicular.y =
                     direction.x;

            double corridorWidth =
                    pedestrians[i].radius;

            Coordinates corridor[4];

            //-----------------------------------------
            // Upper Left
            //-----------------------------------------

            corridor[0].x =
                    pedestrians[i].pos_x
                    +
                    corridorWidth
                    *
                    perpendicular.x;

            corridor[0].y =
                    pedestrians[i].pos_y
                    +
                    corridorWidth
                    *
                    perpendicular.y;

            //-----------------------------------------
            // Upper Right
            //-----------------------------------------

            corridor[1].x =
                    pedestrians[i].finalGoal_x
                    +
                    corridorWidth
                    *
                    perpendicular.x;

            corridor[1].y =
                    pedestrians[i].finalGoal_y
                    +
                    corridorWidth
                    *
                    perpendicular.y;

            //-----------------------------------------
            // Lower Right
            //-----------------------------------------

            corridor[2].x =
                    pedestrians[i].finalGoal_x
                    -
                    corridorWidth
                    *
                    perpendicular.x;

            corridor[2].y =
                    pedestrians[i].finalGoal_y
                    -
                    corridorWidth
                    *
                    perpendicular.y;

            //-----------------------------------------
            // Lower Left
            //-----------------------------------------

            corridor[3].x =
                    pedestrians[i].pos_x
                    -
                    corridorWidth
                    *
                    perpendicular.x;

            corridor[3].y =
                    pedestrians[i].pos_y
                    -
                    corridorWidth
                    *
                    perpendicular.y;

            //-------------------------------------------------
            // Recalculate VGA every 6 iterations
            //-------------------------------------------------

            if(iteration % param.VGAinterval == 0)
            {
                int stuck = 0;

                Coordinates goal =
                        VGA(
                                pedestrians[i],
                                obstacles.data(),
                                (int)obstacles.size(),
                                deg2rad(
                                    param.fieldOfView
                                    /
                                    2.0),
                                corridor,
                                &stuck,
                                stochasticVGA,
                                1,
                                0);

                pedestrians[i].Vgoal.x =
        goal.x;

pedestrians[i].Vgoal.y =
        goal.y;
        

                updateDesiredVelocity(
                        pedestrians[i]);
            }
             //-------------------------------------------------
            // Self Driving Force
            //-------------------------------------------------

            Coordinates Fs =
                    self_force(
                            pedestrians[i],
                            param);

            pedestrians[i].Fs_x =
                    Fs.x;

            pedestrians[i].Fs_y =
                    Fs.y;

            pedestrians[i].Fnew_x +=
                    Fs.x;

            pedestrians[i].Fnew_y +=
                    Fs.y;
                  
                        //-------------------------------------------------
            // UPL Force from all obstacles
            //-------------------------------------------------

            pedestrians[i].Fi_UPL_x = 0.0;
            pedestrians[i].Fi_UPL_y = 0.0;

            for(int j=0;
                j<(int)obstacles.size();
                j++)
            {
                Coordinates Fi =
                        UPL(
                            pedestrians[i],
                            obstacles[j],
                            param);

                pedestrians[i].Fi_UPL_x +=
                        Fi.x;

                pedestrians[i].Fi_UPL_y +=
                        Fi.y;

                pedestrians[i].Fnew_x +=
                        Fi.x;

                pedestrians[i].Fnew_y +=
                        Fi.y;
            }

            //-------------------------------------------------
            // Contact force from static obstacles
            //-------------------------------------------------

            pedestrians[i].Fi_contact_x = 0.0;
            pedestrians[i].Fi_contact_y = 0.0;

            for(int j=0;
                j<(int)obstacles.size();
                j++)
            {
                Coordinates Fc =
                        ContactForce(
                            pedestrians[i],
                            obstacles[j]);

                pedestrians[i].Fi_contact_x +=
                        Fc.x;

                pedestrians[i].Fi_contact_y +=
                        Fc.y;

                pedestrians[i].Fnew_x +=
                        Fc.x;

                pedestrians[i].Fnew_y +=
                        Fc.y;
            }

            //-------------------------------------------------
            // Limit acceleration
            //-------------------------------------------------

            limitAcceleration(
                    pedestrians[i],
                    param);

            updateAcceleration(
                    pedestrians[i]);

            updateForceMagnitude(
                    pedestrians[i]);
                                //-------------------------------------------------
            // Velocity-Verlet Integration
            //-------------------------------------------------

            updatePosition(
                    pedestrians[i],
                    param);

            updateHalfVelocity(
                    pedestrians[i],
                    param);

            updateVelocity(
                    pedestrians[i],
                    param);

            limitSpeed(
                    pedestrians[i]);

            updateOldForce(
                    pedestrians[i]);

            //-------------------------------------------------
            // Update pedestrian quantities
            //-------------------------------------------------

            updateSpeed(
                    pedestrians[i]);

            updateDistanceFromFinalGoal(
                    pedestrians[i]);

            updateDistanceFromVariableGoal(
                    pedestrians[i]);

            checkGoalReached(
                    pedestrians[i]);

            stopPedestrian(
                    pedestrians[i]);

            //-------------------------------------------------
            // Write trajectory
            //-------------------------------------------------

            trajectory
                    << time << ","
                    << pedestrians[i].id << ","
                    << pedestrians[i].pos_x << ","
                    << pedestrians[i].pos_y << ","
                    << pedestrians[i].vel_x << ","
                    << pedestrians[i].vel_y << ","
                    << pedestrians[i].speed << ","
                    << pedestrians[i].Vgoal.x << ","
                    << pedestrians[i].Vgoal.y << ","
                    << pedestrians[i].reachedGoal
                    << endl;
            }
                    //-------------------------------------------------
        // Advance simulation clock
        //-------------------------------------------------

        time += param.dt;

        iteration++;
    }
        trajectory.close();
            cout << endl;

    cout << "----------------------------------------" << endl;
    cout << "Simulation Finished." << endl;
    cout << "Trajectory written to trajectory.csv" << endl;
    cout << "----------------------------------------" << endl;
        return 0;
}
