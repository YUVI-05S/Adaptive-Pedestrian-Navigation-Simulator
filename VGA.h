#ifndef VGA_H
#define VGA_H

#include <iostream>
#include "headerClass.h"
#include "UPL.h"

using namespace std;
//-----------------------------------------------------
// Left / Right Test
//-----------------------------------------------------

int inLeftOrRight(
        Coordinates point0,
        Coordinates point1,
        Coordinates point2)
{
    Coordinates vecA;
    Coordinates vecB;

    vecA.x = point1.x - point0.x;
    vecA.y = point1.y - point0.y;

    vecB.x = point2.x - point0.x;
    vecB.y = point2.y - point0.y;

    double crossProduct =
            vecA.x * vecB.y
            -
            vecA.y * vecB.x;

    if(crossProduct <= 0.0)
    {
        return 0;
    }

    return 1;
}
//-----------------------------------------------------
// Point Inside Rectangle
//-----------------------------------------------------

int pointInsideRectangle(
        Coordinates upperLeft,
        Coordinates upperRight,
        Coordinates lowerRight,
        Coordinates lowerLeft,
        Coordinates point,
        int needIntersectingLine,
        Coordinates* lineVertices,
        int print)
{
    int direction;
    int flag = 0;

    //----------------------------------
    // Side AB
    //----------------------------------

    direction =
            inLeftOrRight(
                    upperLeft,
                    point,
                    upperRight);

    if(print == 1)
    {
        printf(
                "upper left: %f, %f\tpoint: %f, %f\tupper right: %f, %f\n",
                upperLeft.x,
                upperLeft.y,
                point.x,
                point.y,
                upperRight.x,
                upperRight.y);

        cout << "direction 1: "
             << direction
             << endl;
    }

    if(direction == 1)
    {
        //----------------------------------
        // Side CD
        //----------------------------------

        direction =
                inLeftOrRight(
                        lowerRight,
                        point,
                        lowerLeft);

        if(print == 1)
        {
            printf(
                    "lower right: %f, %f\tpoint: %f, %f\tlower left: %f, %f\n",
                    lowerRight.x,
                    lowerRight.y,
                    point.x,
                    point.y,
                    lowerLeft.x,
                    lowerLeft.y);

            cout << "direction 2: "
                 << direction
                 << endl;
        }

        if(direction == 1)
        {
            //----------------------------------
            // Side DA
            //----------------------------------

            direction =
                    inLeftOrRight(
                            lowerLeft,
                            point,
                            upperLeft);

            if(print == 1)
            {
                printf(
                        "lower left: %f, %f\tpoint: %f, %f\tupper left: %f, %f\n",
                        lowerLeft.x,
                        lowerLeft.y,
                        point.x,
                        point.y,
                        upperLeft.x,
                        upperLeft.y);

                cout << "direction 3: "
                     << direction
                     << endl;
            }

            if(direction == 1)
            {
                //----------------------------------
                // Side BC
                //----------------------------------

                direction =
                        inLeftOrRight(
                                upperRight,
                                point,
                                lowerRight);

                if(print == 1)
                {
                    printf(
                            "upper right: %f, %f\tpoint: %f, %f\tlower right: %f, %f\n",
                            upperRight.x,
                            upperRight.y,
                            point.x,
                            point.y,
                            lowerRight.x,
                            lowerRight.y);

                    cout << "direction 4: "
                         << direction
                         << endl;
                }

                if(direction == 1)
                {
                    flag = 1;
                    return flag;
                }

                else
                {
                    if(needIntersectingLine == 1)
                    {
                        lineVertices[0] = lowerRight;
                        lineVertices[1] = upperRight;

                        if(print == 1)
                        {
                            cout << "D to A:" << endl;

                            cout
                                    << "line vertices: ("
                                    << lineVertices[0].x
                                    << ", "
                                    << lineVertices[0].y
                                    << "); ("
                                    << lineVertices[1].x
                                    << ", "
                                    << lineVertices[1].y
                                    << ")\n";
                        }
                    }

                    return flag;
                }
            }

            else
            {
                if(needIntersectingLine == 1)
                {
                    lineVertices[0] = upperLeft;
                    lineVertices[1] = lowerLeft;

                    if(print == 1)
                    {
                        cout << "C to D:" << endl;

                        cout
                                << "line vertices: ("
                                << lineVertices[0].x
                                << ", "
                                << lineVertices[0].y
                                << "); ("
                                << lineVertices[1].x
                                << ", "
                                << lineVertices[1].y
                                << ")\n";
                    }
                }

                return flag;
            }
        }

        else
        {
            if(needIntersectingLine == 1)
            {
                lineVertices[0] = lowerLeft;
                lineVertices[1] = lowerRight;

                if(print == 1)
                {
                    cout << "B to C:" << endl;

                    cout
                            << "line vertices: ("
                            << lineVertices[0].x
                            << ", "
                            << lineVertices[0].y
                            << "); ("
                            << lineVertices[1].x
                            << ", "
                            << lineVertices[1].y
                            << ")\n";
                }
            }

            return flag;
        }
    }

    else
    {
        if(needIntersectingLine == 1)
        {
            lineVertices[0] = upperLeft;
            lineVertices[1] = upperRight;

            if(print == 1)
            {
                cout << "A to B:" << endl;

                cout
                        << "line vertices: ("
                        << lineVertices[0].x
                        << ", "
                        << lineVertices[0].y
                        << "); ("
                        << lineVertices[1].x
                        << ", "
                        << lineVertices[1].y
                        << ")\n";
            }
        }

        return flag;
    }
}
//-----------------------------------------------------
// Obstacle Intersects Path
//-----------------------------------------------------

int intersection(
        Pedestrian ped,
        Obstacle obst,
        int print)
{
    //----------------------------------
    // Path direction
    //----------------------------------

    Coordinates v;
    Coordinates perp_v;

    v.x = ped.Vgoal.x - ped.pos_x;
    v.y = ped.Vgoal.y - ped.pos_y;

    v.mod =
            sqrt(
                v.x * v.x +
                v.y * v.y);

    perp_v.x = -v.y / v.mod;
    perp_v.y =  v.x / v.mod;

    //----------------------------------
    // Corridor vertices
    //----------------------------------

    Coordinates A;
    Coordinates B;
    Coordinates C;
    Coordinates D;

    A.x =
            ped.pos_x +
            (ped.radius + obst.radius) * perp_v.x;

    A.y =
            ped.pos_y +
            (ped.radius + obst.radius) * perp_v.y;

    B.x =
            ped.pos_x -
            (ped.radius + obst.radius) * perp_v.x;

    B.y =
            ped.pos_y -
            (ped.radius + obst.radius) * perp_v.y;

    C.x =
            ped.Vgoal.x +
            (ped.radius + obst.radius) * perp_v.x;

    C.y =
            ped.Vgoal.y +
            (ped.radius + obst.radius) * perp_v.y;

    D.x =
            ped.Vgoal.x -
            (ped.radius + obst.radius) * perp_v.x;

    D.y =
            ped.Vgoal.y -
            (ped.radius + obst.radius) * perp_v.y;

    //----------------------------------
    // Rectangle orientation
    //----------------------------------

    Coordinates upperLeft;
    Coordinates upperRight;
    Coordinates lowerRight;
    Coordinates lowerLeft;

    if((v.x >= 0 && v.y >= 0) ||
       (v.x >= 0 && v.y <= 0))
    {
        upperLeft  = A;
        upperRight = C;
        lowerLeft  = B;
        lowerRight = D;
    }

    if((v.x <= 0 && v.y >= 0) ||
       (v.x <= 0 && v.y <= 0))
    {
        upperLeft  = D;
        upperRight = B;
        lowerLeft  = C;
        lowerRight = A;
    }

    //----------------------------------
    // Debug
    //----------------------------------

    if(print == 1)
    {
        printf(
                "v.x: %f\tv.y: %f\tv.mod: %f\n"
                "perp_v.x: %f\n"
                "perp_v.y: %f\n",
                v.x,
                v.y,
                v.mod,
                perp_v.x,
                perp_v.y);

        printf(
                "UL_x: %f\tUL_y: %f\t"
                "UR_x: %f\tUR_y: %f\n"
                "LL_x: %f\tLL_y: %f\t"
                "LR_x: %f\tLR_y: %f\n",
                upperLeft.x,
                upperLeft.y,
                upperRight.x,
                upperRight.y,
                lowerLeft.x,
                lowerLeft.y,
                lowerRight.x,
                lowerRight.y);
    }

    //----------------------------------
    // Obstacle position
    //----------------------------------

    Coordinates point;
    Coordinates lineVertices[2];

    point.x = obst.x;
    point.y = obst.y;

    //----------------------------------
    // Inside corridor?
    //----------------------------------

    return pointInsideRectangle(
                upperLeft,
                upperRight,
                lowerRight,
                lowerLeft,
                point,
                0,
                lineVertices,
                0);
}
//-----------------------------------------------------
// Nearest Obstacle
//-----------------------------------------------------

int nearestObst(
        Pedestrian ped,
        Obstacle obst[],
        int noOfObst,
        double angleOfSight,
        int* convergence,
        int previousNearestObstacle,
        int print)
{
    //----------------------------------
    // Initialization
    //----------------------------------

    int nearestObstacle = -1;

    double nearestDistance =
            2.0 *
            sqrt(
                    (ped.Vgoal.x - ped.pos_x) *
                    (ped.Vgoal.x - ped.pos_x)
                    +
                    (ped.Vgoal.y - ped.pos_y) *
                    (ped.Vgoal.y - ped.pos_y));

    if(print == 1)
    {
        cout
                << "ped position: "
                << ped.pos_x
                << ", "
                << ped.pos_y
                << endl;

        cout
                << "ped goal: "
                << ped.Vgoal.x
                << ", "
                << ped.Vgoal.y
                << endl;

        cout
                << "initial distance: "
                << nearestDistance
                << endl;
    }

    //----------------------------------
    // Search all obstacles
    //----------------------------------

    for(int i = 0; i < noOfObst; i++)
    {
        if(print == 1)
        {
            cout
                    << "Obstacle "
                    << i
                    << endl;
        }

        int obstacleBetweenPath =
                intersection(
                        ped,
                        obst[i],
                        0);

        // Original code temporarily assumes
        // every obstacle is visible.

        int inSight = 1;

        if(print == 1)
        {
            cout
                    << "Between Path : "
                    << obstacleBetweenPath
                    << "\tIn Sight : "
                    << inSight
                    << endl;
        }

        if(obstacleBetweenPath == 1 &&
           inSight == 1)
        {
            double distance =
                    sqrt(
                            (ped.pos_x - obst[i].x) *
                            (ped.pos_x - obst[i].x)
                            +
                            (ped.pos_y - obst[i].y) *
                            (ped.pos_y - obst[i].y));

            if(print == 1)
            {
                cout
                        << "distance : "
                        << distance
                        << endl;
            }

            if(distance <= nearestDistance)
            {
                nearestObstacle = i;
                nearestDistance = distance;
            }

            if(print == 1)
            {
                cout
                        << "nearest obstacle : "
                        << nearestObstacle
                        << endl;
            }
        }
    }

    //----------------------------------
    // Convergence
    //----------------------------------

    if(nearestObstacle == previousNearestObstacle ||
       nearestObstacle == -1)
    {
        *convergence = 1;
    }
    else
    {
        *convergence = 0;
    }

    return nearestObstacle;
}
//-----------------------------------------------------
// Cluster Formation (Breadth First Search)
//-----------------------------------------------------

int ClusterFormation(
        int* cluster,
        int nearest_obst,
        Obstacle obst[],
        int noOfObst,
        double angleOfSight,
        Pedestrian ped,
        double timesAway_cluster)
{
    int k;
    int inSight;
    int flag;
    int clusterLength;

    double distance;

    //----------------------------------
    // Initialize cluster
    //----------------------------------

    cluster[0] = nearest_obst;
    clusterLength = 1;

    //----------------------------------
    // Breadth First Search
    //----------------------------------

    k = 0;

    do
    {
        for(int i = 0; i < noOfObst; i++)
        {
            //----------------------------------
            // Visibility check (currently disabled)
            //----------------------------------

            // inSight =
            //     obstInSight(
            //             ped,
            //             angleOfSight,
            //             obst[i].x,
            //             obst[i].y,
            //             obst[i].radius,
            //             0);
            //
            // if(inSight == 0)
            //     continue;

            //----------------------------------
            // Already in cluster?
            //----------------------------------

            flag = 0;

            for(int j = 0; j < clusterLength; j++)
            {
                if(cluster[j] == i)
                {
                    flag = 1;
                    break;
                }
            }

            if(flag == 1)
                continue;

            //----------------------------------
            // Distance from current cluster node
            //----------------------------------

            distance =
                    sqrt(
                        (obst[i].x - obst[cluster[k]].x) *
                        (obst[i].x - obst[cluster[k]].x)
                        +
                        (obst[i].y - obst[cluster[k]].y) *
                        (obst[i].y - obst[cluster[k]].y));

            //----------------------------------
            // Add obstacle to cluster
            //----------------------------------

            if(distance <
               0.5 +
               timesAway_cluster *
               2.0 *
               ped.radius)
            {
                cluster[clusterLength] = i;
                clusterLength++;
            }
        }

        k++;

    } while(k < clusterLength);

    return clusterLength;
}
//-----------------------------------------------------
// Convex Hull
//-----------------------------------------------------

int ConvexHull(
        int* convexHull,
        int cluster[],
        int lengthOfCluster,
        Obstacle obst[])
{
    int lowerMostObst;
    int sortedArray[lengthOfCluster];
    int sortedArrayLength;
    int first_no;
    int k0;

    int convexHullLength;
    int convexHullCopy[lengthOfCluster];
    int final[lengthOfCluster];

    int l;
    int k;

    int y_min;
    int y_max;
    int x_min;
    int x_max;

    int crossProduct[3];

    int a;
    int b;
    int c0;
    int flag;
    int direction;

    double pi = 3.14159265;

    //----------------------------------
    // Step 1 : Lower-most obstacle
    //----------------------------------

    lowerMostObst = cluster[0];

    for(int i = 1; i < lengthOfCluster; i++)
    {
        if(obst[cluster[i]].y <
           obst[lowerMostObst].y)
        {
            lowerMostObst = cluster[i];
        }
    }

    //----------------------------------
    // Step 2 : Angle calculation
    //----------------------------------

    for(int i = 0; i < lengthOfCluster; i++)
    {
        if(cluster[i] == lowerMostObst)
            continue;

        obst[cluster[i]].theta =
                atan(
                    (obst[lowerMostObst].y -
                     obst[cluster[i]].y)
                    /
                    (obst[lowerMostObst].x -
                     obst[cluster[i]].x));

        if(obst[cluster[i]].theta < 0)
            obst[cluster[i]].theta += pi;
    }

    //----------------------------------
    // Step 3 : Sort by angle
    //----------------------------------

    sortedArrayLength = 1;

    sortedArray[0] = lowerMostObst;

    sortedArray[1] =
            (lowerMostObst == cluster[0])
            ?
            cluster[1]
            :
            cluster[0];

    Coordinates point[3];

    //----------------------------------
    // Convex hull for >=3 points
    //----------------------------------

    if(lengthOfCluster > 2)
    {
        first_no = sortedArray[1];

        //----------------------------------
        // Insertion sort
        //----------------------------------

        for(int i = 1; i < lengthOfCluster; i++)
        {
            if(cluster[i] == lowerMostObst ||
               cluster[i] == first_no)
            {
                continue;
            }

            k0 = sortedArrayLength;
            sortedArrayLength++;

            if(obst[sortedArray[k0]].theta >
               obst[cluster[i]].theta)
            {
                while(obst[sortedArray[k0]].theta >
                      obst[cluster[i]].theta &&
                      k0 > 0)
                {
                    sortedArray[k0 + 1] =
                            sortedArray[k0];

                    sortedArray[k0] =
                            cluster[i];

                    k0--;
                }
            }
            else
            {
                sortedArray[k0 + 1] =
                        cluster[i];
            }
        }

        sortedArray[sortedArrayLength + 1] =
                lowerMostObst;

        //----------------------------------
        // Extreme points
        //----------------------------------

        y_min = lowerMostObst;

        x_min = sortedArray[0];

        for(int i = 1; i <= sortedArrayLength; i++)
        {
            if(obst[sortedArray[i]].x <
               obst[x_min].x)
            {
                x_min = sortedArray[i];
            }
        }

        x_max = sortedArray[0];

        for(int i = 1; i <= sortedArrayLength; i++)
        {
            if(obst[sortedArray[i]].x >
               obst[x_max].x)
            {
                x_max = sortedArray[i];
            }
        }

        y_max = sortedArray[0];

        for(int i = 1; i <= sortedArrayLength; i++)
        {
            if(obst[sortedArray[i]].y >
               obst[y_max].y)
            {
                y_max = sortedArray[i];
            }
        }

        //----------------------------------
        // Initial hull copy
        //----------------------------------

        l = 0;

        for(int i = 0; i <= sortedArrayLength; i++)
        {
            if(sortedArray[i] == x_max ||
               sortedArray[i] == x_min ||
               sortedArray[i] == y_max ||
               sortedArray[i] == y_min)
            {
                convexHullCopy[l] =
                        sortedArray[i];

                l++;
            }
        }

        convexHullCopy[l] =
                lowerMostObst;

        //----------------------------------
        // Remaining points
        //----------------------------------

        convexHullLength = 0;
        k = 0;

        for(int i = 0; i < l; i++)
        {
            a = 0;

            while(sortedArray[k] !=
                  convexHullCopy[i + 1])
            {
                final[a] = sortedArray[k];
                a++;
                k++;
            }

            final[a] = sortedArray[k];

            do
            {
                flag = 0;
                b = 1;

                while(b != a &&
                      b - 1 != a)
                {
                    crossProduct[0] =
                            final[b - 1];

                    crossProduct[1] =
                            final[b];

                    crossProduct[2] =
                            final[b + 1];

                    point[0].x =
                            obst[crossProduct[0]].x;
                    point[0].y =
                            obst[crossProduct[0]].y;

                    point[1].x =
                            obst[crossProduct[1]].x;
                    point[1].y =
                            obst[crossProduct[1]].y;

                    point[2].x =
                            obst[crossProduct[2]].x;
                    point[2].y =
                            obst[crossProduct[2]].y;

                    direction =
                            inLeftOrRight(
                                    point[0],
                                    point[1],
                                    point[2]);

                    if(direction == 0)
                    {
                        flag = 1;

                        c0 = b;

                        while(c0 != a)
                        {
                            final[c0] =
                                    final[c0 + 1];

                            c0++;
                        }

                        a--;
                    }

                    b++;
                }

            } while(flag == 1);

            //----------------------------------
            // Add hull points
            //----------------------------------

            for(int j = 0; j < a; j++)
            {
                convexHull[convexHullLength] =
                        final[j];

                convexHullLength++;
            }
        }

        convexHullLength--;
    }
    else
    {
        if(lengthOfCluster == 2)
        {
            convexHullLength = 1;

            convexHull[0] = sortedArray[0];
            convexHull[1] = sortedArray[1];
        }
        else
        {
            convexHullLength = 0;

            convexHull[0] = sortedArray[0];
        }
    }

    return convexHullLength;
}
//-----------------------------------------------------
// Distance From Corridor
//-----------------------------------------------------

int DistFromCorridor(
        Pedestrian ped,
        Coordinates obst,
        double obst_radius,
        Coordinates upperLeft,
        Coordinates upperRight,
        Coordinates lowerRight,
        Coordinates lowerLeft,
        int leftOrRight,
        int print)
{
    //----------------------------------
    // Lateral distance from corridor
    //----------------------------------

    double d_min1;
    double d_min2;
    double d_min3;
    double d_min4;

    double A;
    double B;
    double C;

    int sideNo;
    int direction;
    int near;

    //----------------------------------
    // Determine corridor side
    //----------------------------------

    Coordinates points[3];

    points[0].x = ped.initial_pos_x;
    points[0].y = ped.initial_pos_y;

    points[1].x = upperLeft.x;
    points[1].y = upperLeft.y;

    points[2].x = ped.finalGoal_x;
    points[2].y = ped.finalGoal_y;

    direction =
            inLeftOrRight(
                    points[0],
                    points[1],
                    points[2]);

    if(leftOrRight == direction)
        sideNo = 1;
    else
        sideNo = 3;

    //----------------------------------
    // Side 1
    //----------------------------------

    if(sideNo == 1)
    {
        A =
                upperRight.y
                -
                upperLeft.y;

        B =
                -1.0
                *
                (
                    upperRight.x
                    -
                    upperLeft.x
                );

        C =
                (upperRight.x-upperLeft.x)
                *
                upperLeft.y
                -
                (upperRight.y-upperLeft.y)
                *
                upperLeft.x;

        d_min1 =
                (A*obst.x + B*obst.y + C)
                /
                sqrt(A*A + B*B);

        if(d_min1 < 0.0)
            d_min1 *= -1.0;

        if(print == 1)
        {
            printf(
                    "Obst: %f, %f\n",
                    obst.x,
                    obst.y);

            printf(
                    "leftOrRight: %d\t"
                    "side no: %d\n"
                    "d min: %f\t"
                    "(obst_radius + 2*ped_radius): %f\n",
                    leftOrRight,
                    sideNo,
                    d_min1,
                    obst_radius + 2.0*ped.radius);
        }

        if(d_min1 <
           (obst_radius + 2.0*ped.radius))
        {
            sideNo = 1;
            return sideNo;
        }
        else
        {
            sideNo = -1;
        }
    }

    //----------------------------------
    // Side 3
    //----------------------------------

    else
    {
        A =
                lowerRight.y
                -
                lowerLeft.y;

        B =
                -1.0
                *
                (
                    lowerRight.x
                    -
                    lowerLeft.x
                );

        C =
                (lowerRight.x-lowerLeft.x)
                *
                lowerLeft.y
                -
                (lowerRight.y-lowerLeft.y)
                *
                lowerLeft.x;

        d_min3 =
                (A*obst.x + B*obst.y + C)
                /
                sqrt(A*A + B*B);

        if(d_min3 < 0.0)
            d_min3 *= -1.0;

        if(print == 1)
        {
            printf(
                    "Obst: %f, %f\n",
                    obst.x,
                    obst.y);

            printf(
                    "leftOrRight: %d\t"
                    "side no: %d\n"
                    "d min: %f\t"
                    "(obst_radius + 2*ped_radius): %f\n",
                    leftOrRight,
                    sideNo,
                    d_min3,
                    obst_radius + 2.0*ped.radius);
        }

        if(d_min3 <
           (obst_radius + 2.0*ped.radius))
        {
            sideNo = 3;
            return sideNo;
        }
        else
        {
            sideNo = -1;
        }
    }

    return sideNo;
}
//-----------------------------------------------------
// Point In Sight
//-----------------------------------------------------

int pointInSight(
        Coordinates obst,
        Pedestrian ped,
        double angleOfSight,
        int print)
{
    //----------------------------------
    // Pedestrian at rest
    //----------------------------------

    if(ped.vel_x == 0.0 &&
       ped.vel_y == 0.0)
    {
        return 1;
    }

    //----------------------------------
    // Vector from pedestrian to point
    //----------------------------------

    double vectorObst_x =
            obst.x
            -
            ped.pos_x;

    double vectorObst_y =
            obst.y
            -
            ped.pos_y;

    //----------------------------------
    // Dot product
    //----------------------------------

    double dotProduct =
            vectorObst_x * ped.vel_x
            +
            vectorObst_y * ped.vel_y;

    double mod_vectorObst =
            sqrt(
                vectorObst_x * vectorObst_x
                +
                vectorObst_y * vectorObst_y);

    double mod_vel =
            sqrt(
                ped.vel_x * ped.vel_x
                +
                ped.vel_y * ped.vel_y);

    //----------------------------------
    // Cosine
    //----------------------------------

    double cosine =
            dotProduct
            /
            mod_vel
            /
            mod_vectorObst;

    if(print == 1)
    {
        cout
                << "obst: "
                << obst.x
                << "\t"
                << obst.y
                << endl;

        cout
                << "ped: "
                << ped.pos_x
                << "\t"
                << ped.pos_y
                << endl;

        cout
                << "vel: "
                << ped.vel_x
                << "\t"
                << ped.vel_y
                << endl;

        printf(
                "vectorObst: %f, %f\t"
                "dot product: %f\n"
                "mod vectorObst: %f\t"
                "mod vel: %f\n",
                vectorObst_x,
                vectorObst_y,
                dotProduct,
                mod_vectorObst,
                mod_vel);

        cout
                << "cosine: "
                << cosine
                << "\tcos(angleOfSight): "
                << cos(angleOfSight)
                << endl;
    }

    //----------------------------------
    // Inside field of view
    //----------------------------------

    if(cosine >= cos(angleOfSight) &&
       cosine <= 1.0)
    {
        return 1;
    }

    return 0;
}
//-----------------------------------------------------
// Obstacle In Sight
//-----------------------------------------------------

int obstInSight(
        Pedestrian ped,
        double angleOfSight,
        Coordinates obst,
        double obst_radius,
        int print)
{
    //----------------------------------
    // Boundary line angles
    //----------------------------------

    double theta1;
    double theta2;

    double slope1;
    double slope2;

    double pSlope1;
    double pSlope2;

    theta1 =
            atan(
                ped.vel_y /
                ped.vel_x)
            +
            angleOfSight;

    theta2 =
            atan(
                ped.vel_y /
                ped.vel_x)
            -
            angleOfSight;

    slope1 = tan(theta1);
    slope2 = tan(theta2);

    pSlope1 = -1.0 / slope1;
    pSlope2 = -1.0 / slope2;

    //----------------------------------
    // Tangent points on obstacle
    //----------------------------------

    Coordinates p[4];

    p[0].x =
            obst.x
            +
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope1 * pSlope1));

    p[1].x =
            obst.x
            -
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope1 * pSlope1));

    p[0].y =
            obst.y
            +
            pSlope1
            *
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope1 * pSlope1));

    p[1].y =
            obst.y
            -
            pSlope1
            *
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope1 * pSlope1));

    p[2].x =
            obst.x
            +
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope2 * pSlope2));

    p[3].x =
            obst.x
            -
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope2 * pSlope2));

    p[2].y =
            obst.y
            +
            pSlope2
            *
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope2 * pSlope2));

    p[3].y =
            obst.y
            -
            pSlope2
            *
            sqrt(
                obst_radius * obst_radius
                /
                (1.0 + pSlope2 * pSlope2));

    //----------------------------------
    // Visibility test
    //----------------------------------

    int inSight;

    if(ped.vel_x == 0.0 &&
       ped.vel_y == 0.0)
    {
        inSight = 1;
    }
    else
    {
        inSight =
                pointInSight(
                        p[0],
                        ped,
                        angleOfSight,
                        0);
    }

    if(inSight == 0)
    {
        if(ped.vel_x == 0.0 &&
           ped.vel_y == 0.0)
        {
            inSight = 1;
        }
        else
        {
            inSight =
                    pointInSight(
                            p[1],
                            ped,
                            angleOfSight,
                            0);
        }

        if(inSight == 0)
        {
            if(ped.vel_x == 0.0 &&
               ped.vel_y == 0.0)
            {
                inSight = 1;
            }
            else
            {
                inSight =
                        pointInSight(
                                p[2],
                                ped,
                                angleOfSight,
                                0);
            }

            if(inSight == 0)
            {
                if(ped.vel_x == 0.0 &&
                   ped.vel_y == 0.0)
                {
                    inSight = 1;
                }
                else
                {
                    inSight =
                            pointInSight(
                                    p[3],
                                    ped,
                                    angleOfSight,
                                    0);
                }
            }
        }
    }

    //----------------------------------
    // Debug
    //----------------------------------

    if(print == 1)
    {
        cout
                << "Obst ("
                << obst.x
                << ", "
                << obst.y
                << ") in sight is: "
                << inSight
                << endl;
    }

    return inSight;
}
//-----------------------------------------------------
// Variable Goal
//-----------------------------------------------------

Coordinates VariableGoal(
        Pedestrian ped,
        Coordinates TEleft,
        Coordinates TEright,
        double obst_radius,
        double angleOfSight,
        Coordinates vertex[],
        double randNo,
        Coordinates nearestObstacle,
        int print,
        int addProbability,
        int clusterDirection,
        int* stuck,
        int* direct)
{
    Coordinates Vgoal[3];
    Coordinates v;
    Coordinates perp_v;
    Coordinates point0;

double mod_a;
double mod_b;
double dotProduct;

double cos_alpha[2];
double d[2];

int inSight[2];
int quadrant[2];

    double dist_of_ped_from_Vgoal[2];
    double dist_of_finalGoal_from_VgoalR;
    double dist_of_finalGoal_from_VgoalL;
    double total_dist[2];

    double pi = 3.14159265;

    int direction;
    int sideNo;
    int flag[2];

TemporaryElementLeft:

    //----------------------------------
    // Check left temporary element
    //----------------------------------

    sideNo =
            DistFromCorridor(
                    ped,
                    TEleft,
                    obst_radius,
                    vertex[0],
                    vertex[1],
                    vertex[2],
                    vertex[3],
                    0,
                    0);

    if(sideNo != -1)
    {
        flag[0] = 0;
        goto TemporaryElementRight;
    }
    else
    {
        flag[0] = 1;
    }

    //----------------------------------
    // Vector towards TEleft
    //----------------------------------

    v.x = TEleft.x - ped.pos_x;
    v.y = TEleft.y - ped.pos_y;

    v.mod =
            sqrt(
                v.x*v.x +
                v.y*v.y);

    perp_v.x = -v.y / v.mod;
    perp_v.y =  v.x / v.mod;

    //----------------------------------
    // First candidate
    //----------------------------------

    Vgoal[0].x =
            TEleft.x +
            (ped.radius + obst_radius) *
            perp_v.x;

    Vgoal[0].y =
            TEleft.y +
            (ped.radius + obst_radius) *
            perp_v.y;

    point0.x = ped.initial_pos_x;
    point0.y = ped.initial_pos_y;

    direction =
            inLeftOrRight(
                    point0,
                    Vgoal[0],
                    TEleft);

    if(direction == 1)
    {
        Vgoal[0].x =
                TEleft.x -
                (ped.radius + obst_radius) *
                perp_v.x;

        Vgoal[0].y =
                TEleft.y -
                (ped.radius + obst_radius) *
                perp_v.y;
    }

    //----------------------------------
    // Distance calculations
    //----------------------------------

    dist_of_ped_from_Vgoal[0] =
            sqrt(
                (ped.pos_x - Vgoal[0].x) *
                (ped.pos_x - Vgoal[0].x)
                +
                (ped.pos_y - Vgoal[0].y) *
                (ped.pos_y - Vgoal[0].y));

    dist_of_finalGoal_from_VgoalL =
            sqrt(
                (ped.finalGoal_x - Vgoal[0].x) *
                (ped.finalGoal_x - Vgoal[0].x)
                +
                (ped.finalGoal_y - Vgoal[0].y) *
                (ped.finalGoal_y - Vgoal[0].y));

    total_dist[0] =
            dist_of_ped_from_Vgoal[0]
            +
            dist_of_finalGoal_from_VgoalL;

    //----------------------------------
    // Deviation
    //----------------------------------

    mod_a =
            sqrt(
                (ped.finalGoal_x - ped.pos_x) *
                (ped.finalGoal_x - ped.pos_x)
                +
                (ped.finalGoal_y - ped.pos_y) *
                (ped.finalGoal_y - ped.pos_y));

    mod_b =
            sqrt(
                (Vgoal[0].x - ped.pos_x) *
                (Vgoal[0].x - ped.pos_x)
                +
                (Vgoal[0].y - ped.pos_y) *
                (Vgoal[0].y - ped.pos_y));

    dotProduct =
            (ped.finalGoal_x - ped.pos_x) *
            (Vgoal[0].x - ped.pos_x)
            +
            (ped.finalGoal_y - ped.pos_y) *
            (Vgoal[0].y - ped.pos_y);

    cos_alpha[0] =
            trunc(
                10000 *
                dotProduct /
                mod_a /
                mod_b)
            /
            10000;

    d[0] =
            dist_of_ped_from_Vgoal[0]
            *
            sqrt(
                1 -
                cos_alpha[0] *
                cos_alpha[0]);

    inSight[0] =
            obstInSight(
                    ped,
                    angleOfSight,
                    Vgoal[0],
                    ped.radius,
                    0);

    //----------------------------------
    // Quadrant preparation
    //----------------------------------
    mod_a =
            sqrt(
                (ped.finalGoal_x - ped.initial_pos_x) *
                (ped.finalGoal_x - ped.initial_pos_x)
                +
                (ped.finalGoal_y - ped.initial_pos_y) *
                (ped.finalGoal_y - ped.initial_pos_y));

    mod_b =
            sqrt(
                (Vgoal[0].x - ped.pos_x) *
                (Vgoal[0].x - ped.pos_x)
                +
                (Vgoal[0].y - ped.pos_y) *
                (Vgoal[0].y - ped.pos_y));

    dotProduct =
            (ped.finalGoal_x - ped.pos_x) *
            (Vgoal[0].x - ped.pos_x)
            +
            (ped.finalGoal_y - ped.pos_y) *
            (Vgoal[0].y - ped.pos_y);

    cos_alpha[0] =
            trunc(
                10000 *
                dotProduct /
                mod_a /
                mod_b)
            /
            10000;

    //----------------------------------
    // Continue with right temporary element
    //----------------------------------

//----------------------------------
// Temporary Element Right
//----------------------------------

TemporaryElementRight:

//----------------------------------
// Check gap between TEright
// and corridor wall
//----------------------------------

sideNo =
        DistFromCorridor(
                ped,
                TEright,
                obst_radius,
                vertex[0],
                vertex[1],
                vertex[2],
                vertex[3],
                1,
                0);

if(sideNo != -1)
{
    flag[1] = 0;
    goto choosingOneOfTwoVgoals;
}
else
{
    flag[1] = 1;
}

//----------------------------------
// Vector towards TEright
//----------------------------------

v.x = TEright.x - ped.pos_x;
v.y = TEright.y - ped.pos_y;

v.mod =
        sqrt(
            v.x*v.x +
            v.y*v.y);

perp_v.x =  v.y / v.mod;
perp_v.y = -v.x / v.mod;

//----------------------------------
// First right candidate
//----------------------------------

Vgoal[1].x =
        TEright.x +
        (ped.radius + obst_radius) *
        perp_v.x;

Vgoal[1].y =
        TEright.y +
        (ped.radius + obst_radius) *
        perp_v.y;

point0.x = ped.pos_x;
point0.y = ped.pos_y;

direction =
        inLeftOrRight(
                point0,
                Vgoal[1],
                TEright);

if(direction == 0)
{
    Vgoal[1].x =
            TEright.x -
            (ped.radius + obst_radius) *
            perp_v.x;

    Vgoal[1].y =
            TEright.y -
            (ped.radius + obst_radius) *
            perp_v.y;
}

//----------------------------------
// Distance calculations
//----------------------------------

dist_of_ped_from_Vgoal[1] =
        sqrt(
            (ped.pos_x - Vgoal[1].x) *
            (ped.pos_x - Vgoal[1].x)
            +
            (ped.pos_y - Vgoal[1].y) *
            (ped.pos_y - Vgoal[1].y));

dist_of_finalGoal_from_VgoalR =
        sqrt(
            (ped.finalGoal_x - Vgoal[1].x) *
            (ped.finalGoal_x - Vgoal[1].x)
            +
            (ped.finalGoal_y - Vgoal[1].y) *
            (ped.finalGoal_y - Vgoal[1].y));

total_dist[1] =
        dist_of_ped_from_Vgoal[1]
        +
        dist_of_finalGoal_from_VgoalR;

//----------------------------------
// Deviation
//----------------------------------

mod_a =
        sqrt(
            (ped.finalGoal_x - ped.pos_x) *
            (ped.finalGoal_x - ped.pos_x)
            +
            (ped.finalGoal_y - ped.pos_y) *
            (ped.finalGoal_y - ped.pos_y));

mod_b =
        sqrt(
            (Vgoal[1].x - ped.pos_x) *
            (Vgoal[1].x - ped.pos_x)
            +
            (Vgoal[1].y - ped.pos_y) *
            (Vgoal[1].y - ped.pos_y));

dotProduct =
        (ped.finalGoal_x - ped.pos_x) *
        (Vgoal[1].x - ped.pos_x)
        +
        (ped.finalGoal_y - ped.pos_y) *
        (Vgoal[1].y - ped.pos_y);

cos_alpha[1] =
        trunc(
            10000 *
            dotProduct /
            mod_a /
            mod_b)
        /
        10000;

d[1] =
        dist_of_ped_from_Vgoal[1]
        *
        sqrt(
            1 -
            cos_alpha[1] *
            cos_alpha[1]);

inSight[1] =
        obstInSight(
                ped,
                angleOfSight,
                Vgoal[1],
                ped.radius,
                0);

//----------------------------------
// Quadrant preparation
//----------------------------------

mod_a =
        sqrt(
            (ped.finalGoal_x - ped.initial_pos_x) *
            (ped.finalGoal_x - ped.initial_pos_x)
            +
            (ped.finalGoal_y - ped.initial_pos_y) *
            (ped.finalGoal_y - ped.initial_pos_y));

mod_b =
        sqrt(
            (Vgoal[1].x - ped.pos_x) *
            (Vgoal[1].x - ped.pos_x)
            +
            (Vgoal[1].y - ped.pos_y) *
            (Vgoal[1].y - ped.pos_y));

dotProduct =
        (ped.finalGoal_x - ped.pos_x) *
        (Vgoal[1].x - ped.pos_x)
        +
        (ped.finalGoal_y - ped.pos_y) *
        (Vgoal[1].y - ped.pos_y);

cos_alpha[1] =
        trunc(
            10000 *
            dotProduct /
            mod_a /
            mod_b)
        /
        10000;
        //----------------------------------
// Probability
//----------------------------------

double prob[2];
double diff;
double expTerm[2];
double d_ref;
double n;

int choice;

n = 3.6056;

d_ref =
        sqrt(
            (ped.pos_x - nearestObstacle.x) *
            (ped.pos_x - nearestObstacle.x)
            +
            (ped.pos_y - nearestObstacle.y) *
            (ped.pos_y - nearestObstacle.y));

expTerm[0] =
        exp(
            -n *
            d[0]
            /
            d_ref);

expTerm[1] =
        exp(
            -n *
            d[1]
            /
            d_ref);

prob[0] =
        expTerm[0]
        /
        (expTerm[0] + expTerm[1]);

prob[1] =
        expTerm[1]
        /
        (expTerm[0] + expTerm[1]);

diff =
        sqrt(
            (prob[0] - prob[1]) *
            (prob[0] - prob[1]));

if(diff < 1.2)
{
    if(prob[0] > prob[1])
    {
        if(randNo <= prob[1])
        {
            choice = 1;
        }
        else
        {
            choice = 0;
        }
    }

    else if(prob[0] < prob[1])
    {
        if(randNo <= prob[0])
        {
            choice = 0;
        }
        else
        {
            choice = 1;
        }
    }

    else
    {
        choice = 0;
    }
}
else
{
    if(d[0] <= d[1])
    {
        choice = 0;
    }
    else
    {
        choice = 1;
    }
}

//----------------------------------
// Choose one of two Variable Goals
//----------------------------------

choosingOneOfTwoVgoals:
//----------------------------------
// Select Variable Goal
//----------------------------------

double backwardsAngle =
        120.0 * pi / 180.0;

double distance =
        total_dist[0] * 100.0;

int goal;

if(flag[0] == 1 &&
   flag[1] == 1)
{
    if(print == 1)
    {
        cout << "Both TEleft and TEright have enough gap from corridor." << endl;
    }

    if(cos_alpha[0] > cos(backwardsAngle) &&
       cos_alpha[1] > cos(backwardsAngle))
    {
        if(print == 1)
        {
            cout << "VGleft and VGright are within -100 to 100 degrees of angles." << endl;
        }

        if((inSight[0] == 1 && inSight[1] == 1) ||
           (inSight[0] == 0 && inSight[1] == 0))
        {
            if(print == 1)
            {
                cout << "Both variable goals are either insight or out of sight." << endl;
            }

            //----------------------------------
            // Probability selection
            //----------------------------------

            if(addProbability == 1)
            {
                if(clusterDirection == -1)
                {
                    if(choice == 0)
                    {
                        Vgoal[2] = Vgoal[0];
                        goal = 0;
                    }
                    else
                    {
                        Vgoal[2] = Vgoal[1];
                        goal = 1;
                    }

                    if(print == 1)
                    {
                        cout
                                << "Cluster direction is not assigned yet.\trandom no: "
                                << randNo
                                << "\nTEleft ("
                                << TEleft.x
                                << ", "
                                << TEleft.y
                                << ") probability: "
                                << prob[0]
                                << "\nTEright ("
                                << TEright.x
                                << ", "
                                << TEright.y
                                << ") probability: "
                                << prob[1]
                                << "\nVgoal: "
                                << Vgoal[2].x
                                << ", "
                                << Vgoal[2].y
                                << endl;
                    }
                }
                else
                {
                    if(clusterDirection == 0)
                    {
                        Vgoal[2] = Vgoal[0];
                        goal = 0;

                        if(print == 1)
                        {
                            cout << "direction is fixed towards left.\t";
                        }
                    }
                    else if(clusterDirection == 1)
                    {
                        Vgoal[2] = Vgoal[1];
                        goal = 1;

                        if(print == 1)
                        {
                            cout << "direction is fixed towards right.\t";
                        }
                    }
                    else
                    {
                        cout << "CLUSTER DIRECTION NEITHER 0 OR 1 NOR IT'S -1" << endl;
                    }

                    if(print == 1)
                    {
                        cout
                                << "TEleft ("
                                << TEleft.x
                                << ", "
                                << TEleft.y
                                << "\tTEright ("
                                << TEright.x
                                << ", "
                                << TEright.y
                                << ")\nVgoal: "
                                << Vgoal[2].x
                                << ", "
                                << Vgoal[2].y
                                << endl;
                    }
                }
            }

            //----------------------------------
            // Deviation selection
            //----------------------------------

            else
            {
                if(d[0] <= d[1])
                {
                    Vgoal[2] = Vgoal[0];
                    goal = 0;
                }
                else
                {
                    Vgoal[2] = Vgoal[1];
                    goal = 1;
                }

                if(print == 1)
                {
                    cout
                            << "TEleft ("
                            << TEleft.x
                            << ", "
                            << TEleft.y
                            << ") deviation: "
                            << d[0]
                            << "\nTEright ("
                            << TEright.x
                            << ", "
                            << TEright.y
                            << ") deviation: "
                            << d[1]
                            << "\nVgoal: "
                            << Vgoal[2].x
                            << ", "
                            << Vgoal[2].y
                            << endl;
                }
            }
        }
                //----------------------------------
        // One Variable Goal is in sight
        //----------------------------------

        else
        {
            if(inSight[0] == 1)
            {
                Vgoal[2] = Vgoal[0];
                goal = 0;
            }
            else
            {
                Vgoal[2] = Vgoal[1];
                goal = 1;
            }
        }
    }

    //----------------------------------
    // Only one Variable Goal is forward
    //----------------------------------

    else
    {
        if(cos_alpha[0] > cos(backwardsAngle))
        {
            Vgoal[2] = Vgoal[0];
            goal = 0;
        }

        if(cos_alpha[1] > cos(backwardsAngle))
        {
            Vgoal[2] = Vgoal[1];
            goal = 1;
        }
    }
}

else if(flag[0] == 1)
{
    //----------------------------------
    // Only left has sufficient corridor gap
    //----------------------------------

    Vgoal[2] = Vgoal[0];

    goal = 0;
    *stuck = 0;
}

else if(flag[1] == 1)
{
    //----------------------------------
    // Only right has sufficient corridor gap
    //----------------------------------

    Vgoal[2] = Vgoal[1];

    goal = 1;
    *stuck = 0;
}

else
{
    //----------------------------------
    // No feasible Variable Goal
    //----------------------------------

    if(print == 1)
    {
        cout << "no element has gap from corridor." << endl;
    }

    goal = -1;
    *stuck = 1;
}

 //----------------------------------
 // Backward Variable Goal check
 //----------------------------------

if(cos_alpha[0] < cos(backwardsAngle) &&
   cos_alpha[1] < cos(backwardsAngle))
{
    if(print == 1)
    {
        cout << "variable goal is backwards." << endl;

        cout
                << "cos alpha: "
                << cos_alpha[0]
                << " & "
                << cos_alpha[1]
                << " is less than "
                << cos(backwardsAngle)
                << endl;
    }

    goal = -1;
    *stuck = 1;
}
else
{
    *stuck = 0;
}

*direct = goal;

if(print == 1)
{
    cout
            << "direction value inside function: "
            << *direct
            << endl;
}

return Vgoal[2];
}
//-----------------------------------------------------
// Obstacle Within 6R
//-----------------------------------------------------

int obstWithin_6R(
        Obstacle obst,
        Obstacle obstToJudge,
        Coordinates Vgoal,
        double ped_radius,
        int print,
        double timesAway)
{
    //----------------------------------
    // Corridor direction
    //----------------------------------

    Coordinates v;
    Coordinates perp_v;

    v.x = Vgoal.x - obst.x;
    v.y = Vgoal.y - obst.y;

    v.mod =
            sqrt(
                v.x * v.x +
                v.y * v.y);

    perp_v.x = -v.y / v.mod;
    perp_v.y =  v.x / v.mod;

    //----------------------------------
    // Corridor vertices
    //----------------------------------

    Coordinates A;
    Coordinates B;
    Coordinates C;
    Coordinates D;
    Coordinates eightR;

    A.x =
            obst.x +
            (2.0 * obst.radius) *
            perp_v.x;

    A.y =
            obst.y +
            (2.0 * obst.radius) *
            perp_v.y;

    B.x =
            obst.x -
            (2.0 * obst.radius) *
            perp_v.x;

    B.y =
            obst.y -
            (2.0 * obst.radius) *
            perp_v.y;

    eightR.x =
            obst.x +
            (2.0 * ped_radius +
             2.0 * obst.radius +
             4.0 * timesAway * ped_radius)
            *
            v.x
            /
            v.mod;

    eightR.y =
            obst.y +
            (2.0 * ped_radius +
             2.0 * obst.radius +
             4.0 * timesAway * ped_radius)
            *
            v.y
            /
            v.mod;

    C.x =
            eightR.x +
            (2.0 * obst.radius) *
            perp_v.x;

    C.y =
            eightR.y +
            (2.0 * obst.radius) *
            perp_v.y;

    D.x =
            eightR.x -
            (2.0 * obst.radius) *
            perp_v.x;

    D.y =
            eightR.y -
            (2.0 * obst.radius) *
            perp_v.y;

    //----------------------------------
    // Rectangle orientation
    //----------------------------------

    Coordinates upperLeft;
    Coordinates upperRight;
    Coordinates lowerRight;
    Coordinates lowerLeft;

    if((v.x >= 0.0 && v.y >= 0.0) ||
       (v.x >= 0.0 && v.y <= 0.0))
    {
        upperLeft  = A;
        upperRight = C;
        lowerLeft  = B;
        lowerRight = D;
    }

    if((v.x <= 0.0 && v.y >= 0.0) ||
       (v.x <= 0.0 && v.y <= 0.0))
    {
        upperLeft  = D;
        upperRight = B;
        lowerLeft  = C;
        lowerRight = A;
    }

    //----------------------------------
    // Test obstacle
    //----------------------------------

    Coordinates point;
    Coordinates lineVertices[2];

    point.x = obstToJudge.x;
    point.y = obstToJudge.y;

    int flag =
            pointInsideRectangle(
                    upperLeft,
                    upperRight,
                    lowerRight,
                    lowerLeft,
                    point,
                    0,
                    lineVertices,
                    0);

    return flag;
}
//-----------------------------------------------------
// Corridor Within 5R
//-----------------------------------------------------

int CorridorWithin5R(
        Obstacle obst,
        Pedestrian ped,
        double timesAway,
        Coordinates upperLeft,
        Coordinates upperRight,
        Coordinates lowerRight,
        Coordinates lowerLeft,
        Coordinates* lineVertices,
        int print)
{
    //----------------------------------
    // Corridor direction
    //----------------------------------

    Coordinates v;
    Coordinates perp_v;

    v.x = ped.Vgoal.x - obst.x;
    v.y = ped.Vgoal.y - obst.y;

    v.mod =
            sqrt(
                v.x * v.x +
                v.y * v.y);

    perp_v.x = -v.y / v.mod;
    perp_v.y =  v.x / v.mod;

    //----------------------------------
    // 5R corridor point
    //----------------------------------

    Coordinates C;
    Coordinates D;
    Coordinates eightR;

    eightR.x =
            obst.x +
            (2.0 * ped.radius +
             obst.radius +
             4.0 * timesAway * ped.radius)
            *
            v.x
            /
            v.mod;

    eightR.y =
            obst.y +
            (2.0 * ped.radius +
             obst.radius +
             4.0 * timesAway * ped.radius)
            *
            v.y
            /
            v.mod;

    C.x =
            eightR.x +
            (2.0 * obst.radius) *
            perp_v.x;

    C.y =
            eightR.y +
            (2.0 * obst.radius) *
            perp_v.y;

    D.x =
            eightR.x -
            (2.0 * obst.radius) *
            perp_v.x;

    D.y =
            eightR.y -
            (2.0 * obst.radius) *
            perp_v.y;

    //----------------------------------
    // Test point C
    //----------------------------------

    int flag =
            pointInsideRectangle(
                    upperLeft,
                    upperRight,
                    lowerRight,
                    lowerLeft,
                    C,
                    1,
                    lineVertices,
                    print);

    //----------------------------------
    // Test point D
    //----------------------------------

    if(flag == 1)
    {
        flag =
                pointInsideRectangle(
                        upperLeft,
                        upperRight,
                        lowerRight,
                        lowerLeft,
                        D,
                        1,
                        lineVertices,
                        print);

        if(flag == 1)
        {
            if(print == 1)
            {
                cout
                        << "line vertices: ("
                        << lineVertices[0].x
                        << ", "
                        << lineVertices[0].y
                        << "); ("
                        << lineVertices[1].x
                        << ", "
                        << lineVertices[1].y
                        << ")\n";
            }

            return 1;
        }
        else
        {
            if(print == 1)
            {
                cout
                        << "line vertices: ("
                        << lineVertices[0].x
                        << ", "
                        << lineVertices[0].y
                        << "); ("
                        << lineVertices[1].x
                        << ", "
                        << lineVertices[1].y
                        << ")\n";
            }

            return 0;
        }
    }

    //----------------------------------
    // Point C outside
    //----------------------------------

    if(print == 1)
    {
        cout
                << "line vertices: ("
                << lineVertices[0].x
                << ", "
                << lineVertices[0].y
                << "); ("
                << lineVertices[1].x
                << ", "
                << lineVertices[1].y
                << ")\n";
    }

    return 0;
}
//-----------------------------------------------------
// Variable Goal Algorithm
//-----------------------------------------------------

Coordinates VGA(
        Pedestrian ped,
        Obstacle obst[],
        int noOfObst,
        double angleOfSight,
        Coordinates vertex[],
        int* stuck,
        int addProbability,
        int multiPedCase,
        int print)
{
    //----------------------------------
    // Debug
    //----------------------------------

    if(print == 1)
    {
        cout
                << "\n### VARIABLE GOAL ###\n"
                << endl;
    }

    //----------------------------------
    // Initial Variable Goal
    //----------------------------------

    Coordinates Vgoal_function;

    Vgoal_function.x = ped.Vgoal.x;
    Vgoal_function.y = ped.Vgoal.y;

    //----------------------------------
    // Cluster initialization
    //----------------------------------

    Cluster clust[noOfObst];

    int totalNoOfClusters = 0;

    //----------------------------------
    // Initialize obstacles
    //----------------------------------

    for(int i = 0; i < noOfObst; i++)
    {
        clust[i].direct = -1;

        obst[i].clusterNo = -1;
    }
        //----------------------------------
    // Variables
    //----------------------------------

    int nearest_obst;
    int convergence;
    int Vgoal_cellNo;
    int obstForVgoal;

    int lengthOfCluster;
    int cluster[noOfObst];
    int convexHullLength;

    int obst_for_vgoal[noOfObst];
    int crossProduct[3];

    int TEleft;
    int TEright;

    int convexHullVertices[noOfObst];

    int direction;
    int a;
    int b;

    int inside;
    int obstInside_6R;
    int reoccurence;

    double dist_from_obstInside_6R;

    double timesAway_cluster = 1.1;

    double v5;
    double distance2;
    double distance1;

    double timesAway = 0.5;

    double dist_from_Vgoal1;

    double d_minFromCorridor;
    double enoughGapFromCorridor;

    Coordinates points[3];

    Coordinates teLeft;
    Coordinates teRight;

    Coordinates nearestObstacle;

    Coordinates Vgoal[noOfObst];

    Coordinates lineVertices[2];

    //----------------------------------
    // Initialization
    //----------------------------------

    nearest_obst = -1;

    convergence = 0;

    Vgoal_cellNo = 0;

    obstForVgoal = -1;

    //----------------------------------
    // Main VGA loop
    //----------------------------------

    while(convergence == 0)
    {
        if(print == 1)
        {
            cout
                    << "previous nearest obstacle: "
                    << nearest_obst
                    << endl;
        }

        nearest_obst =
                nearestObst(
                        ped,
                        obst,
                        noOfObst,
                        angleOfSight,
                        &convergence,
                        nearest_obst,
                        0);

        if(print == 1)
        {
            cout
                    << "nearest obstacle: "
                    << nearest_obst
                    << endl;
        }

        //----------------------------------
        // Nearest obstacle exists
        //----------------------------------

        if(nearest_obst != -1)
        {
            if(multiPedCase == 1)
            {
                double collisionTime =
                        tau(
                                ped,
                                obst[nearest_obst]);

                if(collisionTime < 0.0 ||
                   collisionTime > 3.0)
                {
                    goto STOP;
                }
            }

            //----------------------------------
            // Cluster assignment
            //----------------------------------

            if(obst[nearest_obst].clusterNo == -1)
            {
                obst[nearest_obst].clusterNo =
                        totalNoOfClusters;

                totalNoOfClusters++;
            }
                        //----------------------------------
            // Cluster Formation
            //----------------------------------

            lengthOfCluster =
                    ClusterFormation(
                            cluster,
                            nearest_obst,
                            obst,
                            noOfObst,
                            angleOfSight,
                            ped,
                            timesAway_cluster);

            clust[obst[nearest_obst].clusterNo].length =
                    lengthOfCluster;

            //----------------------------------
            // Assign cluster number
            //----------------------------------

            for(int i = 0; i < lengthOfCluster; i++)
            {
                obst[cluster[i]].clusterNo =
                        obst[nearest_obst].clusterNo;
            }

            //----------------------------------
            // Convex Hull
            //----------------------------------

            convexHullLength =
                    ConvexHull(
                            convexHullVertices,
                            cluster,
                            lengthOfCluster,
                            obst);
                                        //----------------------------------
            // Tangential Elements
            //----------------------------------

            TEleft = convexHullVertices[0];

            TEright = convexHullVertices[0];

            for(int i = 1; i <= convexHullLength; i++)
            {
                //----------------------------------
                // Left Tangential Element
                //----------------------------------

                crossProduct[1] = TEleft;
                crossProduct[2] = convexHullVertices[i];

                points[0].x = ped.pos_x;
                points[0].y = ped.pos_y;

                points[1].x =
                        obst[crossProduct[1]].x;
                points[1].y =
                        obst[crossProduct[1]].y;

                points[2].x =
                        obst[crossProduct[2]].x;
                points[2].y =
                        obst[crossProduct[2]].y;

                direction =
                        inLeftOrRight(
                                points[0],
                                points[1],
                                points[2]);

                if(direction == 1)
                {
                    TEleft =
                            convexHullVertices[i];
                }

                //----------------------------------
                // Right Tangential Element
                //----------------------------------

                crossProduct[1] = TEright;

                points[1].x =
                        obst[crossProduct[1]].x;
                points[1].y =
                        obst[crossProduct[1]].y;

                direction =
                        inLeftOrRight(
                                points[0],
                                points[1],
                                points[2]);

                if(direction == 0)
                {
                    TEright =
                            convexHullVertices[i];
                }
            }
                        //----------------------------------
            // Variable Goal
            //----------------------------------

            a = TEleft;
            b = TEright;

            v5 = rand();

            v5 =
                    (v5 + 1.0)
                    /
                    (RAND_MAX + 1.0);

            teLeft.x = obst[a].x;
            teLeft.y = obst[a].y;

            teRight.x = obst[b].x;
            teRight.y = obst[b].y;

            nearestObstacle.x =
                    obst[nearest_obst].x;

            nearestObstacle.y =
                    obst[nearest_obst].y;

            Vgoal_function =
                    VariableGoal(
                            ped,
                            teLeft,
                            teRight,
                            obst[a].radius,
                            angleOfSight,
                            vertex,
                            v5,
                            nearestObstacle,
                            print,
                            addProbability,
                            clust[obst[nearest_obst].clusterNo].direct,
                            stuck,
                            &direction);

            //----------------------------------
            // Update current Variable Goal
            //----------------------------------

            ped.Vgoal.x =
                    Vgoal_function.x;

            ped.Vgoal.y =
                    Vgoal_function.y;

            //----------------------------------
            // Stuck condition
            //----------------------------------

            if(*stuck == 1)
            {
                Vgoal_function.x =
                        ped.finalGoal_x;

                Vgoal_function.y =
                        ped.finalGoal_y;

                goto STOP;
            }

            //----------------------------------
            // Update cluster direction
            //----------------------------------

            if(clust[obst[nearest_obst].clusterNo].direct == -1)
            {
                clust[obst[nearest_obst].clusterNo].direct =
                        direction;
            }

            obstForVgoal =
                    (direction == 0)
                    ?
                    a
                    :
                    b;

            if(print == 1)
            {
                cout
                        << "direction: "
                        << direction
                        << "\tobst for vgoal: "
                        << obstForVgoal
                        << endl;
            }
                        //----------------------------------
            // Re-occurrence
            //----------------------------------

            obst_for_vgoal[Vgoal_cellNo] =
                    obstForVgoal;

            Vgoal[Vgoal_cellNo].x =
                    ped.Vgoal.x;

            Vgoal[Vgoal_cellNo].y =
                    ped.Vgoal.y;

            ped.dist_from_goal =
                    sqrt(
                        (ped.pos_x - ped.Vgoal.x) *
                        (ped.pos_x - ped.Vgoal.x)
                        +
                        (ped.pos_y - ped.Vgoal.y) *
                        (ped.pos_y - ped.Vgoal.y));

            distance2 =
                    2.0 *
                    ped.dist_from_goal;

            reoccurence = 0;

            for(int i = 0; i < Vgoal_cellNo; i++)
            {
                if(ped.Vgoal.x == Vgoal[i].x &&
                   ped.Vgoal.y == Vgoal[i].y)
                {
                    reoccurence = 1;

                    for(int j = 0; j < Vgoal_cellNo; j++)
                    {
                        distance1 =
                                sqrt(
                                    (ped.pos_x - Vgoal[j].x) *
                                    (ped.pos_x - Vgoal[j].x)
                                    +
                                    (ped.pos_y - Vgoal[j].y) *
                                    (ped.pos_y - Vgoal[j].y));

                        if(distance2 > distance1)
                        {
                            ped.Vgoal.x =
                                    Vgoal[j].x;

                            ped.Vgoal.y =
                                    Vgoal[j].y;

                            distance2 =
                                    distance1;

                            obstForVgoal =
                                    obst_for_vgoal[j];
                        }
                    }

                    convergence = 1;

                    break;
                }
            }

            //----------------------------------
            // Rebuild cluster if repeated
            //----------------------------------

            if(reoccurence == 1)
            {
                lengthOfCluster =
                        ClusterFormation(
                                cluster,
                                obstForVgoal,
                                obst,
                                noOfObst,
                                angleOfSight,
                                ped,
                                timesAway_cluster);

                convexHullLength =
                        ConvexHull(
                                convexHullVertices,
                                cluster,
                                lengthOfCluster,
                                obst);
            }

            Vgoal_cellNo++;;
        }
    }
        //----------------------------------
    // Current Variable Goal
    //----------------------------------

    Vgoal_function.x =
            ped.Vgoal.x;

    Vgoal_function.y =
            ped.Vgoal.y;

    if(print == 1)
    {
        cout
                << "obst for vgoal: "
                << obstForVgoal
                << endl;

        cout
                << "ped.goal: "
                << ped.Vgoal.x
                << ", "
                << ped.Vgoal.y
                << endl;
    }

    //----------------------------------
    // Update Variable Goal
    //----------------------------------

    if((ped.Vgoal.x != ped.finalGoal_x ||
        ped.Vgoal.y != ped.finalGoal_y)
        &&
        obstForVgoal != -1)
    {
        distance2 =
                10.0 *
                obst[0].radius;

        obstInside_6R = -1;
                //----------------------------------
        // Search obstacle inside 6R region
        //----------------------------------

        for(int i = 0; i < noOfObst; i++)
        {
            if(i == obstForVgoal)
            {
                continue;
            }

            Vgoal_function.x = ped.Vgoal.x;
            Vgoal_function.y = ped.Vgoal.y;

            inside =
                    obstWithin_6R(
                            obst[obstForVgoal],
                            obst[i],
                            Vgoal_function,
                            ped.radius,
                            0,
                            timesAway);

            if(inside == 1)
            {
                if(print == 1)
                {
                    cout
                            << "obst["
                            << i
                            << "] is within 6R."
                            << endl;
                }

                distance1 =
                        sqrt(
                            (obst[obstForVgoal].x - obst[i].x) *
                            (obst[obstForVgoal].x - obst[i].x)
                            +
                            (obst[obstForVgoal].y - obst[i].y) *
                            (obst[obstForVgoal].y - obst[i].y));

                if(distance1 < distance2)
                {
                    obstInside_6R = i;
                    distance2 = distance1;
                }
            }
        }

        if(print == 1)
        {
            cout
                    << "obstInside_6R: "
                    << obstInside_6R
                    << endl;
        }

        dist_from_obstInside_6R =
                distance2;

        //----------------------------------
        // Obstacle exists inside 6R
        //----------------------------------

        if(obstInside_6R != -1)
        {
            dist_from_Vgoal1 =
                    dist_from_obstInside_6R
                    /
                    2.0;

            ped.Vgoal.x =
                    obst[obstForVgoal].x
                    +
                    dist_from_Vgoal1
                    *
                    (ped.Vgoal.x - obst[obstForVgoal].x)
                    /
                    (ped.radius + obst[0].radius);

            ped.Vgoal.y =
                    obst[obstForVgoal].y
                    +
                    dist_from_Vgoal1
                    *
                    (ped.Vgoal.y - obst[obstForVgoal].y)
                    /
                    (ped.radius + obst[0].radius);

            if(print == 1)
            {
                printf(
                    "ped.goal_x = obst[obstForVgoal].x + dist_from_Vgoal1 * (ped.goal_x - obst[obstForVgoal].x) / (ped.radius + obst[0].radius)\n%f = %f + %f * (%f - %f) / (%f + %f)\n",
                    ped.Vgoal.x,
                    obst[obstForVgoal].x,
                    dist_from_Vgoal1,
                    ped.Vgoal.x,
                    obst[obstForVgoal].x,
                    ped.radius,
                    obst[0].radius);
            }
        }

        //----------------------------------
        // No obstacle inside 6R
        //----------------------------------

                else
        {
            //----------------------------------
            // Check corridor clearance
            //----------------------------------

            enoughGapFromCorridor =
                    CorridorWithin5R(
                            obst[obstForVgoal],
                            ped,
                            timesAway,
                            vertex[0],
                            vertex[1],
                            vertex[2],
                            vertex[3],
                            lineVertices,
                            print);

            //----------------------------------
            // Corridor too narrow
            //----------------------------------
Coordinates obstPoint;

obstPoint.x = obst[obstForVgoal].x;
obstPoint.y = obst[obstForVgoal].y;
            if(enoughGapFromCorridor == 0)
            {
               d_minFromCorridor =
        DistFromCorridor(
                ped,
                obstPoint,
                obst[obstForVgoal].radius,
                vertex[0],
                vertex[1],
                vertex[2],
                vertex[3],
                direction,
                print);

                if(print == 1)
                {
                    cout
                            << "Nearest corridor side: "
                            << d_minFromCorridor
                            << endl;
                }

                              d_minFromCorridor =
                        (
                            (lineVertices[1].y - lineVertices[0].y) *
                            (obst[obstForVgoal].x - lineVertices[0].x)
                            -
                            (lineVertices[1].x - lineVertices[0].x) *
                            (obst[obstForVgoal].y - lineVertices[0].y)
                        )
                        /
                        sqrt(
                            (lineVertices[1].y - lineVertices[0].y) *
                            (lineVertices[1].y - lineVertices[0].y)
                            +
                            (lineVertices[1].x - lineVertices[0].x) *
                            (lineVertices[1].x - lineVertices[0].x)
                        );

                if(d_minFromCorridor < 0.0)
                {
                    d_minFromCorridor *= -1.0;
                }

                ped.Vgoal.x =
                        obst[obstForVgoal].x
                        +
                        (d_minFromCorridor + obst[0].radius)
                        / 2.0
                        *
                        (ped.Vgoal.x - obst[obstForVgoal].x)
                        /
                        (ped.radius + obst[0].radius);

                ped.Vgoal.y =
                        obst[obstForVgoal].y
                        +
                        (d_minFromCorridor + obst[0].radius)
                        / 2.0
                        *
                        (ped.Vgoal.y - obst[obstForVgoal].y)
                        /
                        (ped.radius + obst[0].radius);

                if(print == 1)
                {
                    cout << "not enough gap from corridor." << endl;

                    cout
                        << "line vertices: ("
                        << lineVertices[0].x
                        << ", "
                        << lineVertices[0].y
                        << "); ("
                        << lineVertices[1].x
                        << ", "
                        << lineVertices[1].y
                        << ")\n";
                }
            }
            else
            {
                ped.Vgoal.x =
                        obst[obstForVgoal].x
                        +
                        (
                            obst[0].radius
                            +
                            timesAway * 2.0 * ped.radius
                            +
                            ped.radius
                        )
                        *
                        (ped.Vgoal.x - obst[obstForVgoal].x)
                        /
                        (ped.radius + obst[0].radius);

                ped.Vgoal.y =
                        obst[obstForVgoal].y
                        +
                        (
                            obst[0].radius
                            +
                            timesAway * 2.0 * ped.radius
                            +
                            ped.radius
                        )
                        *
                        (ped.Vgoal.y - obst[obstForVgoal].y)
                        /
                        (ped.radius + obst[0].radius);

                if(print == 1)
                {
                    cout << "enough gap from corridor." << endl;
                }
            }
        }
    }

    Vgoal_function.x = ped.Vgoal.x;
    Vgoal_function.y = ped.Vgoal.y;

    if(print == 1)
    {
        cout
                << "ped.goal: "
                << ped.Vgoal.x
                << ", "
                << ped.Vgoal.y
                << endl;
    }

STOP:

    return Vgoal_function;
}
        
#endif