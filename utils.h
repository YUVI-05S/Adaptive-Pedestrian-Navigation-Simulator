#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include "headerClass.h"
#include <vector>
#include <algorithm>
#include <limits>

//-----------------------------------------------------
// Distance between two points
//-----------------------------------------------------

double distance(double x1,
                double y1,
                double x2,
                double y2)
{
    return sqrt((x2-x1)*(x2-x1)
               +(y2-y1)*(y2-y1));
}
//-----------------------------------------------------
// Distance squared between two points
//-----------------------------------------------------

double distanceSquared(double x1,
                       double y1,
                       double x2,
                       double y2)
{
    return (x2-x1)*(x2-x1)
         + (y2-y1)*(y2-y1);
}
//-----------------------------------------------------
// Distance between pedestrian and obstacle
//-----------------------------------------------------

double distance(const Pedestrian& ped,
                const Obstacle& obst)
{
    return sqrt(
            (ped.pos_x-obst.x)*(ped.pos_x-obst.x)
           +(ped.pos_y-obst.y)*(ped.pos_y-obst.y));
}
//-----------------------------------------------------
// Distance between two pedestrians
//-----------------------------------------------------

double distance(const Pedestrian& ped1,
                const Pedestrian& ped2)
{
    return sqrt(
            (ped1.pos_x-ped2.pos_x)*(ped1.pos_x-ped2.pos_x)
           +(ped1.pos_y-ped2.pos_y)*(ped1.pos_y-ped2.pos_y));
}
//-----------------------------------------------------
// Vector from point1 to point2
//-----------------------------------------------------

Coordinates vectorBetween(double x1,
                          double y1,
                          double x2,
                          double y2)
{
    Coordinates v;

    v.x = x2-x1;
    v.y = y2-y1;

    v.mod = sqrt(v.x*v.x+v.y*v.y);

    return v;
}
//-----------------------------------------------------
// Dot product
//-----------------------------------------------------

double dotProduct(double x1,
                  double y1,
                  double x2,
                  double y2)
{
    return x1*x2 + y1*y2;
}
//-----------------------------------------------------
// Cross product
//-----------------------------------------------------

double crossProduct(double x1,
                    double y1,
                    double x2,
                    double y2)
{
    return x1*y2 - y1*x2;
}
//-----------------------------------------------------
// Angle between two vectors
//-----------------------------------------------------

double angleBetween(double x1,
                    double y1,
                    double x2,
                    double y2)
{
    double mod1 = sqrt(x1*x1+y1*y1);
    double mod2 = sqrt(x2*x2+y2*y2);

    if(mod1 < 1e-12 || mod2 < 1e-12)
        return 0.0;

    double cosineTheta =
            dotProduct(x1,y1,x2,y2)
            /
            (mod1*mod2);

    if(cosineTheta > 1.0)
        cosineTheta = 1.0;

    if(cosineTheta < -1.0)
        cosineTheta = -1.0;

    return acos(cosineTheta);
}
//-----------------------------------------------------
// Orientation test
//-----------------------------------------------------

int orientation(double x1,
                double y1,
                double x2,
                double y2,
                double x3,
                double y3)
{
    double val =
            (y2-y1)*(x3-x2)
            -
            (x2-x1)*(y3-y2);

    if(fabs(val)<1e-12)
        return 0;   // collinear

    if(val>0)
        return 1;   // clockwise

    return 2;       // counterclockwise
}
//-----------------------------------------------------
// Normalize vector
//-----------------------------------------------------

Coordinates normalize(double x,
                      double y)
{
    Coordinates unit;

    unit.mod = sqrt(x*x+y*y);

    if(unit.mod < 1e-12)
    {
        unit.x = 0.0;
        unit.y = 0.0;
        unit.mod = 0.0;
    }
    else
    {
        unit.x = x/unit.mod;
        unit.y = y/unit.mod;
    }

    return unit;
}
//-----------------------------------------------------
// Rotate vector
//-----------------------------------------------------

Coordinates rotateVector(double x,
                         double y,
                         double angle)
{
    Coordinates rotated;

    rotated.x =
            x*cos(angle)
            -
            y*sin(angle);

    rotated.y =
            x*sin(angle)
            +
            y*cos(angle);

    rotated.mod =
            sqrt(rotated.x*rotated.x
                 +
                 rotated.y*rotated.y);

    return rotated;
}
//-----------------------------------------------------
// Check if point lies inside circle
//-----------------------------------------------------

bool pointInsideCircle(double px,
                       double py,
                       const Obstacle& obst)
{
    double d =
            distance(px,
                     py,
                     obst.x,
                     obst.y);

    return d <= obst.radius;
}
//-----------------------------------------------------
// Distance from point to circle surface
//-----------------------------------------------------

double distanceToCircle(double px,
                        double py,
                        const Obstacle& obst)
{
    double d =
            distance(px,
                     py,
                     obst.x,
                     obst.y);

    return d - obst.radius;
}
//-----------------------------------------------------
// Projection of point P onto line AB
//-----------------------------------------------------

Coordinates projectPointOntoLine(double px,
                                 double py,
                                 double ax,
                                 double ay,
                                 double bx,
                                 double by)
{
    Coordinates projection;

    double ABx = bx-ax;
    double ABy = by-ay;

    double APx = px-ax;
    double APy = py-ay;

    double AB2 =
            ABx*ABx
            +
            ABy*ABy;

    if(AB2 < 1e-12)
    {
        projection.x = ax;
        projection.y = ay;
    }
    else
    {
        double t =
                (APx*ABx
                 +
                 APy*ABy)
                /
                AB2;

        projection.x =
                ax
                +
                t*ABx;

        projection.y =
                ay
                +
                t*ABy;
    }

    projection.mod = 0.0;

    return projection;
}
//-----------------------------------------------------
// Check if point P lies on segment AB
//-----------------------------------------------------

bool onSegment(double ax,
               double ay,
               double px,
               double py,
               double bx,
               double by)
{
    return
        px <= std::max(ax,bx)+1e-12 &&
        px >= std::min(ax,bx)-1e-12 &&
        py <= std::max(ay,by)+1e-12 &&
        py >= std::min(ay,by)-1e-12;
}
//-----------------------------------------------------
// Check if segments AB and CD intersect
//-----------------------------------------------------

bool doIntersect(double ax,
                 double ay,
                 double bx,
                 double by,
                 double cx,
                 double cy,
                 double dx,
                 double dy)
{
    int o1 = orientation(ax,ay,bx,by,cx,cy);
    int o2 = orientation(ax,ay,bx,by,dx,dy);
    int o3 = orientation(cx,cy,dx,dy,ax,ay);
    int o4 = orientation(cx,cy,dx,dy,bx,by);

    if(o1 != o2 && o3 != o4)
        return true;

    if(o1==0 && onSegment(ax,ay,cx,cy,bx,by))
        return true;

    if(o2==0 && onSegment(ax,ay,dx,dy,bx,by))
        return true;

    if(o3==0 && onSegment(cx,cy,ax,ay,dx,dy))
        return true;

    if(o4==0 && onSegment(cx,cy,bx,by,dx,dy))
        return true;

    return false;
}
//-----------------------------------------------------
// Nearest point on segment AB to point P
//-----------------------------------------------------

Coordinates nearestPointOnSegment(double px,
                                  double py,
                                  double ax,
                                  double ay,
                                  double bx,
                                  double by)
{
    Coordinates nearest;

    double ABx = bx-ax;
    double ABy = by-ay;

    double AB2 = ABx*ABx + ABy*ABy;

    if(AB2 < 1e-12)
    {
        nearest.x = ax;
        nearest.y = ay;
        return nearest;
    }

    double APx = px-ax;
    double APy = py-ay;

    double t =
        (APx*ABx + APy*ABy)
        /
        AB2;

    if(t < 0.0)
        t = 0.0;

    if(t > 1.0)
        t = 1.0;

    nearest.x = ax + t*ABx;
    nearest.y = ay + t*ABy;

    nearest.mod = 0.0;

    return nearest;
}
//-----------------------------------------------------
// Distance from point to segment
//-----------------------------------------------------

double distancePointToSegment(double px,
                              double py,
                              double ax,
                              double ay,
                              double bx,
                              double by)
{
    Coordinates nearest =
            nearestPointOnSegment(px,
                                  py,
                                  ax,
                                  ay,
                                  bx,
                                  by);

    return distance(px,
                    py,
                    nearest.x,
                    nearest.y);
}
//-----------------------------------------------------
// Signed angle from vector 1 to vector 2
//-----------------------------------------------------

double signedAngle(double x1,
                   double y1,
                   double x2,
                   double y2)
{
    double dot =
            dotProduct(x1,y1,x2,y2);

    double cross =
            crossProduct(x1,y1,x2,y2);

    return atan2(cross,dot);
}
//-----------------------------------------------------
// Side test
//-----------------------------------------------------

int sideOfLine(double ax,
               double ay,
               double bx,
               double by,
               double px,
               double py)
{
    double value =
            (bx-ax)*(py-ay)
            -
            (by-ay)*(px-ax);

    if(fabs(value)<1e-12)
        return 0;

    if(value>0)
        return 1;       // left

    return -1;          // right
}
//-----------------------------------------------------
// Polar angle
//-----------------------------------------------------

double polarAngle(double x,
                  double y)
{
    return atan2(y,x);
}
//-----------------------------------------------------
// Relative angle of obstacle wrt pedestrian
//-----------------------------------------------------

double relativeAngle(const Pedestrian& ped,
                     const Obstacle& obst)
{
    double dx =
            obst.x-ped.pos_x;

    double dy =
            obst.y-ped.pos_y;

    return atan2(dy,dx);
}
//-----------------------------------------------------
// Midpoint between two points
//-----------------------------------------------------

Coordinates midpoint(double x1,
                     double y1,
                     double x2,
                     double y2)
{
    Coordinates mid;

    mid.x = 0.5*(x1+x2);

    mid.y = 0.5*(y1+y2);

    mid.mod = 0.0;

    return mid;
}
//-----------------------------------------------------
// Clamp
//-----------------------------------------------------

double clamp(double value,
             double minimum,
             double maximum)
{
    if(value<minimum)
        return minimum;

    if(value>maximum)
        return maximum;

    return value;
}
//-----------------------------------------------------
// Degrees to radians
//-----------------------------------------------------

double deg2rad(double degree)
{
    return degree*M_PI/180.0;
}
//-----------------------------------------------------
// Radians to degrees
//-----------------------------------------------------

double rad2deg(double radian)
{
    return radian*180.0/M_PI;
}
//-----------------------------------------------------
// Unit vector from A to B
//-----------------------------------------------------

Coordinates unitVector(double x1,
                       double y1,
                       double x2,
                       double y2)
{
    return normalize(x2-x1,
                     y2-y1);
}
//-----------------------------------------------------
// Linear interpolation
//-----------------------------------------------------

Coordinates interpolate(double x1,
                        double y1,
                        double x2,
                        double y2,
                        double t)
{
    Coordinates point;

    point.x =
            x1+t*(x2-x1);

    point.y =
            y1+t*(y2-y1);

    point.mod = 0.0;

    return point;
}
//-----------------------------------------------------
// Check circle overlap
//-----------------------------------------------------

bool circlesOverlap(double x1,
                    double y1,
                    double r1,
                    double x2,
                    double y2,
                    double r2)
{
    return distanceSquared(x1,
                           y1,
                           x2,
                           y2)
           <=
           (r1+r2)*(r1+r2);
}
//-----------------------------------------------------
// Pedestrian overlap
//-----------------------------------------------------

bool overlap(const Pedestrian& ped1,
             const Pedestrian& ped2)
{
    return circlesOverlap(
                ped1.pos_x,
                ped1.pos_y,
                ped1.radius,
                ped2.pos_x,
                ped2.pos_y,
                ped2.radius);
}
//-----------------------------------------------------
// Pedestrian-obstacle overlap
//-----------------------------------------------------

bool overlap(const Pedestrian& ped,
             const Obstacle& obst)
{
    return circlesOverlap(
                ped.pos_x,
                ped.pos_y,
                ped.radius,
                obst.x,
                obst.y,
                obst.radius);
}
//-----------------------------------------------------
// Distance from point to circle center
//-----------------------------------------------------

double distanceToCenter(double px,
                        double py,
                        const Obstacle& obst)
{
    return distance(px,
                    py,
                    obst.x,
                    obst.y);
}
//-----------------------------------------------------
// Check if two circles intersect
//-----------------------------------------------------

bool circlesIntersect(double x1,
                      double y1,
                      double r1,
                      double x2,
                      double y2,
                      double r2)
{
    double d2 =
            distanceSquared(x1,
                            y1,
                            x2,
                            y2);

    double sumR =
            r1+r2;

    double diffR =
            fabs(r1-r2);

    return
        d2 <= sumR*sumR
        &&
        d2 >= diffR*diffR;
}
//-----------------------------------------------------
// Check if one circle contains another
//-----------------------------------------------------

bool circleContains(double x1,
                    double y1,
                    double r1,
                    double x2,
                    double y2,
                    double r2)
{
    double d =
            distance(x1,
                     y1,
                     x2,
                     y2);

    return d+r2 <= r1;
}
//-----------------------------------------------------
// Tangent length from point to circle
//-----------------------------------------------------

double tangentLength(double px,
                     double py,
                     const Obstacle& obst)
{
    double d =
            distance(px,
                     py,
                     obst.x,
                     obst.y);

    if(d <= obst.radius)
        return 0.0;

    return sqrt(d*d-obst.radius*obst.radius);
}
//-----------------------------------------------------
// Circle diameter
//-----------------------------------------------------

double diameter(const Obstacle& obst)
{
    return 2.0*obst.radius;
}
//-----------------------------------------------------
// Circle circumference
//-----------------------------------------------------

double circumference(const Obstacle& obst)
{
    return 2.0*M_PI*obst.radius;
}
//-----------------------------------------------------
// Circle area
//-----------------------------------------------------

double area(const Obstacle& obst)
{
    return M_PI*
           obst.radius*
           obst.radius;
}
//-----------------------------------------------------
// Polygon area
//-----------------------------------------------------

double polygonArea(const std::vector<Coordinates>& polygon)
{
    int n = polygon.size();

    if(n < 3)
        return 0.0;

    double area = 0.0;

    for(int i=0;i<n;i++)
    {
        int j = (i+1)%n;

        area += polygon[i].x*polygon[j].y
              - polygon[j].x*polygon[i].y;
    }

    return 0.5*fabs(area);
}
//-----------------------------------------------------
// Polygon orientation
//-----------------------------------------------------

int polygonOrientation(const std::vector<Coordinates>& polygon)
{
    int n = polygon.size();

    double sum = 0.0;

    for(int i=0;i<n;i++)
    {
        int j = (i+1)%n;

        sum +=
            (polygon[j].x-polygon[i].x)
            *
            (polygon[j].y+polygon[i].y);
    }

    if(sum > 0)
        return -1;

    return 1;
}
//-----------------------------------------------------
// Polygon centroid
//-----------------------------------------------------

Coordinates polygonCentroid(const std::vector<Coordinates>& polygon)
{
    Coordinates centroid;

    int n = polygon.size();

    double A = 0.0;

    for(int i=0;i<n;i++)
    {
        int j = (i+1)%n;

        double cross =
            polygon[i].x*polygon[j].y
            -
            polygon[j].x*polygon[i].y;

        A += cross;

        centroid.x +=
            (polygon[i].x+polygon[j].x)
            *
            cross;

        centroid.y +=
            (polygon[i].y+polygon[j].y)
            *
            cross;
    }

    A *= 0.5;

    if(fabs(A)<1e-12)
        return centroid;

    centroid.x /= (6*A);

    centroid.y /= (6*A);

    centroid.mod = sqrt(
                    centroid.x*centroid.x
                    +
                    centroid.y*centroid.y);

    return centroid;
}
//-----------------------------------------------------
// Point inside polygon
//-----------------------------------------------------

bool pointInsidePolygon(double px,
                        double py,
                        const std::vector<Coordinates>& polygon)
{
    int n = polygon.size();

    bool inside = false;

    for(int i=0,j=n-1;i<n;j=i++)
    {
        if(
            ((polygon[i].y>py)!=(polygon[j].y>py))
            &&
            (
                px
                <
                (polygon[j].x-polygon[i].x)
                *
                (py-polygon[i].y)
                /
                (polygon[j].y-polygon[i].y)
                +
                polygon[i].x
            )
          )
        {
            inside = !inside;
        }
    }

    return inside;
}
//-----------------------------------------------------
// Bounding box
//-----------------------------------------------------

void boundingBox(const std::vector<Coordinates>& polygon,
                 double& xmin,
                 double& xmax,
                 double& ymin,
                 double& ymax)
{
    xmin = xmax = polygon[0].x;
    ymin = ymax = polygon[0].y;

    for(auto p : polygon)
    {
        xmin = std::min(xmin,p.x);
        xmax = std::max(xmax,p.x);

        ymin = std::min(ymin,p.y);
        ymax = std::max(ymax,p.y);
    }
}
//-----------------------------------------------------
// Safe acos
//-----------------------------------------------------

double safeAcos(double x)
{
    if(x > 1.0)
        x = 1.0;

    if(x < -1.0)
        x = -1.0;

    return acos(x);
}
//-----------------------------------------------------
// Safe sqrt
//-----------------------------------------------------

double safeSqrt(double x)
{
    if(x < 0.0)
        return 0.0;

    return sqrt(x);
}
//-----------------------------------------------------
// Wrap angle to [-pi,pi]
//-----------------------------------------------------

double wrapAnglePi(double angle)
{
    while(angle > M_PI)
        angle -= 2.0*M_PI;

    while(angle < -M_PI)
        angle += 2.0*M_PI;

    return angle;
}
//-----------------------------------------------------
// Wrap angle to [0,2pi]
//-----------------------------------------------------

double wrapAngle2Pi(double angle)
{
    while(angle < 0.0)
        angle += 2.0*M_PI;

    while(angle >= 2.0*M_PI)
        angle -= 2.0*M_PI;

    return angle;
}
//-----------------------------------------------------
// Mean point
//-----------------------------------------------------

Coordinates meanPoint(const std::vector<Coordinates>& points)
{
    Coordinates mean;

    if(points.empty())
        return mean;

    for(auto p : points)
    {
        mean.x += p.x;
        mean.y += p.y;
    }

    mean.x /= points.size();
    mean.y /= points.size();

    mean.mod =
        sqrt(mean.x*mean.x
             +
             mean.y*mean.y);

    return mean;
}
//-----------------------------------------------------
// Mean angle
//-----------------------------------------------------

double meanAngle(const std::vector<double>& angles)
{
    if(angles.empty())
        return 0.0;

    double sx = 0.0;
    double sy = 0.0;

    for(double a : angles)
    {
        sx += cos(a);
        sy += sin(a);
    }

    return atan2(sy,sx);
}
//-----------------------------------------------------
// Standard deviation
//-----------------------------------------------------

double standardDeviation(const std::vector<double>& values)
{
    if(values.empty())
        return 0.0;

    double mean = 0.0;

    for(double x : values)
        mean += x;

    mean /= values.size();

    double variance = 0.0;

    for(double x : values)
        variance += (x-mean)*(x-mean);

    variance /= values.size();

    return sqrt(variance);
}
//-----------------------------------------------------
// Polar angle comparator helper
//-----------------------------------------------------

double polarAngleFromPoint(double cx,
                           double cy,
                           double px,
                           double py)
{
    return atan2(py-cy,
                 px-cx);
}
//-----------------------------------------------------
// Closest point to reference
//-----------------------------------------------------

Coordinates closestPoint(double x,
                         double y,
                         const std::vector<Coordinates>& points)
{
    Coordinates closest;

    if(points.empty())
        return closest;

    double minDist = 1e100;

    for(auto p : points)
    {
        double d =
            distanceSquared(x,
                            y,
                            p.x,
                            p.y);

        if(d < minDist)
        {
            minDist = d;
            closest = p;
        }
    }

    return closest;
}
//-----------------------------------------------------
// Furthest point to reference
//-----------------------------------------------------

Coordinates furthestPoint(double x,
                          double y,
                          const std::vector<Coordinates>& points)
{
    Coordinates furthest;

    if(points.empty())
        return furthest;

    double maxDist = -1.0;

    for(auto p : points)
    {
        double d =
            distanceSquared(x,
                            y,
                            p.x,
                            p.y);

        if(d > maxDist)
        {
            maxDist = d;
            furthest = p;
        }
    }

    return furthest;
}
//-----------------------------------------------------
// Mean distance from point
//-----------------------------------------------------

double meanDistance(double x,
                    double y,
                    const std::vector<Coordinates>& points)
{
    if(points.empty())
        return 0.0;

    double sum = 0.0;

    for(auto p : points)
    {
        sum += distance(x,
                        y,
                        p.x,
                        p.y);
    }

    return sum/points.size();
}
//-----------------------------------------------------
// Maximum distance from point
//-----------------------------------------------------

double maxDistance(double x,
                   double y,
                   const std::vector<Coordinates>& points)
{
    double maxD = 0.0;

    for(auto p : points)
    {
        double d =
            distance(x,
                     y,
                     p.x,
                     p.y);

        if(d > maxD)
            maxD = d;
    }

    return maxD;
}
//-----------------------------------------------------
// Obstacle center to coordinate
//-----------------------------------------------------

Coordinates obstacleCenter(const Obstacle& obst)
{
    Coordinates c;

    c.x = obst.x;
    c.y = obst.y;

    c.mod = sqrt(c.x*c.x+c.y*c.y);

    return c;
}
//-----------------------------------------------------
// Pedestrian position to coordinate
//-----------------------------------------------------

Coordinates pedestrianPosition(const Pedestrian& ped)
{
    Coordinates c;

    c.x = ped.pos_x;
    c.y = ped.pos_y;

    c.mod = sqrt(c.x*c.x+c.y*c.y);

    return c;
}
//-----------------------------------------------------
// Lexicographic comparison
//-----------------------------------------------------

bool lexicographicLess(const Coordinates& p1,
                       const Coordinates& p2)
{
    if(p1.x != p2.x)
        return p1.x < p2.x;

    return p1.y < p2.y;
}
//-----------------------------------------------------
// Squared magnitude
//-----------------------------------------------------

double normSquared(const Coordinates& p)
{
    return p.x*p.x + p.y*p.y;
}
//-----------------------------------------------------
// Vector between coordinates
//-----------------------------------------------------

Coordinates vectorBetween(const Coordinates& p1,
                          const Coordinates& p2)
{
    Coordinates v;

    v.x = p2.x-p1.x;
    v.y = p2.y-p1.y;

    v.mod = sqrt(v.x*v.x+v.y*v.y);

    return v;
}
//-----------------------------------------------------
// Cross product of OA × OB
//-----------------------------------------------------

double cross(const Coordinates& O,
             const Coordinates& A,
             const Coordinates& B)
{
    return
        (A.x-O.x)*(B.y-O.y)
        -
        (A.y-O.y)*(B.x-O.x);
}
//-----------------------------------------------------
// Distance between coordinates
//-----------------------------------------------------

double distance(const Coordinates& p1,
                const Coordinates& p2)
{
    return sqrt(
            (p1.x-p2.x)*(p1.x-p2.x)
            +
            (p1.y-p2.y)*(p1.y-p2.y));
}
//-----------------------------------------------------
// Polar angle relative to reference point
//-----------------------------------------------------

double polarAngle(const Coordinates& origin,
                  const Coordinates& point)
{
    return atan2(point.y-origin.y,
                 point.x-origin.x);
}
//-----------------------------------------------------
// Polar comparator helper
//-----------------------------------------------------

bool polarLess(const Coordinates& p1,
               const Coordinates& p2,
               const Coordinates& origin)
{
    return polarAngle(origin,p1)
           <
           polarAngle(origin,p2);
}
//-----------------------------------------------------
// Next-to-top element
//-----------------------------------------------------

Coordinates nextToTop(std::vector<Coordinates>& stack)
{
    return stack[stack.size()-2];
}
//-----------------------------------------------------
// Numerical tolerance
//-----------------------------------------------------
constexpr double EPSILON = 1e-12;
//-----------------------------------------------------
// Vector magnitude
//-----------------------------------------------------

double magnitude(double x,
                 double y)
{
    return sqrt(x*x + y*y);
}

//-----------------------------------------------------
// Equality test
//-----------------------------------------------------

bool equal(double a,
           double b)
{
    return fabs(a-b) < EPSILON;
}



//-----------------------------------------------------
// Strictly less with tolerance
//-----------------------------------------------------

bool lessThan(double a,
              double b)
{
    return a < b-EPSILON;
}



//-----------------------------------------------------
// Strictly greater with tolerance
//-----------------------------------------------------

bool greaterThan(double a,
                 double b)
{
    return a > b+EPSILON;
}



//-----------------------------------------------------
// Safe division
//-----------------------------------------------------

double safeDivide(double numerator,
                  double denominator)
{
    if(fabs(denominator) < EPSILON)
        return 0.0;

    return numerator/denominator;
}

//-----------------------------------------------------
// Clamp angle to [0,2π]
//-----------------------------------------------------

double clampAngle(double angle)
{
    while(angle < 0.0)
        angle += 2.0*M_PI;

    while(angle >= 2.0*M_PI)
        angle -= 2.0*M_PI;

    return angle;
}

//-----------------------------------------------------
// Smallest angle difference
//-----------------------------------------------------

double angleDifference(double a,
                       double b)
{
    return wrapAnglePi(a-b);
}



//-----------------------------------------------------
// Same direction test
//-----------------------------------------------------

bool sameDirection(double theta1,
                   double theta2,
                   double tolerance = 5.0*M_PI/180.0)
{
    return fabs(angleDifference(theta1,
                                theta2))
           < tolerance;
}



//-----------------------------------------------------
// Orthogonality test
//-----------------------------------------------------

bool perpendicular(double x1,
                   double y1,
                   double x2,
                   double y2)
{
    return fabs(
            dotProduct(x1,
                       y1,
                       x2,
                       y2)
           )
           < EPSILON;
}



//-----------------------------------------------------
// Parallel test
//-----------------------------------------------------

bool parallel(double x1,
              double y1,
              double x2,
              double y2)
{
    return fabs(
            crossProduct(x1,
                         y1,
                         x2,
                         y2)
           )
           < EPSILON;
}
//-----------------------------------------------------
// Safe asin
//-----------------------------------------------------

double safeAsin(double value)
{
    value = std::max(-1.0,
                     std::min(1.0,
                              value));

    return asin(value);
}

//-----------------------------------------------------
// Finite number check
//-----------------------------------------------------

bool isFinite(double value)
{
    return std::isfinite(value);
}



//-----------------------------------------------------
// Replace NaN or Inf with zero
//-----------------------------------------------------

double sanitize(double value)
{
    if(!std::isfinite(value))
        return 0.0;

    return value;
} 
//-----------------------------------------------------
// Polygon perimeter
//-----------------------------------------------------

double polygonPerimeter(const std::vector<Coordinates>& polygon)
{
    int n = polygon.size();

    if(n < 2)
        return 0.0;

    double perimeter = 0.0;

    for(int i=0;i<n;i++)
    {
        int j = (i+1)%n;

        perimeter += distance(polygon[i],
                              polygon[j]);
    }

    return perimeter;
}



//-----------------------------------------------------
// Cluster centroid
//-----------------------------------------------------

Coordinates clusterCentroid(
                const std::vector<Coordinates>& points)
{
    return meanPoint(points);
}



//-----------------------------------------------------
// Cluster radius
//-----------------------------------------------------

double clusterRadius(
            const std::vector<Coordinates>& points)
{
    if(points.empty())
        return 0.0;

    Coordinates center =
            clusterCentroid(points);

    double radius = 0.0;

    for(auto p : points)
    {
        double d =
                distance(center,p);

        if(d > radius)
            radius = d;
    }

    return radius;
}



//-----------------------------------------------------
// Point cloud diameter
//-----------------------------------------------------

double cloudDiameter(
            const std::vector<Coordinates>& points)
{
    double diameter = 0.0;

    int n = points.size();

    for(int i=0;i<n;i++)
    {
        for(int j=i+1;j<n;j++)
        {
            double d =
                    distance(points[i],
                             points[j]);

            if(d > diameter)
                diameter = d;
        }
    }

    return diameter;
}



//-----------------------------------------------------
// Leftmost point
//-----------------------------------------------------

Coordinates leftmostPoint(
            const std::vector<Coordinates>& points)
{
    Coordinates result = points[0];

    for(auto p : points)
    {
        if(p.x < result.x)
            result = p;
    }

    return result;
}



//-----------------------------------------------------
// Rightmost point
//-----------------------------------------------------

Coordinates rightmostPoint(
            const std::vector<Coordinates>& points)
{
    Coordinates result = points[0];

    for(auto p : points)
    {
        if(p.x > result.x)
            result = p;
    }

    return result;
}



//-----------------------------------------------------
// Highest point
//-----------------------------------------------------

Coordinates highestPoint(
            const std::vector<Coordinates>& points)
{
    Coordinates result = points[0];

    for(auto p : points)
    {
        if(p.y > result.y)
            result = p;
    }

    return result;
}



//-----------------------------------------------------
// Lowest point
//-----------------------------------------------------

Coordinates lowestPoint(
            const std::vector<Coordinates>& points)
{
    Coordinates result = points[0];

    for(auto p : points)
    {
        if(p.y < result.y)
            result = p;
    }

    return result;
}



//-----------------------------------------------------
// Convex hull (Monotone Chain)
//-----------------------------------------------------

std::vector<Coordinates>
convexHull(std::vector<Coordinates> points)
{
    int n = points.size();

    if(n <= 3)
        return points;

    std::sort(points.begin(),
              points.end(),
              lexicographicLess);

    std::vector<Coordinates> H(2*n);

    int k = 0;

    //-------------------------------
    // Lower hull
    //-------------------------------

    for(int i=0;i<n;i++)
    {
        while(k >= 2 &&
              cross(H[k-2],
                    H[k-1],
                    points[i]) <= 0)
        {
            k--;
        }

        H[k++] = points[i];
    }

    //-------------------------------
    // Upper hull
    //-------------------------------

    for(int i=n-2,
            t=k+1;
        i>=0;
        i--)
    {
        while(k >= t &&
              cross(H[k-2],
                    H[k-1],
                    points[i]) <= 0)
        {
            k--;
        }

        H[k++] = points[i];
    }

    H.resize(k-1);

    return H;
}
//-----------------------------------------------------
// Penetration depth : Pedestrian - Obstacle
//-----------------------------------------------------

double penetrationDepth(const Pedestrian& ped,
                        const Obstacle& obst)
{
    double d =
            distance(ped.pos_x,
                     ped.pos_y,
                     obst.x,
                     obst.y);

    double overlap =
            ped.radius
            +
            obst.radius
            -
            d;

    if(overlap < 0.0)
        overlap = 0.0;

    return overlap;
}



//-----------------------------------------------------
// Penetration depth : Pedestrian - Pedestrian
//-----------------------------------------------------

double penetrationDepth(const Pedestrian& ped1,
                        const Pedestrian& ped2)
{
    double d =
            distance(ped1.pos_x,
                     ped1.pos_y,
                     ped2.pos_x,
                     ped2.pos_y);

    double overlap =
            ped1.radius
            +
            ped2.radius
            -
            d;

    if(overlap < 0.0)
        overlap = 0.0;

    return overlap;
}



//-----------------------------------------------------
// Normal vector : Pedestrian - Obstacle
//-----------------------------------------------------

Coordinates normalVector(const Pedestrian& ped,
                         const Obstacle& obst)
{
    return normalize(
                ped.pos_x-obst.x,
                ped.pos_y-obst.y);
}



//-----------------------------------------------------
// Normal vector : Pedestrian - Pedestrian
//-----------------------------------------------------

Coordinates normalVector(const Pedestrian& ped1,
                         const Pedestrian& ped2)
{
    return normalize(
                ped1.pos_x-ped2.pos_x,
                ped1.pos_y-ped2.pos_y);
}



//-----------------------------------------------------
// Tangential vector from normal vector
//-----------------------------------------------------

Coordinates tangentVector(const Coordinates& normal)
{
    Coordinates tangent;

    tangent.x = -normal.y;
    tangent.y = normal.x;

    tangent.mod = 1.0;

    return tangent;
}



//-----------------------------------------------------
// Relative tangential velocity : Pedestrian - Obstacle
//-----------------------------------------------------

double relativeTangentialVelocity(
                        const Pedestrian& ped,
                        const Obstacle& obst)
{
    Coordinates n =
            normalVector(ped,obst);

    Coordinates t =
            tangentVector(n);

    double rvx =
            ped.vel_x-obst.vel_x;

    double rvy =
            ped.vel_y-obst.vel_y;

    return dotProduct(rvx,
                      rvy,
                      t.x,
                      t.y);
}



//-----------------------------------------------------
// Relative tangential velocity : Pedestrian - Pedestrian
//-----------------------------------------------------

double relativeTangentialVelocity(
                        const Pedestrian& ped1,
                        const Pedestrian& ped2)
{
    Coordinates n =
            normalVector(ped1,ped2);

    Coordinates t =
            tangentVector(n);

    double rvx =
            ped1.vel_x-ped2.vel_x;

    double rvy =
            ped1.vel_y-ped2.vel_y;

    return dotProduct(rvx,
                      rvy,
                      t.x,
                      t.y);
}
//-----------------------------------------------------
// Time to Collision (Simplified)
//-----------------------------------------------------
#endif