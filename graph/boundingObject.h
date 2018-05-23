#ifndef _BOUNDING_OBJECT_H_
#define _BOUNDING_OBJECT_H_

#include <Eigen/Dense>
using Eigen::Vector3d;

struct AABoundingBox
{

    double posX;
    double negX;

    double posY;
    double negY;

    double posZ;
    double negZ;
};

//Axis Aligned Bounding Box
class AABB
{
public:
    AABB(){}
    ~AABB(){}
    void setAABB(double MinX, double MaxX, double MinY, double MaxY, double MinZ, double MaxZ);
    bool isInside(Vector3d position) const;

    double posX;
    double negX;
    double xDist;

    double posY;
    double negY;
    double yDist;

    double posZ;
    double negZ;
    double zDist;

    Vector3d centerPos;
};

#endif
