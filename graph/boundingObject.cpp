#include "boundingObject.h"
// #include "kdnode.h"
#include <cfloat>

void AABB::setAABB(double MinX, double MaxX, double MinY, double MaxY, double MinZ, double MaxZ)
{
    xDist = std::fabs(MinX - MaxX);
    yDist = std::fabs(MinY - MaxY);
    zDist = std::fabs(MinZ - MaxZ);

    double xC = (MaxX + MinX) * 0.5;
    double yC = (MaxY + MinY) * 0.5;
    double zC = (MaxZ + MinZ) * 0.5;

    centerPos = Vector3d(xC, yC, zC);

    negX = MinX - FLT_EPSILON;
    negY = MinY - FLT_EPSILON;
    negZ = MinZ - FLT_EPSILON;

    posX = MaxX + FLT_EPSILON;
    posY = MaxY + FLT_EPSILON;
    posZ = MaxZ + FLT_EPSILON;
}

bool AABB::isInside(Vector3d position) const
{
    return (position[0] < posX && position[0] > negX 
         && position[1] < posY && position[1] > negY 
         && position[2] < posZ && position[2] > negZ);
}
