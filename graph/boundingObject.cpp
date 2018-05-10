#include "boundingObject.h"
#include "kdnode.h"
#include <cfloat>

void AABB::setAABB(GLfloat MinX, GLfloat MaxX, GLfloat MinY, GLfloat MaxY, GLfloat MinZ, GLfloat MaxZ)
{
    xDist = glm::abs(MinX - MaxX);
    yDist = glm::abs(MinY - MaxY);
    zDist = glm::abs(MinZ - MaxZ);

    GLfloat xC = (MaxX + MinX) * 0.5f;
    GLfloat yC = (MaxY + MinY) * 0.5f;
    GLfloat zC = (MaxZ + MinZ) * 0.5f;

    centerPos = glm::vec3(xC, yC, zC);

    negX = MinX - FLT_EPSILON;
    negY = MinY - FLT_EPSILON;
    negZ = MinZ - FLT_EPSILON;

    posX = MaxX + FLT_EPSILON;
    posY = MaxY + FLT_EPSILON;
    posZ = MaxZ + FLT_EPSILON;
}

void AABB::splitAABB(int axis, GLfloat splitPlane, AABB &left, AABB &right)
{
    if(axis == KdNode::X_AXIS)
    {    
        left.negX = negX;
        left.posX = splitPlane;
        right.negX = splitPlane + FLT_EPSILON;
        right.posX = posX;
    }
    else
    {
        left.negX = negX;
        right.negX = negX;
        left.posX = posX;
        right.posX = posX;
    }

    if(axis == KdNode::Y_AXIS)
    {    
        left.negY = negY;
        left.posY = splitPlane;
        right.negY = splitPlane + FLT_EPSILON;
        right.posY = posY;
    }
    else
    {
        left.negY = negY;
        right.negY = negY;
        left.posY = posY;
        right.posY = posY;
    }

    if(axis == KdNode::Z_AXIS)
    {    
        left.negZ = negZ;
        left.posZ = splitPlane;
        right.negZ = splitPlane + FLT_EPSILON;
        right.posZ = posZ;
    }
    else
    {
        left.negZ = negZ;
        right.negZ = negZ;
        left.posZ = posZ;
        right.posZ = posZ;
    }
}

GLfloat AABB::getSplitPlane(int axis)
{
	GLfloat plane;
	if(axis == KdNode::X_AXIS)
		plane =  (posX + negX) / 2.0f;
	else if (axis == KdNode::Y_AXIS)
		plane =  (posY + negY) / 2.0f;
	else if (axis == KdNode::Z_AXIS)
		plane =  (posZ + negZ) / 2.0f;
	return plane;
}

bool AABB::isInside(glm::vec3 position) const
{
    return (position[0] < posX && position[0] > negX 
         && position[1] < posY && position[1] > negY 
         && position[2] < posZ && position[2] > negZ);
}
