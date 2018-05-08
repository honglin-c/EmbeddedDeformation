#ifndef _BOUNDING_OBJECT_H_
#define _BOUNDING_OBJECT_H_

#include <GL/glew.h>
#include <glm/vec3.hpp>


struct AABoundingBox
{

    GLfloat posX;
    GLfloat negX;

    GLfloat posY;
    GLfloat negY;

    GLfloat posZ;
    GLfloat negZ;
};

//Axis Aligned Bounding Box
class AABB
{
public:
    AABB(){}
    ~AABB(){}
    void setAABB(GLfloat MinX, GLfloat MaxX, GLfloat MinY, GLfloat MaxY, GLfloat MinZ, GLfloat MaxZ);
    void splitAABB(int axis, AABB &left, AABB &right);
    GLfloat getSplitPlane(int axis);

    GLfloat posX;
    GLfloat negX;
    GLfloat xDist;

    GLfloat posY;
    GLfloat negY;
    GLfloat yDist;

    GLfloat posZ;
    GLfloat negZ;
    GLfloat zDist;

    glm::vec3 centerPos;
};

#endif
