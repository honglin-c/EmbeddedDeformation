#ifndef _KDNODE_H
#define _KDNODE_H

#include <cfloat>
#include <GL/glew.h>
#include <vector>

#include "boundingObject.h"
#include "node.h"

class KdNode
{

public:
    bool leaf;

    // Axis
    enum {X_AXIS, Y_AXIS, Z_AXIS};
    int axis;
    // the value that split the child node
    GLfloat splitPlane; 

    // A node has a bounding volume that is large enough to enclose all of the objects that are inside this or the children of this node.
    AABB boundingVolume;

    // References to it children.
    KdNode * leftChild;
    KdNode * rightChild;

    // the node it contains.
    Node * node;

    KdNode();

    KdNode(bool _isleaf);

    // Node(int i);

    void setLeftChild(KdNode *node);

    void setRightChild(KdNode *node);

    bool isLeafNode();

    void setLeaf(Node * node);

    void setSplitPlane(int _axis, GLfloat _plane);

    void setNodeBoundingVolume(AABB &aabb);

    void setNodeBoundingVolume(GLfloat MinX, GLfloat MaxX, GLfloat MinY, GLfloat MaxY, GLfloat MinZ, GLfloat MaxZ);

    void addChild(KdNode *child);

    void removeFromParent();
};

#endif
