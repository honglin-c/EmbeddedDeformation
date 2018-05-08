#include "kdnode.h"
#include <iostream>
#include <list>
#include <glm/vec3.hpp>

KdNode::KdNode() :
        leftChild(nullptr),
        rightChild(nullptr)
{}

KdNode::KdNode(bool _leaf)
              :leaf(_leaf),
               leftChild(nullptr),
               rightChild(nullptr)
{}

void KdNode::setLeftChild(KdNode * node)
{
    leftChild = node;
}

void KdNode::setRightChild(KdNode * node)
{
    rightChild = node;
}

bool KdNode::isLeafNode()
{
    return leaf;
}

void KdNode::setLeaf(Node * node)
{
    //std::cout << "Set as leaf" << std::endl;
    this->leaf = true;
    this->node = node;
}

void KdNode::setSplitPlane(int _axis, GLfloat _plane)
{
    splitPlane = _plane;
    axis = _axis;
}


void KdNode::setNodeBoundingVolume(AABB &aabb)
{
    this->setNodeBoundingVolume(aabb.negX, aabb.posX, aabb.negY, aabb.posY, aabb.negZ, aabb.posZ);
}

void KdNode::setNodeBoundingVolume(GLfloat MinX, GLfloat MaxX, GLfloat MinY, GLfloat MaxY, GLfloat MinZ, GLfloat MaxZ)
{
    boundingVolume.xDist = glm::abs(MinX - MaxX);
    boundingVolume.yDist = glm::abs(MinY - MaxY);
    boundingVolume.zDist = glm::abs(MinZ - MaxZ);

    GLfloat xC = (MaxX + MinX) * 0.5f;
    GLfloat yC = (MaxY + MinY) * 0.5f;
    GLfloat zC = (MaxZ + MinZ) * 0.5f;

    boundingVolume.centerPos = glm::vec3(xC, yC, zC);

    boundingVolume.negX = MinX - FLT_EPSILON;
    boundingVolume.negY = MinY - FLT_EPSILON;
    boundingVolume.negZ = MinZ - FLT_EPSILON;

    boundingVolume.posX = MaxX + FLT_EPSILON;
    boundingVolume.posY = MaxY + FLT_EPSILON;
    boundingVolume.posZ = MaxZ + FLT_EPSILON;
}

void KdNode::addChild(KdNode *child)
{}

void KdNode::removeFromParent()
{}
