#ifndef _KDTREE_H
#define _KDTREE_H

#include <vector>
#include <queue>

#include "boundingObject.h"
#include "node.h"
#include "kdnode.h"

class distCompare
{
public:
	distCompare(glm::vec3 _position = glm::vec3(0.0f, 0.0f, 0.0f))
	{
		position = _position;
	}
	bool operator() (const Node * lhs, const Node * rhs)
	{
		return (glm::length(lhs->getPosition() - position) < glm::length(rhs->getPosition() - position));
	}
private:
	glm::vec3 position;
};

class KdTree 
{
public:
	KdTree();
	~KdTree();
	KdNode * buildKdTree(std::vector<Node *> nodes, AABB aabb, int depth);
	static bool compareNode(Node * a, Node * b);
	std::vector<Node *> kNearestNeighborSearch(glm::vec3 pos);
	// k nearest neighbor search
	void kNearestNeighborSearch(glm::vec3 pos, KdNode * kdnode, std::priority_queue<Node *> &heap, GLfloat dist);
	void draw();
	void print(KdNode * node);

private:
	const float Infinity = 1e8;
	KdNode * rootNode;
	GLfloat getSplitPlane(AABB aabb, int axis);
};

#endif