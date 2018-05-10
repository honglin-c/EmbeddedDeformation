#ifndef _KDTREE_H
#define _KDTREE_H

#include <vector>

#include "distCompare.h"
#include "boundingObject.h"
#include "node.h"
#include "kdnode.h"

// Not currently working

class KdTree 
{
public:
	KdTree();
	~KdTree();
	KdNode * buildKdTree(std::vector<Node *> nodes, AABB aabb, int depth);
	static bool compareNode(Node * a, Node * b);
	std::vector<Node *> kNearestNeighborSearch(glm::vec3 pos);
	// k nearest neighbor search
	void kNearestNeighborSearch(const glm::vec3 pos, 
								KdNode * kdnode, 
								std::priority_queue<Node *, std::vector<Node *>, distCompare> &heap, 
								GLfloat dist);
	void draw();
	void print(KdNode * node);

private:
	const float Infinity = 1e8;
	KdNode * rootNode;
	GLfloat getSplitPlane(AABB aabb, int axis);
};

#endif