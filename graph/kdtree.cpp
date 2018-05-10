#include "kdtree.h"
#include <iostream>
#include <algorithm>

using std::cout;
using std::endl;

int axis; // current split axis

KdTree::KdTree()
{}

KdTree::~KdTree()
{}

bool KdTree::compareNode(Node * a, Node * b)
{
	glm::vec3 apos = a->getPosition();
	glm::vec3 bpos = b->getPosition();
	return (apos[axis] < bpos[axis]);
}

KdNode * KdTree::buildKdTree(std::vector<Node *> nodes, AABB aabb, int depth = 0)
{
	// cout << "depth: " << depth << endl;
	// for(auto t:nodes){
	// 	glm::vec3 pos = t->getPosition();
	// 	cout << pos[0] << "," << pos[1] << "," << pos[2] << " ";
	// }
	// cout << endl;
	// Empty node list
	if(nodes.size() == 0)
		return nullptr;
	// Create a leaf node
	if(nodes.size() == 1)
	{
		KdNode * leafnode = new KdNode(true);
		leafnode->setLeaf(nodes.front());
		leafnode->setNodeBoundingVolume(aabb);
		if(depth == 0) this->rootNode = leafnode;
		print(leafnode);
		return leafnode;
	}

	axis = depth % 3;

	std::nth_element(nodes.begin(), nodes.begin() + nodes.size() / 2 - 1, nodes.end(), this->compareNode);

	GLfloat plane = (nodes[nodes.size() / 2 - 1]->getPosition())[axis];

	std::vector<Node *> leftList;
	std::vector<Node *> rightList;

	glm::vec3 pos;
	for(size_t i = 0; i < nodes.size(); i++)
	{
		pos = nodes[i]->getPosition();
		if(pos[axis] <= plane)
			leftList.push_back(nodes[i]);
		else if(pos[axis] > plane)
			rightList.push_back(nodes[i]);
	}

	// Create a split node
	AABB leftaabb, rightaabb;
	aabb.splitAABB(axis, plane, leftaabb, rightaabb);

	KdNode * leftchild = buildKdTree(leftList, leftaabb, depth + 1);
	KdNode * rightchild = buildKdTree(rightList, rightaabb, depth + 1);
	KdNode * vnode = new KdNode(false);
	
	vnode->setLeftChild(leftchild);
	vnode->setRightChild(rightchild);
	vnode->setNodeBoundingVolume(aabb);
	vnode->setSplitPlane(axis, plane);

	// print(vnode);

	if(depth == 0)
		this->rootNode = vnode;

	return vnode;
}

std::vector<Node *> KdTree::kNearestNeighborSearch(glm::vec3 pos)
{
	std::priority_queue<Node *, std::vector<Node *>, distCompare> heap(pos);
	this->kNearestNeighborSearch(pos, this->rootNode, heap, Infinity);

	std::vector<Node *> res;
	Node * node = nullptr;
	// get the top 4 elements from the heap
	for(int i = 0; i < 4 && !heap.empty(); i++)
	{
		node = heap.top();
		heap.pop();
		res.push_back(node);
	}
	return res;
}

void KdTree::kNearestNeighborSearch(const glm::vec3 pos, 
									KdNode * kdnode, 
									std::priority_queue<Node *, std::vector<Node *>, distCompare> &heap, 
									GLfloat dist)
{
	if(kdnode->leaf)
	{
		heap.push(kdnode->node);
		return;
	}

	if(dist == Infinity)
	{
		if(pos[kdnode->axis] <= kdnode->splitPlane)
		{
			kNearestNeighborSearch(pos, kdnode->leftChild, heap, dist);
			dist = glm::length(heap.top()->getPosition() - pos);
			if(pos[kdnode->axis] + dist > kdnode->splitPlane)
				kNearestNeighborSearch(pos, kdnode->rightChild, heap, dist);
		}
		else
		{
			kNearestNeighborSearch(pos, kdnode->rightChild, heap, dist);
			dist = glm::length(heap.top()->getPosition() - pos);
			if(pos[kdnode->axis] - dist <= kdnode->splitPlane)
				kNearestNeighborSearch(pos, kdnode->leftChild, heap, dist);
		}
	}
	else
	{
		if(pos[kdnode->axis] - dist <= kdnode->splitPlane)
		{
			kNearestNeighborSearch(pos, kdnode->leftChild, heap, dist);
			dist = glm::length(heap.top()->getPosition() - pos);			
		}
		if(pos[kdnode->axis] + dist > kdnode->splitPlane)
		{
			kNearestNeighborSearch(pos, kdnode->rightChild, heap, dist);
			dist = glm::length(heap.top()->getPosition() - pos);		
		}
	}
}

void KdTree::draw()
{
	this->print(this->rootNode);
}

void KdTree::print(KdNode * node)
{
	const char axisname[3] = {'x', 'y', 'z'};
	if(node == nullptr)
		return;
	cout << axisname[node->axis] <<  " " << node->splitPlane << endl;
	cout << node->node->getPosition()[0] << " " << node->node->getPosition()[1] << " " << node->node->getPosition()[2] << endl;
	cout << "left child:" << (node->leftChild == nullptr) << endl;
	this->print(node->leftChild);
	cout << "right child:" << (node->rightChild == nullptr) << endl;
	this->print(node->rightChild);
}

