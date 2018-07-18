#ifndef _PARAM_X_H
#define _PARAM_X_H

#include <Eigen/Sparse>
#include <vector>

class GraphVertex;
class Node;

class Param
{
public:

};

class XParam: public Param
{
public:
	XParam(){}
	~XParam(){}
	void setVerticesAndNodes(std::vector<GraphVertex *> &vertices, std::vector<Node *> &nodes)
	{
		this->vertices = vertices;
		this->nodes = nodes;
	}
	std::vector<GraphVertex *> vertices;
	std::vector<Node *> nodes;
};

#endif