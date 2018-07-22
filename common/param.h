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

class DeformParam: public Param
{
public:
	DeformParam(){}
	~DeformParam(){}
	void setParamInfo(const std::string &modelName, std::vector<GraphVertex *> &vertices, std::vector<Node *> &nodes)
	{
		this->modelName = modelName;
		this->vertices = vertices;
		this->nodes = nodes;
	}
	std::string modelName;
	std::vector<GraphVertex *> vertices;
	std::vector<Node *> nodes;
};

class AnimateParam: public Param
{
public:
	AnimateParam(){}
	~AnimateParam(){}
	void setParamInfo(std::vector<Eigen::Vector3d> old_positions, std::vector<Node *> nodes)
	{
		this->old_positions = old_positions;
		this->nodes = nodes;
	}
	std::vector<Eigen::Vector3d> old_positions;
	std::vector<Node *> nodes;
};

#endif