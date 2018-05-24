#include "graphVertex.h"
#include "node.h"
#include "../mesh.h"

GraphVertex::GraphVertex(): isFixed(false), isHandled(false)
{}

GraphVertex::GraphVertex(Vector3d _position):position(_position),
											  position_init(_position),
											  isFixed(false),
											  isHandled(false)
{
}

GraphVertex::GraphVertex(struct Vertex _vertex):isFixed(false),
												isHandled(false)
{
	position = Vector3d(_vertex.Position[0], _vertex.Position[1], _vertex.Position[2]);
	normal = Vector3d(_vertex.Normal[0], _vertex.Normal[1], _vertex.Normal[2]);
	position_init = position;
	normal_init = normal;
}

GraphVertex::~GraphVertex()
{}

void GraphVertex::setPositionAndNormal(Vector3d _position, Vector3d _normal)
{
	position = _position;
	normal = _normal;
}

Vector3d GraphVertex::getPosition() const
{
	return position;
}

Vector3d GraphVertex::getNormal() const
{
	return normal;
}


void GraphVertex::setNodes(std::vector<Node *> _nodes, std::vector<double> _weights)
{
	nodes = _nodes;
	weights = _weights;
}

void GraphVertex::updatePosition(void)
{
	Vector3d newPosition = Vector3d(0.0, 0.0, 0.0);
	for(size_t i = 0 ; i < nodes.size(); i++)
	{
		newPosition += weights[i] * nodes[i]->applyMapping(position_init);
	}
	position = newPosition;
}

std::vector<Node *> GraphVertex::getNodes()
{
	return nodes;
}

std::vector<double> GraphVertex::getWeights()
{
	return weights;
}

void GraphVertex::updateNeighbor()
{
	for(auto m:nodes)
	{
		for(auto n:nodes)
		{
			if(m != n)
				m->addNeighbor(n);
		}
	}
}

void GraphVertex::setFixed(bool is_fixed)
{
	isFixed = is_fixed;
	user_position = position;
}

void GraphVertex::setHandled(bool is_Handled)
{
	isHandled = is_Handled;
}


// Deprecated
double GraphVertex::getConValue()
{
	if(isFixed || isHandled)
		return (position - user_position).norm();
	else
		return 0.0;
}

void GraphVertex::userSetPosition(Vector3d _user_position)
{
	if(!isFixed)
	{
		user_position = _user_position;
		isHandled = true;
	}
}


// Get vertex's [ture position - user-specific position]
Vector3d GraphVertex::getConTerm()
{
	if(isFixed || isHandled)
	{
		Vector3d conTerm = position - user_position;
		return conTerm;
	}
	else
		return Vector3d(0.0, 0.0, 0.0);
}