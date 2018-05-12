#include "graphVertex.h"
#include "node.h"
#include "../mesh.h"

GraphVertex::GraphVertex(): isFixed(false), isHandled(false)
{}

GraphVertex::GraphVertex(glm::vec3 _position):position(_position), 
											  position_init(_position),
											  isFixed(false), 
											  isHandled(false)
{}

GraphVertex::GraphVertex(struct Vertex _vertex):position(_vertex.Position),
												normal(_vertex.Normal), 
												position_init(_vertex.Position),
												normal_init(_vertex.Normal), 
												isFixed(false), 
												isHandled(false)
{}

GraphVertex::~GraphVertex()
{}

void GraphVertex::setPositionAndNormal(glm::vec3 _position, glm::vec3 _normal)
{
	position = _position;
	normal = _normal;
}

glm::vec3 GraphVertex::getPosition() const
{
	return position;
}

glm::vec3 GraphVertex::getNormal() const
{
	return normal;
}


void GraphVertex::setNodes(std::vector<Node *> _nodes, std::vector<float> _weights)
{
	nodes = _nodes;
	weights = _weights;
}

void GraphVertex::updatePosition(void)
{
	if(!isFixed)
	{
		glm::vec3 newPosition = glm::vec3(0.0f, 0.0f, 0.0f);
		for(size_t i = 0 ; i < nodes.size(); i++)
		{
			newPosition += weights[i] * nodes[i]->applyMapping(position);
		}
		position = newPosition;
	}
}

std::vector<Node *> GraphVertex::getNodes()
{
	return nodes;
}

std::vector<float> GraphVertex::getWeights()
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
}

float GraphVertex::getConValue()
{
	if(!isFixed && isHandled)
		return glm::length(position - user_position);
	else
		return 0.0f;
}

void GraphVertex::userSetPosition(glm::vec3 _user_position)
{
	if(!isFixed)
	{
		position_init = position = user_position = _user_position;
		isHandled = true;
	}
}


// Get vertex's [ture position - user-specific position]
Vector3f GraphVertex::getConTerm()
{
	if(!isFixed && isHandled)
	{
		glm::vec3 conTerm = position - user_position;
		return Vector3f(conTerm[0], conTerm[1], conTerm[2]);
	}
	else
		return Vector3f(0.0f, 0.0f, 0.0f);
}









