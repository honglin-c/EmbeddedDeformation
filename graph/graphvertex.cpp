#include "graphvertex.h"
#include "node.h"
#include "../mesh.h"

GraphVertex::GraphVertex()
{}

GraphVertex::GraphVertex(glm::vec3 _position):position(_position)
{}

GraphVertex::GraphVertex(struct Vertex _vertex):position(_vertex.Position), normal(_vertex.Normal)
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

	glm::vec3 newPosition = glm::vec3(0.0f, 0.0f, 0.0f);
	for(size_t i = 0 ; i < nodes.size(); i++)
	{
		newPosition += weights[i] * nodes[i]->applyMapping(position);
	}
	position = newPosition;
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




