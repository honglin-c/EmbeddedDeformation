#include "graphvertex.h"
#include "node.h"

GraphVertex::GraphVertex()
{}

GraphVertex::GraphVertex(glm::vec3 _position):position(_position)
{}

GraphVertex::~GraphVertex()
{}

void GraphVertex::setPosition(glm::vec3 _position)
{
	position = _position;
}

void GraphVertex::setVertexNode(std::vector<VertexNode> _nodes)
{
	nodes = _nodes;
}

void GraphVertex::updatePosition(void)
{

	glm::vec3 newPosition = glm::vec3(0.0f, 0.0f, 0.0f);
	for(size_t i = 0 ; i < nodes.size(); i++)
	{
		newPosition += nodes[i].weight * nodes[i].node->applyMapping(position);
	}
	position = newPosition;
}



