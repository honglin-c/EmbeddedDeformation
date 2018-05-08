#include "node.h"

Node::Node()
{}

Node::Node(glm::vec3 _position):position(_position)
{}

Node::~Node()
{}

void Node::setPosition(glm::vec3 _position)
{
	position = _position;
}

glm::vec3 Node::getPosition() const
{
	return position;
}

glm::vec3 Node::applyMapping(glm::vec3 p)
{
	return rotation * (p - position) + position + translation;
}
