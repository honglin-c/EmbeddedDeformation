#ifndef _DIST_COMPARE_H
#define _DIST_COMPARE_H

#include <vector>
#include <queue>

#include <glm/vec3.hpp>
#include "node.h"

class distCompare
{
public:
	distCompare(glm::vec3 _position = glm::vec3(0.0f, 0.0f, 0.0f))
	{
		position = _position;
	}
	bool operator() (const Node * lhs, const Node * rhs)
	{
		return (glm::length(lhs->getPosition() - position) > glm::length(rhs->getPosition() - position));
	}
private:
	glm::vec3 position;
};

#endif