#ifndef _DIST_COMPARE_H
#define _DIST_COMPARE_H

#include <vector>
#include <queue>

#include <Eigen/Dense>
#include "node.h"

using Eigen::Vector3d;

class distCompare
{
public:
	distCompare(Vector3d _position = Vector3d(0.0f, 0.0f, 0.0f))
	{
		position = _position;
	}
	bool operator() (const Node * lhs, const Node * rhs)
	{
		return ((lhs->getPosition() - position).norm() > (rhs->getPosition() - position).norm());
	}
private:
	Vector3d position;
};

#endif