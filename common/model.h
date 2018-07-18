#ifndef _MODEL_H
#define _MODEL_H

#include <Eigen/Sparse>

class Model
{
public:
	virtual void updateParam(Param * param) = 0;
};

#endif