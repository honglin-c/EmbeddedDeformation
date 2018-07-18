#ifndef _RESIDUAL_FUNCTION_H
#define _RESIDUAL_FUNCTION_H

#include <Eigen/Sparse>
#include <Eigen/Dense>

#include <memory>

class Param;

class ResidualFunction
{
public:
	virtual Eigen::SparseMatrix<double> calcJf(std::shared_ptr<Param> param) = 0;
	virtual Eigen::VectorXd 			calcfx(std::shared_ptr<Param> param) = 0;
};

#endif