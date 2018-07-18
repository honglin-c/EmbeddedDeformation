#ifndef _GAUSS_NEWTON_OPTIMIZER_H
#define _GAUSS_NEWTON_OPTIMIZER_H

#include "../common/residualFunction.h"
#include "../common/param.h"
#include "../common/GaussNewtonSolver.h"
#include "../graph/graphVertex.h"
#include "../graph/node.h"
#include "targetFunction.h"

#include <memory>
#include <iostream>

#include <Eigen/SparseCholesky>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SimplicialLDLT;
using Eigen::SelfAdjointEigenSolver;
using Eigen::SparseVector;

class GaussNewtonOptimizer: public GaussNewtonSolver
{
public:
	GaussNewtonOptimizer();
	~GaussNewtonOptimizer();
	bool solve(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param);

private:
	const int 	 max_iter = 20;
	const double epsilon = 1e-6;
	const int 	 x_rt = 12;

	void updateParam(std::shared_ptr<Param> param, Eigen::VectorXd delta);

	Eigen::VectorXd descentDirection(const Eigen::SparseMatrix<double> &Jf, 
							  		 const Eigen::VectorXd &fx, 
							  		 SimplicialLDLT<Eigen::SparseMatrix<double>> &chol, 
							 		 bool symbolic);
	void debug(const std::string s);
	int constraint_count; // use to measure benchmark
};

#endif