#ifndef _GAUSS_NEWTON_OPTIMIZER_H
#define _GAUSS_NEWTON_OPTIMIZER_H

#include "../common/residualFunction.h"
#include "../common/param.h"
#include "../common/GaussNewtonSolver.h"
#include "../graph/graphVertex.h"
#include "../graph/node.h"
#include "../graph/deformTargetFunction.h"
#include "../animation/animateTargetFunction.h"

#include <memory>
#include <iostream>

#include <Eigen/SparseCholesky>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#define DEBUG

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SimplicialLDLT;
using Eigen::SelfAdjointEigenSolver;
using Eigen::SparseVector;

class GaussNewtonOptimizer: public GaussNewtonSolver
{
public:
	GaussNewtonOptimizer(std::shared_ptr<Param> param);
	~GaussNewtonOptimizer();
	bool solve(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param);
	bool solveSingleStep(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param);

private:
	const int 	 max_iter = 12;
	const double epsilon = 1e-6;
	const int 	 x_rt = 12;
	const double timestep = 0.1;
	const double c1 = 1e-4;
	const double c2 = 0.9;
	const double mu = 1e-7;

	void updateParam(std::shared_ptr<Param> param, Eigen::VectorXd delta, bool animation);

	Eigen::VectorXd descentDirection(const Eigen::SparseMatrix<double> &Jf,
							  		 const Eigen::VectorXd &fx,
							  		 SimplicialLDLT<Eigen::SparseMatrix<double>> &chol,
							 		 bool symbolic);

	Eigen::VectorXd descentDirection(const Eigen::SparseMatrix<double> &Jf,
							  		 const Eigen::VectorXd &fx_Ep,
							  		 const Eigen::SparseMatrix<double> &fx_Ek1, // E_k1 is related to delta
							  		 const Eigen::VectorXd &fx_Ek2); // E_k2 is unrelated to delta
	// Perform a Line Search
	double lineSearch(std::shared_ptr<ResidualFunction> f, 
					  std::shared_ptr<Param> param,
					  Eigen::VectorXd delta);
	// Zoom using bisection interpolation 
	// Reference: Numerical Optimization P61 Algorithm 3.6
	double zoom(std::shared_ptr<ResidualFunction> f,
				std::shared_ptr<Param> param, 
				Eigen::VectorXd delta,
				double left, double right);
	void debug(const std::string s);
	int constraint_count; // use to measure benchmark
};

#endif