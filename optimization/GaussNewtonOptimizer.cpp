#include "GaussNewtonOptimizer.h"

using namespace std;
using namespace Eigen;

GaussNewtonOptimizer::GaussNewtonOptimizer()
{}	

GaussNewtonOptimizer::~GaussNewtonOptimizer()
{}

bool GaussNewtonOptimizer::solve(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param)
{
	// Initialize
	shared_ptr<XParam> xparam = static_pointer_cast<XParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	shared_ptr<TargetFunction> tf = static_pointer_cast<TargetFunction, ResidualFunction>(f);
	// this->reg_begin = tf->reg_begin;
	// this->con_begin = tf->con_begin;

	double Fx = 0.0, Fx_old = 0.0;
  	SimplicialLDLT<SparseMatrix<double>> chol;

  	int x_order = 12 * nodes.size();
	VectorXd delta(x_order);

	for(int i = 0; i < max_iter; i++)
	{
		debug("1");
		SparseMd Jf = f->calcJf(param);

		debug("1.1");
		VectorXd fx = f->calcfx(param);

		debug("1.2");
		if(i == 0)
			delta = descentDirection(Jf, fx, chol, true);
		else
			delta = descentDirection(Jf, fx, chol, false);

		debug("2");

		Fx_old = Fx;

		Fx = fx.transpose() * fx;

		debug("2.5");

		MatrixXd deltaFx = 2.0 * fx.transpose() * Jf;

		debug("3");

		if(std::fabs(Fx - Fx_old) < epsilon * (1.0 + Fx)
			&& deltaFx.maxCoeff() < 1e-2 * (1.0 + Fx)
			&& delta.maxCoeff() < 1e-3 * (1.0 + delta.maxCoeff()))
		{
			return true;
		}
		updateParam(param, delta);

		debug("6");
	}
	return false;
}

void GaussNewtonOptimizer::updateParam(std::shared_ptr<Param> param, Eigen::VectorXd delta)
{
	shared_ptr<XParam> xparam = static_pointer_cast<XParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	// First, update all the nodes's R and t
	int n_i = 0;
	VectorXd rot;
	VectorXd t;
	Matrix3d delta_rotation;
	Vector3d delta_translation;
	for(auto n:nodes)
	{
		rot = delta.segment(n_i * x_rt, 9);
		t = delta.segment(n_i * x_rt + 9, 3);

	 	delta_rotation << rot(0), rot(1), rot(2),
	 					  rot(3), rot(4), rot(5),
	 					  rot(6), rot(7), rot(8);
		delta_translation = t;

		n->addDeltaRotation(delta_rotation);
		n->addDeltaTranslation(delta_translation);
		n_i++;
	}
	
	// Second, update all the vertices's position and normal
	// transform vertex position and normal using the current transformation
	Vector3d position, normal;
	for(auto v:vertices)
	{
		position = normal = Vector3d(0.0, 0.0, 0.0);
		for(int i = 0; i < v->nodes.size(); i++)
		{
			position += v->weights[i] * v->nodes[i]->transformPosition(v->position_init);
			normal += v->weights[i] * v->nodes[i]->transformNormal(v->normal_init);
		}
		v->setPositionAndNormal(position, normal);
	}
}


// Use cholesky decomposition to calculate the descent direction
VectorXd GaussNewtonOptimizer::descentDirection(const Eigen::SparseMatrix<double> &Jf, 
												const VectorXd &fx, 
												SimplicialLDLT<SparseMd> &chol, 
												bool symbolic)
{
	// Solving
  	SparseMd JfTJf = Jf.transpose() * Jf;

  	if(symbolic)
  		chol.analyzePattern(JfTJf);
	// Compute the sparse Cholesky Decomposition of Jf^T * Jf
  	chol.compute(JfTJf);

  	// For debugging: Catch error when Cholesky decomposition fail
  	// if(chol.info() == Eigen::ComputationInfo::NumericalIssue)
  	// {
  	// 	std::cout << "ERROR: Cholesky Decompostion Fail! JfTJf is not positive definite" << std::endl;

  	// 	for(int i = 0; i < JfTJf.rows(); i++)
  	// 	{
  	// 		VectorXd this_row = JfTJf.row(i);
  	// 		double norm2 = this_row.norm();
  	// 		if(norm2 < 1e-8)
  	// 		{
  	// 			std::cout << "Catch rows with nearly 0 norm: " << i << "  " << norm2 << std::endl;
  	// 			int count = 0;
  	// 			for(auto v:vertices)
  	// 			{
  	// 				if(v->isFixed || v->isHandled)
  	// 				{
  	// 					if(count + con_begin + 4 > i)
  	// 					{
  	// 						std::cout << "weight: " << v->weights[i - count - reg_end] << std::endl;
  	// 						break;
  	// 					}
  	// 					else
  	// 						count += 4;
  	// 				}
  	// 			}
  	// 		}
  	// 	} 

  	// 	// Calculate all the eigenvalues and catch the negative ones
  	// 	MatrixXd temp = MatrixXd(JfTJf);
  	// 	SelfAdjointEigenSolver<MatrixXd> solver(temp);
  	// 	int size = solver.eigenvalues().size();
  	// 	VectorXd x;
  	// 	for(int k = 0; k < size; k++)
  	// 	{
  	// 		complex<double> result = solver.eigenvalues()(k);
  	// 		if(result.real() < 0.0)
  	// 		{
  	// 			std::cout << "catch negative eigenvalue (" << k << ") : " << result << std::endl;
  	// 			x = solver.eigenvectors().col(k);
  	// 		}
  	// 	}
  	// 	// use the first 0 eigenvector as the descent direction for debug
  	// 	return x;
  	// }

	// get the solution to the given right hand side
  	VectorXd x = chol.solve(-1.0 * Jf.transpose() * fx);
  	return x;
}

void GaussNewtonOptimizer::debug(const std::string s)
{
	std::cout << s << std::endl;
}