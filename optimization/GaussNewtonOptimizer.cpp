#include "GaussNewtonOptimizer.h"
#include <fstream>

using namespace std;
using namespace Eigen;

GaussNewtonOptimizer::GaussNewtonOptimizer():constraint_count(0)
{}	

GaussNewtonOptimizer::~GaussNewtonOptimizer()
{}

bool GaussNewtonOptimizer::solve(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param)
{
	bool stopped = false;
	// Initialize
	shared_ptr<XParam> xparam = static_pointer_cast<XParam, Param>(param);
	std::string modelName = xparam->modelName;
	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;
	shared_ptr<TargetFunction> tf = static_pointer_cast<TargetFunction, ResidualFunction>(f);
	double Fx = 0.0, Fx_old = 0.0;
  	SimplicialLDLT<SparseMatrix<double>> chol;

  	int x_order = 12 * nodes.size();
	VectorXd delta(x_order);

	// Benchmarks
	ofstream fout("../benchmark/" + modelName + "_measure_time.txt");
	clock_t begin, end;
	clock_t solve_time = 0, def_time = 0;

	int i;
	for(i = 0; i < max_iter; i++)
	{
		begin = clock();
		SparseMd Jf = f->calcJf(param);
		VectorXd fx = f->calcfx(param);

		if(i == 0)
			delta = descentDirection(Jf, fx, chol, true);
		else
			delta = descentDirection(Jf, fx, chol, false);

		Fx_old = Fx;

		Fx = fx.transpose() * fx;

		MatrixXd deltaFx = 2.0 * fx.transpose() * Jf;

		if(std::fabs(Fx - Fx_old) < epsilon * (1.0 + Fx)
			&& deltaFx.maxCoeff() < 1e-2 * (1.0 + Fx)
			&& delta.maxCoeff() < 1e-3 * (1.0 + delta.maxCoeff()))
		{
			stopped = true;
		}

		end = clock();
		solve_time += end - begin;

		// Record update time
		begin = clock();
		updateParam(param, delta);
		end = clock();
		def_time += end - begin;
	}

	double avg_solve_time = (double)solve_time / (CLOCKS_PER_SEC * i);
	double avg_def_time = (double)def_time / (CLOCKS_PER_SEC * i);

	if(fout.is_open())
	{
		fout << "vertex num: " << vertices.size() << endl;
		fout << "node num: " << nodes.size() << endl;
		fout << "constraint_count: " << constraint_count << endl;
		fout << "avg solve time: " << avg_solve_time * 1e3 << endl;
		fout << "avg def time: " << avg_def_time * 1e3 << endl;
		fout << "iteration: " << i << endl;
	}
	fout.close();
	return stopped;
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
	// get the solution to the given right hand side
  	VectorXd x = chol.solve(-1.0 * Jf.transpose() * fx);
  	return x;
}

void GaussNewtonOptimizer::debug(const std::string s)
{
	std::cout << s << std::endl;
}