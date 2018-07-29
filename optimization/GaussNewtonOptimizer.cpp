#include "GaussNewtonOptimizer.h"
#include <fstream>

using namespace std;
using namespace Eigen;

GaussNewtonOptimizer::GaussNewtonOptimizer(std::shared_ptr<Param> param):constraint_count(0)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;
	VectorXd delta = VectorXd::Zero(12 * nodes.size());
}

GaussNewtonOptimizer::~GaussNewtonOptimizer()
{}

bool GaussNewtonOptimizer::solveSingleFrame(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param)
{
	bool stopped = false;
	// Initialize
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;
	shared_ptr<AnimateTargetFunction> tf = static_pointer_cast<AnimateTargetFunction, ResidualFunction>(f);
	timestep = tf->getTimestep();
	double Fx = 0.0, Fx_old = 0.0;
  	SimplicialLDLT<SparseMatrix<double>> chol;

  	VectorXd fx;

  	int i = 0;
  	for(i = 0; i < max_iter; i++)
  	{
	  	// Perform a single step in Gauss-Newton Method
		SparseMd Jf = tf->calcJf(xparam);
		fx = tf->calcfx(xparam);

		VectorXd delta = descentDirection(Jf, fx, chol, false);

		Fx_old = Fx;
		Fx = fx.transpose() * fx;

		MatrixXd deltaFx = 2.0 * fx.transpose() * Jf;

		cout << "fabs(Fx - Fx_old) = " << std::fabs(Fx - Fx_old) << " threshold = " <<  epsilon * (1.0 + Fx) << endl;
		cout << "Fx = " << Fx << " Fx_old = " << Fx_old << endl;
		cout << "deltaFx.maxCoeff() = " << deltaFx.maxCoeff() << " threshold = " << std::cbrt(epsilon) * (1.0 + Fx) << endl;
		cout << "delta.maxCoeff()   = " << delta.maxCoeff() << " threshold = " << std::sqrt(epsilon) * (1.0 + delta.maxCoeff()) << endl;

		if(std::fabs(Fx - Fx_old) < epsilon * (1.0 + Fx) /*&&
		   deltaFx.maxCoeff() < std::cbrt(epsilon) * (1.0 + Fx) &&
		   delta.maxCoeff() < std::sqrt(epsilon) * (1.0 + delta.maxCoeff())*/)
		{
			stopped = true;
			cout << "Exit at " << i << "-th iteration" << endl << endl;
			break;
		}

		double alpha = lineSearch(tf, xparam, delta, true);
		cout << "Line Search: alpha = " << alpha << endl;
		updateParam(xparam, alpha * delta, true);
  	}
  	updateFrame(xparam);

 #ifdef DEBUG
		VectorXd fx_Ep = fx.segment(0, tf->kin_begin);
		VectorXd fx_rot = fx.segment(0, tf->reg_begin);
		VectorXd fx_reg = fx.segment(tf->reg_begin, tf->con_begin - tf->reg_begin);
		VectorXd fx_con = fx.segment(tf->con_begin, tf->kin_begin - tf->con_begin);
		VectorXd fx_Ek = fx.segment(tf->kin_begin, tf->gv_begin - tf->kin_begin);
		VectorXd fx_gv = fx.segment(tf->gv_begin, fx.size() - tf->gv_begin);
		double Fx_Ep  = fx_Ep.transpose() * fx_Ep;
		double Fx_rot = fx_rot.transpose() * fx_rot;
		double Fx_reg = fx_reg.transpose() * fx_reg;
		double Fx_con = fx_con.transpose() * fx_con;
		double Fx_Ek = fx_Ek.transpose() * fx_Ek;
		double Fx_gv  = fx_gv.transpose() * fx_gv;
		double samplePos = nodes[4]->getTranslationFrame()[1];
		double sampleVelocity = nodes[4]->getVelocityFrame()[1];
		cout << "Fx     = " << Fx << endl;
		cout << "Fx_Ep  = " << Fx_Ep << endl;
		cout << "Fx_Erot= " << Fx_rot << endl;
		cout << "Fx_Ereg= " << Fx_reg << endl;
		cout << "Fx_Econ= " << Fx_con << endl;
		cout << "Fx_Ek = " << Fx_Ek << endl;
		cout << "samplePos=" << samplePos << endl;
		cout << "sampleVelocity=" << sampleVelocity << endl;
		cout << "Fx_Egv = " << Fx_gv << endl << endl;

		ofstream dataFile("Fx_data.txt", std::fstream::in | std::fstream::out | std::fstream::app);

		if (dataFile.is_open()) 
		{
			dataFile << Fx << " ";
			dataFile << Fx_Ep << " ";
			dataFile << Fx_rot << " ";
			dataFile << Fx_reg << " ";
			dataFile << Fx_con << " ";
			dataFile << Fx_Ek << " ";
			dataFile << samplePos << " ";
			dataFile << sampleVelocity << " ";
			dataFile << Fx_gv << endl;
			dataFile.close();
		} 
#endif

	return stopped;
}

bool GaussNewtonOptimizer::solve(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param)
{
	bool stopped = false;
	// Initialize
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::string modelName = xparam->modelName;
	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;
	// shared_ptr<DeformTargetFunction> tf = static_pointer_cast<DeformTargetFunction, ResidualFunction>(f);
	shared_ptr<DeformTargetFunction> tf = static_pointer_cast<DeformTargetFunction, ResidualFunction>(f);
	double Fx = 0.0, Fx_old = 0.0;
  	SimplicialLDLT<SparseMatrix<double>> chol;

  	int x_order = 12 * nodes.size();
	VectorXd delta = VectorXd::Zero(x_order);

	// Benchmarks
	ofstream fout("../benchmark/" + modelName + "_measure_time.txt");
	clock_t begin, end;
	clock_t solve_time = 0, def_time = 0;

	int i;
	for(i = 0; i < max_iter; i++)
	{
		begin = clock();
		SparseMd Jf = tf->calcJf(xparam);
		VectorXd fx = tf->calcfx(xparam);

		if(i == 0)
			delta = descentDirection(Jf, fx, chol, true);
		else
			delta = descentDirection(Jf, fx, chol, false);

		Fx_old = Fx;

		Fx = fx.transpose() * fx;

		MatrixXd deltaFx = 2.0 * fx.transpose() * Jf;

		if(std::fabs(Fx - Fx_old) < epsilon * (1.0 + Fx) &&
		   deltaFx.maxCoeff() < std::cbrt(epsilon) * (1.0 + Fx) &&
		   delta.maxCoeff() < std::sqrt(epsilon) * (1.0 + delta.maxCoeff()))
		{
			stopped = true;
			break;
		}
		else
		{
			cout << "fabs(Fx - Fx_old)  = " << std::fabs(Fx - Fx_old) << " threshold = " << epsilon * (1.0 + Fx) << endl;
			cout << "deltaFx.maxCoeff() = " << deltaFx.maxCoeff() << " threshold = " << std::cbrt(epsilon) * (1.0 + Fx) << endl;
			cout << "delta.maxCoeff()   = " << delta.maxCoeff() << " threshold = " << std::sqrt(epsilon) * (1.0 + delta.maxCoeff()) << endl;
		}

		end = clock();
		solve_time += end - begin;

#ifdef DEBUG
		VectorXd fx_rot = fx.segment(0, tf->reg_begin);
		VectorXd fx_reg = fx.segment(tf->reg_begin, tf->con_begin - tf->reg_begin);
		VectorXd fx_con = fx.segment(tf->con_begin, fx.size() - tf->con_begin);
		double Fx_rot = fx_rot.transpose() * fx_rot;
		double Fx_reg = fx_reg.transpose() * fx_reg;
		double Fx_con = fx_con.transpose() * fx_con;

		cout << "Fx     = " << Fx << endl;
		cout << "Fx_Erot= " << Fx_rot << endl;
		cout << "Fx_Ereg= " << Fx_reg << endl;
		cout << "Fx_Econ= " << Fx_con << endl;
#endif

		// Record update time
		begin = clock();

		double alpha = lineSearch(tf, xparam, delta, false);
		cout << "Line Search: alpha = " << alpha << endl;
		updateParam(xparam, alpha * delta, false);
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

// void GaussNewtonOptimizer::applyIneritia(std::shared_ptr<Param> param)
// {
// 	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
// 	std::vector<Node *> nodes = xparam->nodes;

// 	VectorXd displacement = VectorXd::Zero(nodes.size() * x_rt);
// 	for(int n_i = 0; n_i < nodes.size(); n_i ++)
// 	{
// 		Vector3d velocity = nodes[n_i]->getVelocity();
// 		for(int ti = 0; ti < 3; ti++)
// 			displacement[n_i * x_rt + 9 + ti] = velocity[ti] * timestep;
// 	}
// 	updateParam(param, 0.00001 * displacement, false);
// }

void GaussNewtonOptimizer::updateFrame(std::shared_ptr<Param> param)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;
	for(auto n: nodes)
	{
		n->setVelocityFrame((n->getTranslation() - n->getTranslationFrame()) / timestep);
		n->updateTranslationFrame();
	}
}

void GaussNewtonOptimizer::updateParam(std::shared_ptr<Param> param, Eigen::VectorXd delta, bool animation)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	// First, update all the nodes's R and t
	int n_i = 0;
	VectorXd rot;
	Vector3d t;
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

		// Update velocity if it is animation
		// if(animation)
		// {
		// 	// n->setVelocity(2 * t / timestep - n->getVelocity());
		// 	n->setVelocity(t / timestep);
		// }
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
// Split fx into fx1(explicit part) and fx2(implicit part)
VectorXd GaussNewtonOptimizer::descentDirection(const Eigen::SparseMatrix<double> &Jf,
												const VectorXd &fx,
												SimplicialLDLT<SparseMd> &chol,
												bool symbolic)
{
	// Solving
	MatrixXd I = MatrixXd::Identity(Jf.cols(), Jf.cols());
  	SparseMd JfTJf = Jf.transpose() * Jf + mu * I;

  	if(symbolic)
  		chol.analyzePattern(JfTJf);
	// Compute the sparse Cholesky Decomposition of Jf^T * Jf
  	chol.compute(JfTJf);
	// get the solution to the given right hand side
  	VectorXd x = chol.solve(-1.0 * Jf.transpose() * fx);

  	if(chol.info() == Eigen::ComputationInfo::NumericalIssue)
  	{
  		std::cout << "ERROR: Cholesky Decompostion Fail! JfTJf is not positive definite" << std::endl;

  		// Calculate all the eigenvalues and catch the negative ones
  		MatrixXd temp = MatrixXd(JfTJf);
  		SelfAdjointEigenSolver<MatrixXd> solver(temp);
  		int size = solver.eigenvalues().size();
  		VectorXd x;
  		for(int k = 0; k < size; k++)
  		{
  			complex<double> result = solver.eigenvalues()(k);
  			if(result.real() < 0.0f)
  			{
  				std::cout << "catch negative eigenvalue (" << k << ") : " << result << std::endl;
  				x = solver.eigenvectors().col(k);
  			}
  		}
  		// use the first 0 eigenvector as the descent direction for debug
  		return x;
  		// std::cout << solver.eigenvalues() << std::endl;
	}
  	return x;
}

VectorXd GaussNewtonOptimizer::descentDirection(const Eigen::SparseMatrix<double> &Jf,
										  		const Eigen::VectorXd &fx_Ep,
										  		const Eigen::SparseMatrix<double> &fx_Ek1, // E_k1 is related to delta
										  		const Eigen::VectorXd &fx_Ek2) // E_k2 is unrelated to delta
{
	SimplicialLDLT<SparseMd> chol;
	// cout << "d1" << endl;
	// cout << "Jf size: " << Jf.rows() << " " << Jf.cols() << endl;
	// cout << "fx_Ek1 size: " << fx_Ek1.size() << endl;
	MatrixXd I = MatrixXd::Identity(Jf.cols(), Jf.cols());
	SparseMd A = Jf.transpose() * Jf + Jf.transpose() * fx_Ek1 + mu * I;
	// cout << "d2" << endl;
	VectorXd b = -1.0 * Jf.transpose() * (fx_Ep + fx_Ek2);
	// cout << "d3" << endl;
	chol.compute(A);
	VectorXd x = chol.solve(b);
	if(chol.info() == Eigen::ComputationInfo::NumericalIssue)
  	{
  		std::cout << "ERROR: Cholesky Decompostion Fail! JfTJf is not positive definite" << std::endl;

  		// Calculate all the eigenvalues and catch the negative ones
  		MatrixXd temp = MatrixXd(A);
  		SelfAdjointEigenSolver<MatrixXd> solver(temp);
  		int size = solver.eigenvalues().size();
  		VectorXd x;
  		for(int k = 0; k < size; k++)
  		{
  			complex<double> result = solver.eigenvalues()(k);
  			if(result.real() < 0.0f)
  			{
  				std::cout << "catch negative eigenvalue (" << k << ") : " << result << std::endl;
  				x = solver.eigenvectors().col(k);
  			}
  		}
  		// use the first 0 eigenvector as the descent direction for debug
  		return x;
  		// std::cout << solver.eigenvalues() << std::endl;
	}
	// cout << "d4" << endl;
	return x;
}

Eigen::VectorXd GaussNewtonOptimizer::descentDirection(const Eigen::SparseMatrix<double> &Jf,
											  		   const Eigen::VectorXd &fx_Ep,
											  		   const Eigen::SparseMatrix<double> &fx_Ek1, // E_k1 is related to delta
											  		   const Eigen::VectorXd &fx_Ek2, // E_k2 is unrelated to delta
											  		   const Eigen::VectorXd &JfTfx_Gv)
{
	SimplicialLDLT<SparseMd> chol;
	MatrixXd I = MatrixXd::Identity(Jf.cols(), Jf.cols());
	SparseMd A = Jf.transpose() * Jf + Jf.transpose() * fx_Ek1 + mu * I;
	// cout << "d2" << endl;
	VectorXd b = -1.0 * (Jf.transpose() * (fx_Ep + fx_Ek2) + JfTfx_Gv);
	// VectorXd b = -1.0 * (Jf.transpose() * (fx_Ep + fx_Ek2) + JfTfx_Gv);
	// cout << "d3" << endl;
	chol.compute(A);
	VectorXd x = chol.solve(b);
	if(chol.info() == Eigen::ComputationInfo::NumericalIssue)
  	{
  		std::cout << "ERROR: Cholesky Decompostion Fail! JfTJf is not positive definite" << std::endl;

  		// Calculate all the eigenvalues and catch the negative ones
  		MatrixXd temp = MatrixXd(A);
  		SelfAdjointEigenSolver<MatrixXd> solver(temp);
  		int size = solver.eigenvalues().size();
  		VectorXd x;
  		for(int k = 0; k < size; k++)
  		{
  			complex<double> result = solver.eigenvalues()(k);
  			if(result.real() < 0.0f)
  			{
  				std::cout << "catch negative eigenvalue (" << k << ") : " << result << std::endl;
  				x = solver.eigenvectors().col(k);
  			}
  		}
  		// use the first 0 eigenvector as the descent direction for debug
  		return x;
  		// std::cout << solver.eigenvalues() << std::endl;
	}
	// cout << "d4" << endl;
	return x;
}

// Perform a Line Search
double GaussNewtonOptimizer::lineSearch(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param, Eigen::VectorXd delta, bool animation)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	shared_ptr<DeformTargetFunction> tf;
	if(animation)
		tf = static_pointer_cast<AnimateTargetFunction, ResidualFunction>(f);
	else 
		tf = static_pointer_cast<DeformTargetFunction, ResidualFunction>(f);

	double alpha = tf->line_search_upper_bound, p = 0.5;
	double Fx0, Fxi = 0.0;
	VectorXd JF0, JFi;
	Fx0 = tf->calcFx(xparam);
	JF0 = tf->calcJF(xparam); 

	for(int i = 0; i < tf->line_search_iter; i++)
	{
		updateParam(xparam, alpha * delta, true);
		Fxi = tf->calcFx(xparam);
		JFi = tf->calcJF(xparam);
		updateParam(xparam, -alpha * delta, true); // restore xparam
		if(Fxi <= Fx0 + c1 * alpha * JF0.transpose() * delta 
		&& JFi.transpose() * delta >= c2 * JF0.transpose() * delta)
		{
			return alpha;
		}
		alpha = p * alpha;
	}

	return alpha;
}

// Zoom using bisection interpolation 
// Reference: Numerical Optimization P61 Algorithm 3.6
double GaussNewtonOptimizer::zoom(std::shared_ptr<ResidualFunction> f,
								  std::shared_ptr<Param> param, 
								  Eigen::VectorXd delta,
								  double left, double right)
{

}

void GaussNewtonOptimizer::debug(const std::string s)
{
	std::cout << s << std::endl;
}