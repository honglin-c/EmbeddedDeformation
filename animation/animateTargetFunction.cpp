#include "animateTargetFunction.h"
#include <iostream>

using namespace Eigen;
using namespace std;

AnimateTargetFunction::AnimateTargetFunction(std::shared_ptr<Param> param): DeformTargetFunction(param)
{
	setOrder(param);
	gravity = g * w_gv;
}

AnimateTargetFunction::~AnimateTargetFunction()
{
}

void AnimateTargetFunction::setTimestep(double timestep)
{
	this->timestep = timestep;
}

Eigen::SparseMatrix<double> AnimateTargetFunction::calcJf(std::shared_ptr<Param> param)
{
	SparseMatrix<double> Jf(fx_order, x_order);

	vector<Tf> tripletList;
	tripletList.reserve(0.1 * fx_order * x_order);

	// E_rot
	// row range: from 0 to nodes.size() * 6 - 1
	calcJfRot(param, tripletList);

	// E_reg
	// row range: from nodes.size() * 6 to nodes.size() * 6 + nodes.size() * neighbor(n).size()
	calcJfReg(param, tripletList);

	// E_con
	// row range: from nodes.size() * (6 + N(nodes).size())
	//            to nodes.size() * (6 + N(nodes).size()) + vertices.size()
	calcJfCon(param, tripletList);

	// E_kin
	// row range: from nodes.size() * (6 + N(nodes).size()) + vertices.size() + 1
	// 			  to nodes.size() * (6 + N(nodes).size()) + vertices.size() + nodes.size() * 3
	calcJfKin(param, tripletList);

	// E_gv
	// row range: from nodes.size() * (6 + N(nodes).size()) + vertices.size() + nodes.size() * 3 + 1
	// 			  to nodes.size() * (6 + N(nodes).size()) + vertices.size() + nodes.size() * 6 + 1
	calcJfGv(param, tripletList);

	Jf.setFromTriplets(tripletList.begin(), tripletList.end());

	return Jf;
}

Eigen::VectorXd AnimateTargetFunction::calcfx(std::shared_ptr<Param> param)
{
	VectorXd fx(fx_order);
	int index = 0;
	// Erot
	calcfxRot(param, fx);
	// Ereg
	calcfxReg(param, fx);
	// Econ
	calcfxCon(param, fx);
	// Ekin
	calcfxKin(param, fx);
	// Eg
	calcfxGv(param, fx);
	return fx;
}

double AnimateTargetFunction::calcFx(std::shared_ptr<Param> param)
{
	VectorXd fx = calcfx(param);
	return fx.transpose() * fx;
}

VectorXd AnimateTargetFunction::calcJF(std::shared_ptr<Param> param)
{
	SparseMatrix<double> Jf = calcJf(param);
	VectorXd fx = calcfx(param);
	return 2.0 * Jf.transpose() * fx;
}

Eigen::VectorXd AnimateTargetFunction::calcfx_Ep(std::shared_ptr<Param> param)
{
	VectorXd fx_Ep = VectorXd::Zero(fx_order);
	int index = 0;
	// Erot
	calcfxRot(param, fx_Ep);
	// Ereg
	calcfxReg(param, fx_Ep);
	// Econ
	calcfxCon(param, fx_Ep);

	return fx_Ep;
}

Eigen::SparseMatrix<double> AnimateTargetFunction::calcfx_Ek1(std::shared_ptr<Param> param)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;

	SparseMatrix<double> fx_Ek1(fx_order, x_order);
	vector<Tf> tripletList;
	tripletList.reserve(0.1 * fx_order * x_order);

	for(int n_i = 0; n_i < nodes.size(); n_i++)
	{
		for(int ti = 0; ti < 3 ;ti++)
		{
			int this_row = kin_begin + 3 * n_i + ti;
			int this_col = n_i * x_rt + 9 + ti;
			tripletList.push_back(Tf(this_row, this_col, m / std::sqrt(2) / timestep));

		}
	}
	fx_Ek1.setFromTriplets(tripletList.begin(), tripletList.end());

	return fx_Ek1;
}

Eigen::VectorXd AnimateTargetFunction::calcfx_Ek2(std::shared_ptr<Param> param)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;

	VectorXd fx_Ek2 = VectorXd::Zero(fx_order);
	int index = kin_begin;
	for(int i = 0; i < nodes.size(); i++)
	{
		Vector3d velocity = nodes[i]->getVelocity();
		fx_Ek2(index + 3 * i + 0) = -m / std::sqrt(2) * velocity(0);
		fx_Ek2(index + 3 * i + 1) = -m / std::sqrt(2) * velocity(1);
		fx_Ek2(index + 3 * i + 2) = -m / std::sqrt(2) * velocity(2);
	}
	return fx_Ek2;
}

Eigen::VectorXd AnimateTargetFunction::calcJfTfx_Gv(std::shared_ptr<Param> param)
{
	VectorXd JfTfx_G = VectorXd::Zero(x_order);
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;

	for(int n_i = 0; n_i < nodes.size(); n_i++)
	{
		JfTfx_G[x_rt * n_i + 9 + 1] = 0.5 * m * m * gravity;
	}
	return JfTfx_G;
}

void AnimateTargetFunction::calcJfKin(std::shared_ptr<Param> param, std::vector<Tf> &tripletList)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;
	int row, col;
	for(int n_i = 0; n_i < nodes.size(); n_i++)
	{
		for(int ti = 0; ti < 3; ti++)
		{
			row = kin_begin + 3 * n_i + ti;
			col = n_i * 12 + 9 + ti;
			tripletList.push_back(Tf(row, col, m / std::sqrt(2) / timestep));
		}
	}
}

void AnimateTargetFunction::calcfxKin(std::shared_ptr<Param> param, Eigen::VectorXd &fx)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;
	int index = kin_begin;
	Vector3d kinTerm;

	for(int n_i = 0; n_i < nodes.size(); n_i++, index += 3)
	{
		kinTerm = m * nodes[n_i]->getVelocity() / std::sqrt(2);
		fx[index + 0] = kinTerm(0);
		fx[index + 1] = kinTerm(1);
		fx[index + 2] = kinTerm(2);
	}
}

void AnimateTargetFunction::calcJfGv(std::shared_ptr<Param> param, std::vector<Tf> &tripletList)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;
	int row, col;
	for(int n_i = 0; n_i < nodes.size(); n_i++)
	{
		row = gv_begin + n_i + 1; // for y only
		col = n_i * x_rt + 9 + 1;
		tripletList.push_back(Tf(row, col, m * std::sqrt(gravity) * 0.5 / std::sqrt(default_height + nodes[n_i]->getTranslation()[1])));
	}
}

void AnimateTargetFunction::calcfxGv(std::shared_ptr<Param> param, Eigen::VectorXd &fx)
{
	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);
	std::vector<Node *> nodes = xparam->nodes;

	for(int n_i = 0; n_i < nodes.size(); n_i++)
	{
		fx[gv_begin + n_i * 3 + 1] = m * std::sqrt(gravity) * std::sqrt(default_height + nodes[n_i]->getTranslation()[1]);
	}
}

void AnimateTargetFunction::setOrder(std::shared_ptr<Param> param)
{
	DeformTargetFunction::setOrder(param);

	shared_ptr<DeformParam> xparam = static_pointer_cast<DeformParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	// Kinetic term
	kin_begin = fx_order;
	fx_order += 3 * nodes.size();
	// Gravity term
	gv_begin = fx_order;
	fx_order += 3 * nodes.size();
}
