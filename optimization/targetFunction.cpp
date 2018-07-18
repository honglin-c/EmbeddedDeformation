#include "targetFunction.h"
#include <iostream>

using namespace Eigen;
using namespace std;

TargetFunction::TargetFunction(shared_ptr<Param> param)
{
	setOrder(param);
}

TargetFunction::~TargetFunction()
{}

void TargetFunction::setOrder(shared_ptr<Param> param)
{
	shared_ptr<XParam> xparam = static_pointer_cast<XParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	x_order = 12 * nodes.size();
	fx_order = 6 * nodes.size();
	std::cout << "Erot end pos: " << fx_order << std::endl;

	reg_begin = fx_order;
	for(auto n:nodes)
		fx_order += 3 * n->getNeighbors().size();
	std::cout << "Ereg end pos: " << fx_order << std::endl;

	con_begin = fx_order;
	for(auto v:vertices)
	{
		if(v->isFixed || v->isHandled)
			fx_order += 3;
	}
	std::cout << "Econ end pos: " << fx_order << std::endl;

	std::cout << "x_order:" << x_order << " fx_order:" << fx_order << std::endl;
}

Eigen::SparseMatrix<double> TargetFunction::calcJf(shared_ptr<Param> param)
{
	SparseMatrix<double> Jf(fx_order, x_order);

	vector<Tf> tripletList;
	tripletList.reserve(0.1 * fx_order * x_order);

	// E_rot
	// row range: from 0 to nodes.size() * 6 - 1
	calcJfRot(param, tripletList);
	debug("1.0.2");

	// E_reg
	// row range: from nodes.size() * 6 to nodes.size() * 6 + nodes.size() * neighbor(n).size()
	calcJfReg(param, tripletList);
	debug("1.0.3");

	// E_con
	// row range: from nodes.size() * (6 + N(nodes).size())
	//            to nodes.size() * (6 + N(nodes).size()) + vertices.size()
	calcJfCon(param, tripletList);
	debug("1.0.4");

	Jf.setFromTriplets(tripletList.begin(), tripletList.end());

	debug("1.0.5");

	return Jf;
}

void TargetFunction::calcJfRot(shared_ptr<Param> param, vector<Tf> &tripletList)
{
	shared_ptr<XParam> xparam =

static_pointer_cast<XParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	int row = 0, col;

	// iterate through nodes by columns: 12 columns at a time
	for(int ci = 0; ci < nodes.size(); ci++)
	{
		auto n = nodes[ci];

		debug("1.0.1");

		// Erot -- the first 6 rows
		Matrix3d rotation = n->matRotation();
		row = 6 * ci; //Erot begins at row-th row

		debug("1.0.1.1");

		// Ri_11 to Ri_33: iterate the first 9 columns of a node
		for(int roti = 0; roti < 3; roti++)
		{
			for(int rotj = 0; rotj < 3; rotj++)
			{
				col = ci * x_rt + 3 * roti + rotj;

				// c1 * c2
				if(rotj == 0 || rotj == 1)
					tripletList.push_back(Tf(row + 0, col, std::sqrt(w_rot) * rotation(roti, 1 - rotj)));

				debug("1.0.1.2");

				// c1 * c3
				if(rotj == 0 || rotj == 2)
					tripletList.push_back(Tf(row + 1, col, std::sqrt(w_rot) * rotation(roti, 2 - rotj)));

				// c2 * c3
				if(rotj == 1 || rotj == 2)
					tripletList.push_back(Tf(row + 2, col, std::sqrt(w_rot) * rotation(roti, 3 - rotj)));

				// c1 * c1 -1
				if(rotj == 0)
					tripletList.push_back(Tf(row + 3, col, std::sqrt(w_rot) * 2.0 * rotation(roti, 0)));

				// c2 * c2 -1
				if(rotj == 1)
					tripletList.push_back(Tf(row + 4, col, std::sqrt(w_rot) * 2.0 * rotation(roti, 1)));

				// c3 * c3 -1
				if(rotj == 2)
					tripletList.push_back(Tf(row + 5, col, std::sqrt(w_rot) * 2.0 * rotation(roti, 2)));
			}
		}
	}
}

void TargetFunction::calcJfReg(shared_ptr<Param> param, vector<Tf> &tripletList)
{
	shared_ptr<XParam> xparam =

static_pointer_cast<XParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	int ni = 0, ci = 0;
	int this_row, this_col;
	int row = reg_begin, col;
	// iterate through nodes * N(nodes) rows
	for(auto n_row:nodes) // derive n_row node's R -- nonzero when n_row == nodes[ci]
	{
		int neighbor_count = 0;
		// iterate through all neighbors
		for(auto neighbor:n_row->getNeighbors())
		{
			Vector3d reg_k = n_row->getRegTerm(neighbor);

			// iterate from Rni_11 to Rni_33
			for(int rot_i = 0; rot_i < 3; rot_i ++)
			{
				for(int rot_j = 0; rot_j < 3; rot_j++)
				{
					this_row = row + 3 * neighbor_count + rot_i;
					this_col = ci * x_rt + rot_i * 3 + rot_j;
					tripletList.push_back(Tf(this_row, this_col,
											 std::sqrt(w_reg) * (neighbor->getPosition() - n_row->getPosition())[rot_j]));
				}
			}

			// iterate through ti_1 to ti_3
			for(int ti = 0; ti < 3; ti++)
			{
				// not consider t_k here
				this_row = row + 3 * neighbor_count + ti;
				this_col = ci * x_rt + 9 + ti;
				tripletList.push_back(Tf(this_row, this_col, std::sqrt(w_reg) * 1.0));
			}

			neighbor_count++;
		}

		row += 3 * n_row->getNeighbors().size();
		ci++; // increase column index(ci-th node's R)
	}

	// only consider deriving t_k here
	row = nodes.size() * 6;
	for(auto n_row:nodes)
	{
		int neighbor_count = 0;
		// iterate through all neighbors
		for(auto neighbor:n_row->getNeighbors())
		{
			Vector3d reg_k = n_row->getRegTerm(neighbor);
			ci = 0;
			for(auto n_col:nodes)
			{
				if(n_col == neighbor)
				{
					col = ci * x_rt + 9;
					for(int ti = 0; ti < 3; ti++)
					{
						this_row = row + 3 * neighbor_count + ti;
						this_col = col + ti;
						tripletList.push_back(Tf(this_row, this_col, std::sqrt(w_reg) * (-1.0)));
					}
				}
				ci++; // increase column index(ci-th node's R)
			}
			neighbor_count++;
		}
		row += 3 * n_row->getNeighbors().size();
	}
}

void TargetFunction::calcJfCon(shared_ptr<Param> param, vector<Tf> &tripletList)
{
	shared_ptr<XParam> xparam =

static_pointer_cast<XParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;
	std::vector<Node *> nodes = xparam->nodes;

	int row = con_begin, col;
	int this_row, this_col;

	for(auto v:vertices)
	{
		if(!v->isFixed && !v->isHandled)
		{
			// If the vertex is not constrainted
			continue;
		}

		Vector3d conTerm = v->getConTerm();
		int vn_count = 0;

		for(auto vn:v->nodes)
		{
			int n_col_count = 0;
			for(auto n_col:nodes)
			{
				// if n_col is one of the v's neighbors
				if(vn == n_col)
				{
					// derive R
					for(int rot_i = 0; rot_i < 3; rot_i++)
					{
						for(int rot_j = 0; rot_j < 3; rot_j++)
						{
							this_row = row + rot_i;
							this_col = n_col_count * x_rt + 3 * rot_i + rot_j;
							tripletList.push_back(Tf(this_row,
													 this_col,
													 std::sqrt(w_con) * v->weights[vn_count] * (v->getPosition() - vn->getPosition())[rot_j]));
						}
					}

					// derive t
					for(int ti = 0; ti < 3; ti++)
					{
						this_row = row + ti;
						this_col = n_col_count * x_rt + 9 + ti;
						tripletList.push_back(Tf(this_row,
												 this_col,
												 std::sqrt(w_con) * v->weights[vn_count]));
					}
				}
				n_col_count++;
			}
			vn_count++;
		}
		row += 3;
	}
}

Eigen::VectorXd TargetFunction::calcfx(shared_ptr<Param> param)
{
	VectorXd fx(fx_order);
	int index = 0;

	debug("1.1.1");

	// Erot
	calcfxRot(param, fx);
	debug("1.1.2");

	// Ereg
	calcfxReg(param, fx);
	debug("1.1.3");

	// Econ
	calcfxCon(param, fx);
	debug("1.1.4");

	return fx;
}

void TargetFunction::calcfxRot(shared_ptr<Param> param, VectorXd &fx)
{
	shared_ptr<XParam> xparam =

static_pointer_cast<XParam, Param>(param);

	std::vector<Node *> nodes = xparam->nodes;

	int index = 0;

	VectorXd rotTerm(6);
	for(auto n:nodes)
	{
		rotTerm = n->getRotTerm();
		// std::cout << "get Rot term: " << std::endl;
		// std::cout << rotTerm << std::endl;
		fx[index + 0] = std::sqrt(w_rot) * rotTerm(0);
		fx[index + 1] = std::sqrt(w_rot) * rotTerm(1);
		fx[index + 2] = std::sqrt(w_rot) * rotTerm(2);
		fx[index + 3] = std::sqrt(w_rot) * rotTerm(3);
		fx[index + 4] = std::sqrt(w_rot) * rotTerm(4);
		fx[index + 5] = std::sqrt(w_rot) * rotTerm(5);
		index += 6;
	}
}

void TargetFunction::calcfxReg(shared_ptr<Param> param, VectorXd &fx)
{
	shared_ptr<XParam> xparam =

static_pointer_cast<XParam, Param>(param);

	std::vector<Node *> nodes = xparam->nodes;

	int index = reg_begin;

	MatrixXd regTerm;
	for(auto n:nodes)
	{
		regTerm = n->getRegTerm();
		for(int i = 0; i < regTerm.rows(); i++)
		{
			for(int j = 0; j < 3; j++)
			{
				fx[index++] = std::sqrt(w_reg) * regTerm(i, j);
			}
		}
	}
}

void TargetFunction::calcfxCon(shared_ptr<Param> param, VectorXd &fx)
{
	shared_ptr<XParam> xparam =

static_pointer_cast<XParam, Param>(param);

	std::vector<GraphVertex *> vertices = xparam->vertices;

	int index = con_begin;
	for(auto v:vertices)
	{
		if(!v->isFixed && !v->isHandled)
		{
			continue;
		}
		VectorXd conTerm = v->getConTerm();
		for(int j = 0; j < 3; j++)
			fx[index++] = std::sqrt(w_con) * conTerm(j);
	}
}

void TargetFunction::debug(const std::string s)
{
	std::cout << s << std::endl;
}