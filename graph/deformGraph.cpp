#include "deformGraph.h"
#include "distCompare.h"
#include <limits>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>

using namespace std;

DeformGraph::DeformGraph()
{

}

DeformGraph::DeformGraph(std::string &name,
						 vector<GraphVertex *> &vertices,
						 vector<Node *> &nodes):
						modelName(name),
						vertices(vertices),
						nodes(nodes),
						constraint_count(0)
{
	findKNN();
}

DeformGraph::~DeformGraph()
{
	for(auto v:vertices)
		delete v;
	for(auto n:nodes)
		delete n;
}

void DeformGraph::findKNN()
{
	for(auto v:vertices)
	{
		Vector3d pos = v->getPosition();
		std::priority_queue<Node *, std::vector<Node *>, distCompare> heap(pos);
		Vector3d curpos;
		double min = 1e10, max = 0.0;

		// Iterate through all nodes to find the nearest k nodes
		for(auto t:nodes)
		{
			heap.push(t);
			if ((t->getPosition() - pos).norm() < min)
				min = (t->getPosition() - pos).norm();
			if ((t->getPosition() - pos).norm() > max)
				max = (t->getPosition() - pos).norm();
		}

		assert(heap.size() >= this->k);
		vector<Node *> vnodes;
		vector<double> dists, weights;
		Node * pnode = nullptr;

		// Get the top k nodes from heap
		for(int i = 0; i < this->k; i++)
		{
			pnode = heap.top();
			vnodes.push_back(pnode);
			dists.push_back((pnode->getPosition() - pos).norm());
			curpos = pnode->getPosition();
			heap.pop();
		}

		// dist_max = dist of k+1 nearest nodes
		double dist_max = (heap.top()->getPosition() - pos).norm();
		double norm_sum = 0.0; // sum used to normalizing
		double weight;

		// Equation 4
		for(auto d:dists)
		{
			weight = (1.0 - d / dist_max) * (1.0 - d / dist_max);
			weights.push_back(weight);
			norm_sum += weight;
		}
		// Normalize
		for(int i = 0; i < weights.size(); i++)
		{
			weights[i] /= norm_sum;
			assert(weights[i] == weights[i]); // avoid nan
		}
		v->setNodes(vnodes, weights);

		// Update the neighbor of each nodes that belongs to the vertex
		v->updateNeighbor();
	}
}

void DeformGraph::print() const
{
	vector<double> weights;
	vector<Node *> vnodes;
	for(auto v:vertices)
	{
		cout << "vertex: " << v->getPosition()[0] << " " << v->getPosition()[1] << " " << v->getPosition()[2] << endl;
		weights = v->getWeights();
		vnodes = v->getNodes();
		for(int i = 0; i < weights.size(); i++)
		{
			cout << "node: " << vnodes[i]->getPosition()[0] << " " << vnodes[i]->getPosition()[1] << " " << vnodes[i]->getPosition()[2] << "   ";
			cout << "weight: " << weights[i] << endl;
		}
		cout << endl;
	}
}

vector<Vertex> DeformGraph::returnVertices()
{
	vector<Vertex> ret;
	for(auto v:vertices)
	{
		glm::vec3 position = glm::vec3(v->position[0], v->position[1], v->position[2]);
		glm::vec3 normal = glm::vec3(v->normal[0], v->normal[1], v->normal[2]);
		Vertex rv = {position, normal};
		ret.push_back(rv);
	}
	return ret;
}

vector<glm::vec3> DeformGraph::returnNodes()
{
	vector<glm::vec3> ret;
	for(auto n:nodes)
	{
		Vector3d position = n->getPosition() + n->getTranslation();
		ret.push_back(glm::vec3(position[0], position[1], position[2]));
	}
	return ret;
}


void DeformGraph::print(const Vector3d &v)
{
	cout << v[0] << "," << v[1] << "," << v[2] << endl;
}


void DeformGraph::applyTransformation(Matrix3d &rotation, Vector3d &translation, AABB &aabb)
{
	for(auto n:nodes)
	{
		if(aabb.isInside(n->getPosition()))
		{
			n->setTransformation(rotation, translation);
		}
	}

	// Update the positions of all vertices within the aabb
	Vector3d position, normal;
	for(auto v:vertices)
	{
		if(!aabb.isInside(v->getPosition()))
			continue;

		position = normal = Vector3d(0.0, 0.0, 0.0);
		for(int i = 0; i < v->nodes.size(); i++)
		{
			position += v->weights[i] * v->nodes[i]->transformPosition(v->position_init);
			normal += v->weights[i] * v->nodes[i]->transformNormal(v->normal_init);
		}
		v->userSetPosition(position);
		constraint_count++;
	}

	this->update();
}


void DeformGraph::update()
{
	// transform vertex position and normal using the current transformation
	Vector3d position, normal;
	Matrix3d rotation;
    double sin45, cos45, sin90, cos90;
    sin90 = 1.0; cos90 = 0.0;
    sin45 = cos45 = std::sqrt(2.0) / 2.0;

	rotation << cos90, 0.0, sin90,
                0.0,  1.0, 0.0,
                -sin90,0.0, cos90;
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


void DeformGraph::addFixedConstraint(AABB &aabb)
{
	for(auto v:vertices)
	{
		if(aabb.isInside(v->getPosition()))
		{
			v->setFixed(true);
			constraint_count ++;
		}
	}
}

void DeformGraph::debug(std::string s)
{
	std::cout << s << std::endl;
}

// Gauss-Newton : for Cholesky Decompostion, implement fill-reducing and symbolic facterization later
void DeformGraph::optimize()
{
	ofstream fout("benchmark/" + modelName + "_measure_time.txt");
	clock_t begin, end;
	clock_t solve_time = 0, def_time = 0;
	updateOrder();
	double Fx, Fx_old = 0.0;
  	SimplicialLDLT<SparseMd> chol;

	VectorXd delta(x_order);
	int i;
	for(i = 0; i < max_iter; i++)
	{
		begin = clock();
		SparseMd Jf = this->getJf();

		VectorXd fx = this->getfx();

		if(i == 0)
			delta = descentDirection(Jf, fx, chol, true);
		else
			delta = descentDirection(Jf, fx, chol, false);

		Fx = fx.transpose() * fx;

		MatrixXd deltaFx = 2.0 * fx.transpose() * Jf;

		if(std::fabs(Fx - Fx_old) < epsilon * (1.0 + Fx)
		&& deltaFx.maxCoeff() < 1e-2 * (1.0 + Fx)
		&& delta.maxCoeff() < 1e-3 * (1.0 + delta.maxCoeff()))
		{
			return;
		}

		VectorXd fx_Jx0 = fx + Jf * delta;

		// Update R and t of all nodes
		updateNodesRt(delta);

		VectorXd fx_x0 = this->getfx();

		end = clock();
		solve_time += end - begin;
		// Update the positions of all vertices using current transformation
		begin = clock();
		update();
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
}

void DeformGraph::updateNodesRt(VectorXd delta)
{
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
}


void DeformGraph::updateOrder()
{
	x_order = 12 * nodes.size();
	fx_order = 6 * nodes.size();
	rot_end = fx_order;
	for(auto n:nodes)
		fx_order += 3 * n->getNeighbors().size();
	reg_end = fx_order;
	for(auto v:vertices)
	{
		if(v->isFixed || v->isHandled)
			fx_order += 3;
	}
}


// Compute Jf - double check here
SparseMd DeformGraph::getJf()
{
	vector<Tf> tripletList;
	tripletList.reserve(0.1 * fx_order * x_order);
	// Create a sparse matrix whose entry is 0 unless it is set in the below
	SparseMd Jf(fx_order, x_order);
	int row, col;

	// iterate through nodes by columns: 12 columns at a time
	for(int ci = 0; ci < nodes.size(); ci++)
	{
		auto n = nodes[ci];

// Erot -- the first 6 rows
//=============================================================================
		Matrix3d rotation = n->matRotation();
		row = 6 * ci; //Erot begins at row-th row

		// Ri_11 to Ri_33: iterate the first 9 columns of a node
		for(int roti = 0; roti < 3; roti++)
		{
			for(int rotj = 0; rotj < 3; rotj++)
			{
				col = ci * x_rt + 3 * roti + rotj;

				// c1 * c2
				if(rotj == 0 || rotj == 1)
					tripletList.push_back(Tf(row + 0, col, std::sqrt(w_rot) * rotation(roti, 1 - rotj)));

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

//===================================================================================================
// Ereg: row - from nodes.size() * 6 to nodes.size() * 6 + nodes.size() * neighbor(n).size()
//===================================================================================================
	int ni = 0, ci = 0;
	int this_row, this_col;
	row = nodes.size() * 6;
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
					tripletList.push_back(Tf(this_row, this_col, std::sqrt(w_reg) * (neighbor->getPosition() - n_row->getPosition())[rot_j]));
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
//===================================================================================================
// Econ: row - from nodes.size() * (6 + N(nodes).size())
//             to nodes.size() * (6 + N(nodes).size()) + vertices.size()
//===================================================================================================

	// row is already set to nodes.size() * (6 + 3 * N(nodes).size())
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

	Jf.setFromTriplets(tripletList.begin(), tripletList.end());


	return Jf;
}

VectorXd DeformGraph::getfx()
{
	VectorXd fx(fx_order);
	int index = 0;

	// Erot
	VectorXd rotTerm(6);
	for(auto n:nodes)
	{
		rotTerm = n->getRotTerm();
		fx[index + 0] = std::sqrt(w_rot) * rotTerm(0);
		fx[index + 1] = std::sqrt(w_rot) * rotTerm(1);
		fx[index + 2] = std::sqrt(w_rot) * rotTerm(2);
		fx[index + 3] = std::sqrt(w_rot) * rotTerm(3);
		fx[index + 4] = std::sqrt(w_rot) * rotTerm(4);
		fx[index + 5] = std::sqrt(w_rot) * rotTerm(5);
		index += 6;
	}

	// Ereg
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

	// Econ
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


	return fx;
}

// Use cholesky decomposition to calculate the descent direction
VectorXd DeformGraph::descentDirection(const SparseMd &Jf, const VectorXd &fx, SimplicialLDLT<SparseMd> &chol, bool symbolic)
{
	// Solving:
  	SparseMd JfTJf = Jf.transpose() * Jf;
  	if(symbolic)
  		chol.analyzePattern(JfTJf);
	// Compute the sparse Cholesky Decomposition of Jf^T * Jf
  	chol.compute(JfTJf);

  	if(chol.info() == Eigen::ComputationInfo::NumericalIssue)
  	{
  		std::cout << "ERROR: Cholesky Decompostion Fail! JfTJf is not positive definite" << std::endl;
  	}

	// get the solution to the given right hand side
  	VectorXd x = chol.solve(-1.0 * Jf.transpose() * fx);
  	return x;
}


