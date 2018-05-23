#include "deformGraph.h"
#include "distCompare.h"
#include <limits>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

DeformGraph::DeformGraph()
{

}

DeformGraph::DeformGraph(std::string &name,
						 vector<GraphVertex *> &vertices,
						 vector<Node *> &nodes):
						modelName(name),
						vertices(vertices),
						nodes(nodes)
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
		cout << "position: ";
		print(v->getPosition());
		cout << "normal: ";
		print(v->getNormal());
		cout << endl << "expected min: " << min << " max: " << max << endl;
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
			cout << curpos[0] << "," << curpos[1] << "," << curpos[2] <<  " dist:" <<  (curpos - pos).norm() << endl;
			heap.pop();
		}

		// dist_max = dist of k+1 nearest nodes
		double dist_max = (heap.top()->getPosition() - pos).norm();
		double norm_sum = 0.0f; // sum used to normalizing
		double weight;

		// Equation 4
		for(auto d:dists)
		{
			weight = (1.0f - d / dist_max) * (1.0f - d / dist_max);
			weights.push_back(weight);
			norm_sum += weight;
		}
		std::cout << "norm sum:" << norm_sum << std::endl;
		// Normalize
		for(int i = 0; i < weights.size(); i++)
		{
			weights[i] /= norm_sum;
			std::cout << "weight" << i <<":" << weights[i] << std::endl;
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

void DeformGraph::outputToFile()
{
	ofstream fout("models/deform/" + modelName + ".obj");

	if(!fout.is_open())
	{
		cout << "ERROR: Cannot output deform model" << endl;
		return;
	}
	fout << "# " << endl << "# " << endl << "mtllib " + modelName + ".mtl" << endl;
	Vector3d position, normal;
	for(auto v:vertices)
	{
		position = v->getPosition();
		normal = v->getNormal();
		fout << "vn " << setiosflags(ios::fixed) << setprecision(6) << normal[0] << " " << normal[1] << " " << normal[2] << endl;
		fout << "v " << setiosflags(ios::fixed) << setprecision(6) << position[0] << " " << position[1] << " " << position[2] << endl;
	}
	fout.close();
}

vector<Vertex> DeformGraph::returnVertices()
{
	vector<Vertex> ret;
	for(auto v:vertices)
	{
		glm::vec3 position = glm::vec3(3.5f * v->position[0], 3.5f * v->position[1], 3.5f * v->position[2]);
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




void DeformGraph::draw()
{
	// glGenVertexArrays(1, &VAO);
 //    glGenBuffers(1, &this->VBO);
 //    glGenBuffers(1, &this->EBO);

 //    glBindVertexArray(this->VAO);
 //    // Load data into vertex buffers
 //    glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
 //    // A great thing about structs is that their memory layout is sequential for all its items.
 //    // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a Vector3d/2 array which
 //    // again translates to 3/2 floats which translates to a byte array.
 //    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &this->vertices[0], GL_STATIC_DRAW);

 //    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->EBO);
 //    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(GLuint), &this->indices[0], GL_STATIC_DRAW);

 //    // Set the vertex attribute pointers
 //    // Vertex Positions
 //    glEnableVertexAttribArray(0);
 //    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) 0);
 //    // Vertex Normals
 //    glEnableVertexAttribArray(1);
 //    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) offsetof(Vertex, Normal));
 //    // Vertex Texture Coords
 //    glEnableVertexAttribArray(2);
 //    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) offsetof(Vertex, TexCoords));

 //    glBindVertexArray(0);


	// glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	// glClear(GL_COLOR_BUFFER_BIT);
 	// glColor3f(0.0f, 0.4f, 0.2f);
	// for(auto v:vertices)
	// {
	// 	vector<Vector3d> vnodes;
	// 	for(auto n:v->getNodes())
	// 		vnodes.push_back(n->getPosition());

	// 	// create an edge between each nodes that influences the same vertex
	// 	for(int i = 0; i < vnodes.size(); i++)
	// 	{
	// 		for(int j = i + 1; j < vnodes.size(); j++)
	// 		{
	// 			glBegin(GL_LINES);
 //    			glVertex3f(vnodes[i][0], vnodes[i][1], vnodes[i][2]);
 //    			glVertex3f(vnodes[j][0], vnodes[j][1], vnodes[j][2]);
	// 			glEnd();
	// 			break;
	// 			cout << "~" << endl;
	// 		}
	// 		break;
	// 	}
	// }
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
			std::cout << "applyTransform: " << std::endl;
			std::cout << "rot: " << std::endl;
			std::cout << rotation << std::endl;
			std::cout << "t: " << std::endl;
			std::cout << translation << std::endl;
		}
	}

	// Update the positions of all vertices within the aabb
	Vector3d position, normal;
	for(auto v:vertices)
	{
		if(!aabb.isInside(v->getPosition()))
			continue;

		position = normal = Vector3d(0.0f, 0.0f, 0.0f);
		for(int i = 0; i < v->nodes.size(); i++)
		{
			position += v->weights[i] * v->nodes[i]->transformPosition(v->position_init);
			normal += v->weights[i] * v->nodes[i]->transformNormal(v->normal_init);
			// std::cout << "weight " << i << ":" << v->weights[i] << std::endl;
			// std::cout << "accumulated position:";
			// print(position);
			// std::cout << "accumulated normal:";
			// print(normal);
		}
		v->userSetPosition(position);
		// v->setPositionAndNormal(position, normal);
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

	rotation << cos90, 0.0f, sin90,
                0.0f,  1.0f, 0.0f,
                -sin90,0.0f, cos90;
	for(auto v:vertices)
	{
		position = normal = Vector3d(0.0f, 0.0f, 0.0f);
		for(int i = 0; i < v->nodes.size(); i++)
		{
			position += v->weights[i] * v->nodes[i]->transformPosition(v->position_init);
			normal += v->weights[i] * v->nodes[i]->transformNormal(v->normal_init);
			std::cout << i << "-th weight " << i << ":" << v->weights[i] << std::endl;
			std::cout << "accumulated position:";
			print(position);
			std::cout << "accumulated normal:";
			print(normal);
			Vector3d expected_pos = rotation * v->position_init;
			Vector3d expected_normal = rotation.inverse() * v->normal_init;
			std::cout << "expected position: " << std::endl;
			std::cout << expected_pos << std::endl;
			std::cout << "expected normal: " << std::endl;
			std::cout << expected_normal << std::endl;

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
	updateOrder();
	double Fx, Fx_old = 0.0f;
  	SimplicialLDLT<SparseMd> chol;

	VectorXd delta(x_order);
	for(int i = 0; i < max_iter; i++)
	{
		debug("1");
		SparseMd Jf = this->getJf();
		// std::cout << i << "-th Jf: " << std::endl;
		// for(int j = 0; j < Jf.rows(); j++)
		// {
		// 	for(int k = 0; k <Jf.cols(); k++)
		// 	{
		// 		if(std::fabs(Jf.coeffRef(j, k)) != 0.0f)
		// 			std::cout << "catch Jf(" << j << ", " << k << ") " << Jf.coeffRef(j, k) << std::endl;
		// 	}
		// }

		debug("1.1");
		VectorXd fx = this->getfx();
		std::cout << i << "-th fx: " << std::endl;

		VectorXd E_rot = fx.segment(0, rot_end);
		VectorXd E_reg = fx.segment(rot_end, reg_end - rot_end);
		VectorXd E_con = fx.segment(reg_end, fx.size() - reg_end);

		std::cout << "E_rot: " << E_rot.colwise().norm() << std::endl;
		std::cout << "E_reg: " << E_reg.colwise().norm() << std::endl;
		std::cout << "E_con: " << E_con.colwise().norm() << std::endl;

		// std::cout << fx << std::endl;
		// for(int j = 0; j < fx.rows(); j++)
		// {
		// 	if(std::fabs(fx(j)) > 1e-3)
		// 		std::cout << j << "-th huge: " << fx(j) << std::endl;
		// 	else if(std::fabs(fx(j)) < 1e-4 && std::fabs(fx(j)) > 0)
		// 		std::cout << j << "-th tiny: " << fx(j) << std::endl;
		// }
		debug("1.2");
		if(i == 0)
			delta = descentDirection(Jf, fx, chol, true);
		else
			delta = descentDirection(Jf, fx, chol, false);

		// std::cout << "checkpoint: " << rot_end << " " << reg_end << " "  << fx_order << std::endl;
		// std::cout << i << "-th delta: " << std::endl;
		// for(int j = 0; j < delta.rows(); j++)
		// {
		// 	if(delta(j) > 0.001f)
		// 		std::cout << j << "-th huge: " << delta(j) << std::endl;
		// }

		debug("2");

		Fx = fx.transpose() * fx;

		debug("2.5");

		MatrixXd deltaFx = 2.0f * fx.transpose() * Jf;

		debug("3");

		if(std::fabs(Fx - Fx_old) < epsilon * (1.0f + Fx)
		&& deltaFx.maxCoeff() < 1e-2 * (1.0f + Fx)
		&& delta.maxCoeff() < 1e-3 * (1.0f + delta.maxCoeff()))
		{
			return;
		}

		VectorXd fx_Jx0 = fx + Jf * delta;

		debug("4");
		// Update R and t of all nodes
		updateNodesRt(delta);

		VectorXd fx_x0 = this->getfx();

		std::cout << "fx_Jx0: " << fx_Jx0.colwise().norm() << " fx_x0: " << fx_x0.colwise().norm() << std::endl;

		for(int k = 0; k < fx_x0.size(); k++)
		{
			if(std::fabs(fx_Jx0(k) - fx_x0(k)) > 0.02f)
				std::cout << "catch huge difference " << k << ": " << fx_Jx0(k) << ", " << fx_x0(k) << std::endl;
		}

		debug("5");
		// Update the positions of all vertices using current transformation
		update();

		debug("6");
	}
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
		std::cout << n_i << "-th delta rot: " << std::endl;
		std::cout << rot << std::endl;
		std::cout << n_i << "-th delta t: " << std::endl;
		std::cout << t << std::endl;

		// std::cout << n_i << std::endl;
	 	delta_rotation << rot(0), rot(1), rot(2),
	 					  rot(3), rot(4), rot(5),
	 					  rot(6), rot(7), rot(8);
		delta_translation = t;
		// std::cout << n_i << std::endl;
		n->addDeltaRotation(delta_rotation);
		n->addDeltaTranslation(delta_translation);
		n_i++;
	}
}


void DeformGraph::updateOrder()
{
	x_order = 12 * nodes.size();
	fx_order = 6 * nodes.size();
	std::cout << "Erot end pos: " << fx_order << std::endl;
	rot_end = fx_order;
	for(auto n:nodes)
		fx_order += 3 * n->getNeighbors().size();
	std::cout << "Ereg end pos: " << fx_order << std::endl;
	reg_end = fx_order;
	fx_order += 3 * vertices.size();
	std::cout << "x_order:" << x_order << " fx_order:" << fx_order << std::endl;
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

		debug("1.0.1");

// Erot -- the first 6 rows
//=============================================================================
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
					// Jf.insert(row + 0, col) = rotation(roti, 1 - rotj);

				debug("1.0.1.2");

				// c1 * c3
				if(rotj == 0 || rotj == 2)
					tripletList.push_back(Tf(row + 1, col, std::sqrt(w_rot) * rotation(roti, 2 - rotj)));
					// Jf.insert(row + 1, col) = rotation(roti, 2 - rotj);

				// c2 * c3
				if(rotj == 1 || rotj == 2)
					tripletList.push_back(Tf(row + 2, col, std::sqrt(w_rot) * rotation(roti, 3 - rotj)));
					// Jf.insert(row + 2, col) = rotation(roti, 3 - rotj);

				// c1 * c1 -1
				if(rotj == 0)
					tripletList.push_back(Tf(row + 3, col, std::sqrt(w_rot) * 2.0f * rotation(roti, 0)));
					// Jf.insert(row + 3, col) = 2.0f * rotation(roti, 0);

				// c2 * c2 -1
				if(rotj == 1)
					tripletList.push_back(Tf(row + 4, col, std::sqrt(w_rot) * 2.0f * rotation(roti, 1)));
					// Jf.insert(row + 4, col) = 2.0f * rotation(roti, 1);

				// c3 * c3 -1
				if(rotj == 2)
					tripletList.push_back(Tf(row + 5, col, std::sqrt(w_rot) * 2.0f * rotation(roti, 2)));
					// Jf.insert(row + 5, col) = 2.0f * rotation(roti, 2);
			}
		}
	}

	debug("1.0.2");

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
					// Jf.insert(this_row, this_col) = (neighbor->getPosition() - n_row->getPosition())[rot_j];
				}
			}

			// iterate through ti_1 to ti_3
			for(int ti = 0; ti < 3; ti++)
			{
				// not consider t_k here
				this_row = row + 3 * neighbor_count + ti;
				this_col = ci * x_rt + 9 + ti;
				tripletList.push_back(Tf(this_row, this_col, std::sqrt(w_reg) * 1.0f));
				// Jf.insert(this_row, this_col) = 1.0f;
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
						tripletList.push_back(Tf(this_row, this_col, std::sqrt(w_reg) * (-1.0f)));
						// Jf.insert(this_row, this_col) = -1.0f;
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

	debug("1.0.3");
	// row is already set to nodes.size() * (6 + 3 * N(nodes).size())
	for(auto v:vertices)
	{
		if(!v->isFixed && !v->isHandled)
		{
			// If the vertex is not constrainted
			row += 3;
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
							// std::cout << row << " " << col << std::endl;
							tripletList.push_back(Tf(this_row,
													 this_col,
													 std::sqrt(w_con) * v->weights[vn_count] * (v->getPosition() - vn->getPosition())[rot_j]));
							// Jf.insert(this_row, this_col) = v->weights[vn_count]
							// 			 * (v->getPosition() - vn->getPosition())[rot_j];
						}
					}

					// derive t
					for(int ti = 0; ti < 3; ti++)
					{
						this_row = row + ti;
						this_col = n_col_count * x_rt + 9 + ti;
						// std::cout << row << " " << col << std::endl;
						tripletList.push_back(Tf(this_row, 
												 this_col, 
												 std::sqrt(w_con) * v->weights[vn_count]));
						// Jf.insert(this_row, this_col) = v->weights[vn_count];
					}
				}
				n_col_count++;
			}
			vn_count++;
		}
		row += 3;
	}
	debug("1.0.4");

	Jf.setFromTriplets(tripletList.begin(), tripletList.end());

	// assert(Jf.coeffRef(1, 0) == Jf.coeffRef(3, 1) && Jf.coeffRef(6, 2) == Jf.coeffRef(3, 1));

	debug("1.0.5");

	return Jf;
}

// // Deprecated
// double DeformGraph::dRegTerm(Vector3d &regTerm, int i)
// {
// 	assert(i >= 0 && i < 3);
// 	if(regTerm.norm() == 0.0f)
// 		return 0.0f;
// 	return sqrt10 * regTerm(i) / std::sqrt(regTerm(0) * regTerm(0) + regTerm(1) * regTerm(1) + regTerm(2) * regTerm(2));
// }

// // Deprecated
// double DeformGraph::dConTerm(Vector3d &conTerm, int i)
// {
// 	assert(i >= 0 && i < 3);
// 	if(conTerm.norm() == 0.0f)
// 		return 0.0f;
// 	return 10 * conTerm(i) / std::sqrt(conTerm(0) * conTerm(0) + conTerm(1) * conTerm(1) + conTerm(2) * conTerm(2));
// }

VectorXd DeformGraph::getfx()
{
	VectorXd fx(fx_order);
	int index = 0;

	debug("1.1.1");

	// Erot
	VectorXd rotTerm(6);
	for(auto n:nodes)
	{
		rotTerm = n->getRotTerm();
		std::cout << "get Rot term: " << std::endl;
		std::cout << rotTerm << std::endl;
		fx[index + 0] = std::sqrt(w_rot) * rotTerm(0);
		fx[index + 1] = std::sqrt(w_rot) * rotTerm(1);
		fx[index + 2] = std::sqrt(w_rot) * rotTerm(2);
		fx[index + 3] = std::sqrt(w_rot) * rotTerm(3);
		fx[index + 4] = std::sqrt(w_rot) * rotTerm(4);
		fx[index + 5] = std::sqrt(w_rot) * rotTerm(5);
		index += 6;
	}

	debug("1.1.2");

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
	debug("1.1.3");

	// Econ
	for(auto v:vertices)
	{
		if(!v->isFixed && !v->isHandled)
		{
			index += 3;
			continue;
		}
		VectorXd conTerm = v->getConTerm();
		for(int j = 0; j < 3; j++)
			fx[index++] = std::sqrt(w_con) * conTerm(j);
	}

	debug("1.1.4");

	return fx;
}

// Use cholesky decomposition to calculate the descent direction
VectorXd DeformGraph::descentDirection(const SparseMd &Jf, const VectorXd &fx, SimplicialLDLT<SparseMd> &chol, bool symbolic)
{
	// Solving:
  	SparseMd JfTJf = Jf.transpose() * Jf;
  	SparseMd I(JfTJf.rows(), JfTJf.cols());
  	I.setIdentity();
  	JfTJf += 1e-10 * I;
  	if(symbolic)
  		chol.analyzePattern(JfTJf);
	// Compute the sparse Cholesky Decomposition of Jf^T * Jf
  	chol.compute(JfTJf);

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

	// get the solution to the given right hand side
  	VectorXd x = chol.solve(-1.0f * Jf.transpose() * fx);
  	return x;
}

double DeformGraph::exactLineSearch()
{

}



