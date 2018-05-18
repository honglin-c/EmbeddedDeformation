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

DeformGraph::DeformGraph(vector<GraphVertex *> &vertices,
						 vector<Node *> &nodes):
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
		glm::vec3 pos = v->getPosition();
		std::priority_queue<Node *, std::vector<Node *>, distCompare> heap(pos);
		glm::vec3 curpos;
		float min = 1e10, max = 0.0;

		// Iterate through all nodes to find the nearest k nodes
		for(auto t:nodes)
		{
			heap.push(t);
			if (glm::length(t->getPosition() - pos) < min)
				min = glm::length(t->getPosition() - pos);
			if (glm::length(t->getPosition() - pos) > max)
				max = glm::length(t->getPosition() - pos);
		}

		assert(heap.size() >= this->k);
		// cout << "position: ";
		// print(v->getPosition());
		// cout << "normal: ";
		// print(v->getNormal());
		// cout << endl << "expected min: " << min << " max: " << max << endl;
		vector<Node *> vnodes;
		vector<float> dists, weights;
		Node * pnode = nullptr;

		// Get the top k nodes from heap
		for(int i = 0; i < this->k; i++)
		{
			pnode = heap.top();
			vnodes.push_back(pnode);
			dists.push_back(glm::length(pnode->getPosition() - pos));
			curpos = pnode->getPosition();
			// cout << curpos[0] << "," << curpos[1] << "," << curpos[2] <<  " dist:" <<  glm::length(curpos - pos) << endl;
			heap.pop();
		}

		// dist_max = dist of k+1 nearest nodes
		float dist_max = glm::length(heap.top()->getPosition() - pos);
		float norm_sum = 0.0f; // sum used to normalizing
		float weight;

		// Equation 4
		for(auto d:dists)
		{
			weight = (1.0f - d / dist_max) * (1.0f - d / dist_max);
			weights.push_back(weight);
			norm_sum += weight;
		}
		// std::cout << "norm sum:" << norm_sum << std::endl;
		// Normalize
		for(int i = 0; i < weights.size(); i++)
		{
			weights[i] /= norm_sum;
			// std::cout << "weight" << i <<":" << weights[i] << std::endl;
			assert(weights[i] == weights[i]); // avoid nan
		}

		v->setNodes(vnodes, weights);

		// Update the neighbor of each nodes that belongs to the vertex
		v->updateNeighbor();
	}
}

void DeformGraph::print() const
{
	vector<float> weights;
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
	ofstream fout("models/deform/cat.obj");

	if(!fout.is_open())
	{
		cout << "ERROR: Cannot output deform model" << endl;
		return;
	}
	fout << "# " << endl << "# " << endl << "mtllib cat.mtl" << endl;
	glm::vec3 position, normal;
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
		Vertex rv = {3.5f * v->position, v->normal};
		ret.push_back(rv);
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
 //    // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
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
	// 	vector<glm::vec3> vnodes;
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

void DeformGraph::print(const glm::vec3 &v)
{
	cout << v[0] << "," << v[1] << "," << v[2] << endl;
}


void DeformGraph::applyTransformation(glm::mat3 &rotation, glm::vec3 &translation, AABB &aabb)
{
	for(auto n:nodes)
	{
		if(aabb.isInside(n->getPosition()))
		{
			n->setTransformation(rotation, translation);
		}
	}

	// Update the positions of all vertices within the aabb
	glm::vec3 position, normal;
	for(auto v:vertices)
	{
		if(!aabb.isInside(v->getPosition()))
			continue;

		position = normal = glm::vec3(0.0f, 0.0f, 0.0f);
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
		v->setPositionAndNormal(position, normal);
	}
}


void DeformGraph::update()
{
	// transform vertex position and normal using the current transformation
	glm::vec3 position, normal;
	for(auto v:vertices)
	{
		position = normal = glm::vec3(0.0f, 0.0f, 0.0f);
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
	float Fx, Fx_old = 0.0f;
  	SimplicialCholesky<SparseMf> chol;

	VectorXf delta(x_order);
	for(int i = 0; i < max_iter; i++)
	{
		debug("1");
		SparseMf Jf = this->getJf();
		debug("1.1");
		VectorXf fx = this->getfx();
		std::cout << fx << std::endl;
		debug("1.2");
		if(i == 0)
			delta = descentDirection(Jf, fx, chol, true);
		else
			delta = descentDirection(Jf, fx, chol, false);
		delta.normalize();

		debug("2");

		Fx = fx.transpose() * fx;

		debug("2.5");

		MatrixXf deltaFx = 2.0f * fx.transpose() * Jf;

		debug("3");

		if(std::fabs(Fx - Fx_old) < epsilon * (1.0f + Fx)
		&& deltaFx.maxCoeff() < 1e-2 * (1.0f + Fx)
		&& delta.maxCoeff() < 1e-3 * (1.0f + delta.maxCoeff()))
		{
			return;
		}

		debug("4");
		// Update R and t of all nodes
		updateNodesRt(delta);

		debug("5");
		// Update the positions of all vertices using current transformation
		update();

		debug("6");
	}
}

void DeformGraph::updateNodesRt(VectorXf delta)
{
	int n_i = 0;
	VectorXf rot;
	VectorXf t;
	glm::mat3 delta_rotation;
	glm::vec3 delta_translation;
	for(auto n:nodes)
	{
		rot = delta.segment(n_i * x_rt, 9);
		t = delta.segment(n_i * x_rt + 9, 3);
		std::cout << n_i << std::endl;
		delta_rotation = glm::mat3(rot[0], rot[1], rot[2],
								   rot[3], rot[4], rot[5],
								   rot[6], rot[7], rot[8]);
		delta_translation = glm::vec3(t[0], t[1], t[2]);
		std::cout << n_i << std::endl;
		n->addDeltaRotation(delta_rotation);
		n->addDeltaTranslation(delta_translation);
		n_i++;
	}
}


void DeformGraph::updateOrder()
{
	x_order = 12 * nodes.size();
	fx_order = 6 * nodes.size();
	for(auto n:nodes)
		fx_order += n->getNeighbors().size();
	fx_order += vertices.size();
}


// Compute Jf - double check here
SparseMf DeformGraph::getJf()
{
	// Create a sparse matrix whose entry is 0 unless it is set in the below
	SparseMf Jf(fx_order, x_order);
	int row, col;

	// iterate through nodes by columns: 12 columns at a time
	for(int ci = 0; ci < nodes.size(); ci++)
	{
		auto n = nodes[ci];

		debug("1.0.1");

// Erot -- the first 6 rows
//=============================================================================
		Matrix3f rotation = n->matRotation();
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
					Jf.insert(row + 0, col) = rotation(roti, 1 - rotj);

				debug("1.0.1.2");

				// c1 * c3
				if(rotj == 0 || rotj == 2)
					Jf.insert(row + 1, col) = rotation(roti, 2 - rotj);

				// c2 * c3
				if(rotj == 1 || rotj == 2)
					Jf.insert(row + 2, col) = rotation(roti, 3 - rotj);

				// c1 * c1 -1
				if(rotj == 0)
					Jf.insert(row + 3, col) = 2.0f * rotation(roti, 0);

				// c2 * c2 -1
				if(rotj == 1)
					Jf.insert(row + 4, col) = 2.0f * rotation(roti, 1);

				// c3 * c3 -1
				if(rotj == 2)
					Jf.insert(row + 5, col) = 2.0f * rotation(roti, 2);
			}
		}
	}

	debug("1.0.2");

//===================================================================================================
// Ereg: row - from nodes.size() * 6 to nodes.size() * 6 + nodes.size() * neighbor(n).size()
//===================================================================================================
	int ni = 0, ci = 0;
	row = nodes.size() * 6;
	// iterate through nodes * N(nodes) rows
	for(auto n_row:nodes) // derive n_row node's R -- nonzero when n_row == nodes[ci]
	{
		int neighbor_count = 0;
		// iterate through all neighbors
		for(auto neighbor:n_row->getNeighbors())
		{
			Vector3f reg_k = n_row->getRegTerm(neighbor);
			// iterate from Rni_11 to Rni_33

			for(int rot_i = 0; rot_i < 3; rot_i++)
			{
				for(int rot_j = 0; rot_j < 3; rot_j++)
				{
					col = ci * x_rt + 3 * rot_i + rot_j;
					Jf.insert(row + neighbor_count, col) = dRegTerm(reg_k, rot_i)
														* (neighbor->getPosition()[rot_j] - n_row->getPosition()[rot_j]);
				}
			}

			// iterate through ti_1 to ti_3
			for(int ti = 0; ti < 3; ti++)
			{
				col = ci * x_rt + 9 + ti;
				Jf.insert(row + neighbor_count, col) = dRegTerm(reg_k, ti); // not consider t_k here
			}

			neighbor_count++;
		}

		row += n_row->getNeighbors().size();
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
			Vector3f reg_k = n_row->getRegTerm(neighbor);
			ci = 0;
			for(auto n_col:nodes)
			{
				if(n_col == neighbor)
				{
					col = ci * x_rt + 9;
					for(int ti = 0; ti < 3; ti++)
					{
						Jf.insert(row + neighbor_count, col + ti) = -1.0f * dRegTerm(reg_k, ti);
					}
				}
				ci++; // increase column index(ci-th node's R)
			}
			neighbor_count++;
		}
		row += n_row->getNeighbors().size();
	}
//===================================================================================================
// Econ: row - from nodes.size() * (6 + N(nodes).size())
//             to nodes.size() * (6 + N(nodes).size()) + vertices.size()
//===================================================================================================

	debug("1.0.3");
	// row is already set to nodes.size() * (6 + N(nodes).size())
	for(auto v:vertices)
	{
		Vector3f conTerm = v->getConTerm();
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
							col = n_col_count * x_rt + 3 * rot_i + rot_j;
							std::cout << row << " " << col << std::endl;
							Jf.insert(row, col) = dConTerm(conTerm, rot_i) * v->weights[vn_count]
										 * (v->getPosition()[rot_j] - vn->getPosition()[rot_j]);
						}
					}

					// derive t
					for(int ti = 0; ti < 3; ti++)
					{
						col = n_col_count * x_rt + 9 + ti;
						std::cout << row << " " << col << std::endl;
						Jf.insert(row, col) = dConTerm(conTerm, ti) * v->weights[vn_count];
					}
				}
				n_col_count++;
			}
			vn_count++;
		}
		row++;
	}
	debug("1.0.4");

	return Jf;
}

float DeformGraph::dRegTerm(Vector3f &regTerm, int i)
{
	assert(i >= 0 && i < 3);
	if(regTerm.norm() == 0.0f)
		return 0.0f;
	return sqrt10 * regTerm(i) / std::sqrt(regTerm(0) * regTerm(0) + regTerm(1) * regTerm(1) + regTerm(2) * regTerm(2));
}

float DeformGraph::dConTerm(Vector3f &conTerm, int i)
{
	assert(i >= 0 && i < 3);
	if(conTerm.norm() == 0.0f)
		return 0.0f;
	return 10 * conTerm(i) / std::sqrt(conTerm(0) * conTerm(0) + conTerm(1) * conTerm(1) + conTerm(2) * conTerm(2));
}

VectorXf DeformGraph::getfx()
{
	VectorXf fx(fx_order);
	int index = 0;

	debug("1.1.1");

	// Erot
	VectorXf rotTerm(6);
	for(auto n:nodes)
	{
		rotTerm =  n->getRotTerm();
		fx[index + 0] = rotTerm(0);
		fx[index + 1] = rotTerm(1);
		fx[index + 2] = rotTerm(2);
		fx[index + 3] = rotTerm(3);
		fx[index + 4] = rotTerm(4);
		fx[index + 5] = rotTerm(5);
		index += 6;
	}

	debug("1.1.2");

	// Ereg
	MatrixXf regTerm;
	for(auto n:nodes)
	{
		regTerm = n->getRegTerm();
		for(int i = 0; i < regTerm.rows(); i++)
		{
			fx[index++] = sqrt10 * regTerm.row(i).norm();
		}
	}
	debug("1.1.3");

	// Econ
	for(auto v:vertices)
	{
		fx[index++] = 10.0f * v->getConValue();
	}

	debug("1.1.4");

	return fx;
}

// Implement symbolic factorization and fill-reducing later - use Eigen first
VectorXf DeformGraph::descentDirection(const SparseMf &Jf, const VectorXf &fx, SimplicialCholesky<SparseMf> &chol, bool symbolic)
{
	// Solving:
  	SparseMf JfTJf = Jf.transpose() * Jf;
  	if(symbolic)
  		chol.analyzePattern(JfTJf);
  	chol.factorize(JfTJf);
	// performs a Cholesky factorizatison of A
  	VectorXf x = chol.solve(Jf.transpose() * fx);
	// use the factorization to solve for the given right hand side
  	return x;
}

float DeformGraph::exactLineSearch()
{

}



