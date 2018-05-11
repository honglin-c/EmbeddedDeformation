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

void DeformGraph::update()
{

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
		cout << "position: ";
		print(v->getPosition());
		cout << "normal: ";
		print(v->getNormal());
		cout << endl << "expected min: " << min << " max: " << max << endl;
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
			cout << curpos[0] << "," << curpos[1] << "," << curpos[2] <<  " dist:" <<  glm::length(curpos - pos) << endl;
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

	// transform vertex position and normal
	glm::vec3 position, normal;
	for(auto v:vertices)
	{
		if(!aabb.isInside(v->getPosition()))
			continue;

		position = normal = glm::vec3(0.0f, 0.0f, 0.0f);
		for(int i = 0; i < v->nodes.size(); i++)
		{
			position += v->weights[i] * v->nodes[i]->transformPosition(v->position);
			normal += v->weights[i] * v->nodes[i]->transformNormal(v->normal);
			std::cout << "weight " << i << ":" << v->weights[i] << std::endl;
			std::cout << "accumulated position:";
			print(position);
			std::cout << "accumulated normal:";
			print(normal);
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


