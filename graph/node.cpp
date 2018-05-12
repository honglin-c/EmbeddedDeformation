#include "node.h"
#include <iostream>

Node::Node():transformed(false)
{
	rotation = glm::mat3(1.0f, 0.0f, 0.0f,
						 0.0f, 1.0f, 0.0f,
						 0.0f, 0.0f, 1.0f);
	translation = glm::vec3(0.0f, 0.0f, 0.0f);
}

Node::Node(glm::vec3 _position):position(_position), transformed(false)
{
	rotation = glm::mat3(1.0f, 0.0f, 0.0f,
						 0.0f, 1.0f, 0.0f,
						 0.0f, 0.0f, 1.0f);
	translation = glm::vec3(0.0f, 0.0f, 0.0f);
}

Node::~Node()
{}

void Node::setPosition(glm::vec3 _position)
{
	position = _position;
}

glm::vec3 Node::getPosition() const
{
	return position;
}

glm::vec3 Node::getTranslation() const
{
	return translation;
}


void Node::addDeltaRotation(glm::mat3 delta)
{
	rotation += delta;
}

void Node::addDeltaTranslation(glm::vec3 delta)
{
	translation += delta;
}


glm::vec3 Node::applyMapping(glm::vec3 p)
{
	return rotation * (p - position) + position + translation;
}

void Node::setTransformation(glm::mat3 &_rotation, glm::vec3 &_translation)
{
	rotation = _rotation;
	translation = _translation;
	transformed = true;
}


void print(const glm::vec3 &v)
{
    std::cout << v[0] << ", " << v[1] << ", " << v[2] << std::endl;
}

glm::vec3 Node::transformPosition(glm::vec3 vpos)
{
	if(transformed)
	{
		std::cout << "transform position:" << std::endl;
		print(rotation[0]);
		print(rotation[1]);
		print(rotation[2]);
		print(vpos);
		print(position);
		print(translation);
		print(rotation * (vpos - position) + position + translation);
		return (rotation * (vpos - position) + position + translation);
	}
	else{
		std::cout << "non-transform position:" << std::endl;
		print(vpos);
		return vpos;
	}
}

glm::vec3 Node::transformNormal(glm::vec3 normal)
{
	if(transformed)
	{
		std::cout << "transform normal:" << std::endl;
		print(normal);
		print(glm::inverse(rotation)[0]);
		print(glm::inverse(rotation)[1]);
		print(glm::inverse(rotation)[2]);
		print(glm::inverse(rotation) * normal);
		return glm::inverse(rotation) * normal;
	}
	else
	{
		std::cout << "non-transform normal:" << std::endl;
		print(normal);
		return normal;
	}
}

void Node::addNeighbor(Node * n)
{
	neighbors.insert(n);
}

std::set<Node *> Node::getNeighbors()
{
	return neighbors;
}


float Node::getRotValue()
{
	glm::vec3 c1 = glm::column(rotation, 0);
	glm::vec3 c2 = glm::column(rotation, 1);
	glm::vec3 c3 = glm::column(rotation, 2);
	float c12 = glm::dot(c1, c2);
	float c13 = glm::dot(c1, c3);
	float c23 = glm::dot(c2, c3);
	float c11 = glm::dot(c1, c1);
	float c22 = glm::dot(c2, c2);
	float c33 = glm::dot(c3, c3);
	return (c12 * c12 + c13 * c13 + c23 * c23
		 + (c11 - 1.0f) * (c11 - 1.0f) + (c22 - 1.0f) * (c22 - 1.0f) + (c33 - 1.0f) * (c33 - 1.0f));
}

float Node::getRegValue()
{
	float reg = 0.0f;
	for(auto n:neighbors)
	{
		glm::vec3 npos = n->getPosition();
		glm::vec3 nt = n->getTranslation();
		float norm2 = glm::length(rotation * (npos - position) + position + translation - (npos + nt));
		reg += norm2 * norm2;
	}
	return reg;
}

// Get [(c1*c2) (c1*c3) (c2*c3) (c1*c1-1) (c2*c2-1) (c3*c3-1)]
VectorXf Node::getRotTerm()
{
	VectorXf rot(6);
	glm::vec3 c1 = glm::column(rotation, 0);
	glm::vec3 c2 = glm::column(rotation, 1);
	glm::vec3 c3 = glm::column(rotation, 2);
	rot[0] = glm::dot(c1, c2);
	rot[1] = glm::dot(c1, c3);
	rot[2] = glm::dot(c2, c3);
	rot[3] = glm::dot(c1, c1) - 1.0f;
	rot[4] = glm::dot(c2, c2) - 1.0f;
	rot[5] = glm::dot(c3, c3) - 1.0f;
	return rot;
}

// Get all neighbor's [Rj * (gk - gj) + gj + tj - (gk + tk)]
MatrixXf Node::getRegTerm()
{
	MatrixXf reg(neighbors.size(), 3);
	int count = 0;
	for(auto n:neighbors)
	{
		glm::vec3 npos = n->getPosition();
		glm::vec3 nt = n->getTranslation();
		glm::vec3 regterm = rotation * (npos - position) + position + translation - (npos + nt);
		reg(count, 0) = regterm[0];
		reg(count, 1) = regterm[1];
		reg(count, 2) = regterm[2];
		count ++;
	}
	return reg;
}

Matrix3f Node::matRotation() const
{
	Matrix3f rot;
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			rot(i, j) = rotation[i][j];
	return rot;
}

// Get a certain neighbor's [Rj * (gk - gj) + gj + tj - (gk + tk)]
Vector3f Node::getRegTerm(Node * neighbor)
{
	Vector3f reg;
	glm::vec3 npos = neighbor->getPosition();
	glm::vec3 nt = neighbor->getTranslation();
	glm::vec3 regterm = rotation * (npos - position) + position + translation - (npos + nt);
	reg =  Vector3f(regterm[0], regterm[1], regterm[2]);
	return reg;
}


