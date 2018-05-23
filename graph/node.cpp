#include "node.h"
#include <iostream>

Node::Node():transformed(false)
{
	rotation = Matrix3d::Identity();
	translation = Vector3d(0.0f, 0.0f, 0.0f);
}

Node::Node(glm::vec3 _position):position(Vector3d(_position[0], _position[1], _position[2])),
								transformed(false)
{
	rotation = Matrix3d::Identity();
	translation = Vector3d(0.0f, 0.0f, 0.0f);
}

Node::~Node()
{}

// void Node::setPosition(Vector3d _position)
// {
// 	position = _position;
// }

Vector3d Node::getPosition() const
{
	return position;
}

Vector3d Node::getTranslation() const
{
	return translation;
}


void Node::addDeltaRotation(Matrix3d delta)
{
	Matrix3d before = this->matRotation();
	rotation += delta;
	Matrix3d after = this->matRotation();
	Matrix3d ata = after.transpose() * after;
	// if(ata != Eigen::MatrixXd::Identity(3, 3))
	// {
	// 	std::cout << "before: " << std::endl;
	// 	std::cout << before << std::endl;
	// 	std::cout << "after: " << std::endl;
	// 	std::cout << after << std::endl;
	// 	std::cout << "A^T * A: " << std::endl;
	// 	std::cout << ata << std::endl;
	// 	// for(int i = 0; i < ata.rows(); i++)
	// 	// {
	// 	// 	for(int j = 0; j < ata.cols(); j++)
	// 	// 	{
	// 	// 		if(i != j && ata(i, j) > 1e-4)
	// 	// 			assert(0);
	// 	// 	}
	// 	// }
	// }
}

void Node::addDeltaTranslation(Vector3d delta)
{
	translation += delta;
}


Vector3d Node::applyMapping(Vector3d p)
{
	return rotation * (p - position) + position + translation;
}

void Node::setTransformation(Matrix3d &_rotation, Vector3d &_translation)
{
	rotation = _rotation;
	translation = _translation;
	transformed = true;
	// Matrix3d rot = this->matRotation();
	// assert(rot.transpose() * rot == MatrixXd::Identity(3, 3));
}


void print(const Vector3d &v)
{
    std::cout << v << std::endl;
}

Vector3d Node::transformPosition(Vector3d vpos)
{
	return (rotation * (vpos - position) + position + translation);
}

Vector3d Node::transformNormal(Vector3d normal)
{
	return rotation.inverse() * normal;
}

void Node::addNeighbor(Node * n)
{
	neighbors.insert(n);
}

std::set<Node *> Node::getNeighbors()
{
	return neighbors;
}


double Node::getRotValue()
{
	Vector3d c1 = rotation.col(0);
	Vector3d c2 = rotation.col(1);
	Vector3d c3 = rotation.col(2);
	double c12 = c1.dot(c2);
	double c13 = c1.dot(c3);
	double c23 = c2.dot(c3);
	double c11 = c1.dot(c1);
	double c22 = c2.dot(c2);
	double c33 = c3.dot(c3);
	return (c12 * c12 + c13 * c13 + c23 * c23
		 + (c11 - 1.0f) * (c11 - 1.0f) + (c22 - 1.0f) * (c22 - 1.0f) + (c33 - 1.0f) * (c33 - 1.0f));
}

double Node::getRegValue()
{
	double reg = 0.0f;
	for(auto n:neighbors)
	{
		Vector3d npos = n->getPosition();
		Vector3d nt = n->getTranslation();
		double norm2 = (rotation * (npos - position) + position + translation - (npos + nt)).norm();
		reg += norm2 * norm2;
	}
	return reg;
}

// Get [(c1*c2) (c1*c3) (c2*c3) (c1*c1-1) (c2*c2-1) (c3*c3-1)]
VectorXd Node::getRotTerm()
{
	VectorXd rot(6);
	Vector3d c1 = rotation.col(0);
	Vector3d c2 = rotation.col(1);
	Vector3d c3 = rotation.col(2);
	rot(0) = c1.dot(c2);
	rot(1) = c1.dot(c3);
	rot(2) = c2.dot(c3);
	rot(3) = c1.dot(c1) - 1.0f;
	rot(4) = c2.dot(c2) - 1.0f;
	rot(5) = c3.dot(c3) - 1.0f;
	return rot;
}

// Get all neighbor's [Rj * (gk - gj) + gj + tj - (gk + tk)]
MatrixXd Node::getRegTerm()
{
	MatrixXd reg(neighbors.size(), 3);
	int count = 0;
	for(auto n:neighbors)
	{
		Vector3d npos = n->getPosition();
		Vector3d nt = n->getTranslation();
		Vector3d regterm = rotation * (npos - position) + position + translation - (npos + nt);
		reg(count, 0) = regterm[0];
		reg(count, 1) = regterm[1];
		reg(count, 2) = regterm[2];
		count ++;
	}
	return reg;
}

Matrix3d Node::matRotation() const
{
	return rotation;
}

// Get a certain neighbor's [Rj * (gk - gj) + gj + tj - (gk + tk)]
Vector3d Node::getRegTerm(Node * neighbor)
{
	Vector3d reg;
	Vector3d npos = neighbor->getPosition();
	Vector3d nt = neighbor->getTranslation();
	Vector3d regterm = rotation * (npos - position) + position + translation - (npos + nt);
	return regterm;
}


