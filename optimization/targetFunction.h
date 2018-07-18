#ifndef _TARGET_FUNCTION_H
#define _TARGET_FUNCTION_H

#include "../common/param.h"
#include "../common/residualFunction.h"
#include "../graph/graphVertex.h"
#include "../graph/node.h"

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>

typedef Eigen::SparseMatrix<double> SparseMd;
typedef Eigen::Triplet<double> Tf;

class TargetFunction: public ResidualFunction
{
public:
	TargetFunction(std::shared_ptr<Param> param);
	~TargetFunction();

	Eigen::SparseMatrix<double> calcJf(std::shared_ptr<Param> param);
	Eigen::VectorXd 			calcfx(std::shared_ptr<Param> param);

	int reg_begin;
	int con_begin;
private:
	const int 	 k = 4;
	const double sqrt10 = 3.16227766;
	const int 	 x_rt = 12;
	const double w_rot = 1.0;
	const double w_reg = 10.0;
	const double w_con = 100.0;

	int x_order;
	int fx_order;

	void setOrder(std::shared_ptr<Param> param);

	void calcJfRot(std::shared_ptr<Param> param, std::vector<Tf> &tripletList);
	void calcJfReg(std::shared_ptr<Param> param, std::vector<Tf> &tripletList);
	void calcJfCon(std::shared_ptr<Param> param, std::vector<Tf> &tripletList);

	void calcfxRot(std::shared_ptr<Param> param, Eigen::VectorXd &fx);
	void calcfxReg(std::shared_ptr<Param> param, Eigen::VectorXd &fx);
	void calcfxCon(std::shared_ptr<Param> param, Eigen::VectorXd &fx);

	void debug(const std::string s);
};

#endif