#ifndef _ANIMATE_TARGET_FUNCTION_H
#define _ANIMATE_TARGET_FUNCTION_H

#include "../common/param.h"
#include "../common/residualFunction.h"
#include "../graph/deformTargetFunction.h"

class AnimateTargetFunction: public DeformTargetFunction
{
public:
	AnimateTargetFunction(std::shared_ptr<Param> param);
	~AnimateTargetFunction();

	void setTimestep(double timestep);

	Eigen::SparseMatrix<double> calcJf(std::shared_ptr<Param> param);
	Eigen::VectorXd 			calcfx(std::shared_ptr<Param> param); // use vo in 1/2 * mv^2: for debug

	Eigen::VectorXd 			calcfx_Ep(std::shared_ptr<Param> param);
	Eigen::SparseMatrix<double> calcfx_Ek1(std::shared_ptr<Param> param);
	Eigen::VectorXd 			calcfx_Ek2(std::shared_ptr<Param> param);

	double						calcFx(std::shared_ptr<Param> param);
	Eigen::VectorXd             calcJF(std::shared_ptr<Param> param);

	int kin_begin;
private:
	double timestep;
	const double m = 1; // M = m^2 * I
	// Calculate Kinetic Energy in fx and Jf
	void calcJfKin(std::shared_ptr<Param> param, std::vector<Tf> &tripletList);

	void calcfxKin(std::shared_ptr<Param> param, Eigen::VectorXd &fx);

	void setOrder(std::shared_ptr<Param> param);

};

#endif