#ifndef ANIMATE_TARGET_FUNCTION
#define ANIMATE_TARGET_FUNCTION

#include "../common/param.h"
#include "../common/residualFunction.h"

class AnimateTargetFunction: public ResidualFunction
{
public:
	AnimateTargetFunction(std::shared_ptr<Param> param);
	~AnimateTargetFunction();

	Eigen::SparseMatrix<double> calcJf(std::shared_ptr<Param> param);
	Eigen::VectorXd 			calcfx(std::shared_ptr<Param> param);
private:


}

#endif