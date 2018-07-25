#ifndef _GAUSS_NEWTON_SOLVER_H
#define _GAUSS_NEWTON_SOLVER_H

#include <memory>

class ResidualFunction;
class Param;

class GaussNewtonSolver
{
public:
	virtual bool solve(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param) = 0;
	virtual bool solveSingleStep(std::shared_ptr<ResidualFunction> f, std::shared_ptr<Param> param) = 0;
};

#endif