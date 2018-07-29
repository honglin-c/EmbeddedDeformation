/*
 * A simple example to show the framework
 * of physically based simulation
 */

#include <iostream>
#include <cmath>

#define MAXITER 20000
#define EPSILON 1e-12

using namespace std;

//-> One dimensional harmonic oscillator:  k/2*(|x|-d)^2
class one_dim_spring_energy
{
public:
  one_dim_spring_energy(const double k, const double len)
    : k_(k), len_(len) {}
  int Val(double *val, const double *x) const {
    *val += 0.5*k_*(fabs(*x)-len_)*(fabs(*x)-len_);
    return 0;
  }
  int Gra(double *gra, const double *x) const {
    *gra += k_*(fabs(*x)-len_)*(*x)/fabs(*x);
    return 0;
  }
  int Hes(double *hes, const double *x) const {
    *hes += k_;
    return 0;
  }
private:
  const double k_, len_; // stiffness and rest length
};

//-> variational implicit Euler
class impl_euler_energy
{
 public:
  impl_euler_energy(const double x0, const double m, const double h)
      : m_(m), h_(h), vn_(0), xn_(x0) {}
  int Val(double *val, const double *x) const {
    *val += 0.5 * pow(sqrt(m_)*((*x-xn_)/h_-vn_), 2);
    return 0;
  }
  int Gra(double *gra, const double *x) const {
    *gra += m_*((*x-xn_)/h_-vn_)/h_;
    return 0;
  }
  int Hes(double *hes, const double *x) const {
    *hes += m_/(h_*h_);
    return 0;
  }
  void update(const double *x) { // update the position and velocity
    vn_ = (*x-xn_)/h_;
    xn_ = *x;
  }
 private:
  const double m_, h_;
  double xn_, vn_;  // pos and vel of last frame
};

static double E;   // total energy
static double Ep;
static double Ek;       
static double m;   // mass
static double h;   // time step
static double x;   // position
static double v;   // velocity
static one_dim_spring_energy *Ev; // potential
static impl_euler_energy *Et;     // variational impl euler

//-> advance the system via Newton's method
void advance() {

  double xstar = x;
  double prev_x = xstar;

  Ep = Ek = 0;
  Ev->Val(&Ep, &xstar);
  Et->Val(&Ek, &xstar);
  E = Ep + Ek;

  for (size_t iter = 0; iter < MAXITER; ++iter) {
    //-> evaluate objective value
    double value = 0; {
      Ev->Val(&value, &xstar);
      Et->Val(&value, &xstar);
    }
    //-> evaluate objective gradient
    double grad = 0; {
      Ev->Gra(&grad, &xstar);
      Et->Gra(&grad, &xstar);
      grad *= -1;
    }
    //-> evaluate objective hessian
    double hess = 0; {
      Ev->Hes(&hess, &xstar);
      Et->Hes(&hess, &xstar);
    }
    //-> determine search direction, corresponds to solving Hx = g
    double dx = grad/hess;
    if ( fabs(dx) <= EPSILON*prev_x ) {
      // cout << "\t@CONVERGED\n";
      break;
    }
    prev_x = xstar;
    xstar += dx;

    //-> maybe line search is needed for robustness
    {
      // line search, just omit here.
    }
  }

  x = xstar;
}

int main(int argc, char *argv[])
{
  // set mass
  m = 1.0;
  // set initial velocity
  v = 0.0;
  // set initial position
  x = 2.0;
  // set time step
  h = 0.01;

  // init a spring energy
  Ev = new one_dim_spring_energy(100.0, 1.0);
  Et = new impl_euler_energy(x, m, h);

  // simulate
  for (size_t frame = 0; frame < 500; ++frame) {
    // cout << "frame: " << frame << " position: " << x << endl;
    cout << x << " " << E << " " << Ep << " " << Ek << endl;
    
    advance();
    Et->update(&x);
  }

  delete Ev;
  delete Et;

  // cout << "[INFO] done\n";
  return 0;
}
