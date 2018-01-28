#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using CppAD::AD;

// Definition of basic settings (target: To have them at one location)
class BaseSettings {
 public:
  const double Lf = 2.67;

  static const size_t N = 10;
  const double dt = 0.1;

  const double ref_v = 120;
  
  size_t x_start     = 0;
  size_t y_start     = x_start + N;
  size_t psi_start   = y_start + N;
  size_t v_start     = psi_start + N;
  size_t cte_start   = v_start + N;
  size_t epsi_start  = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start     = delta_start + N - 1;

  BaseSettings() {};
  ~BaseSettings() {};
};


class FG_eval : public BaseSettings {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs;};
  ~FG_eval() {};
  
  void operator()(ADvector& fg, const ADvector& vars);
};


class MPC : public BaseSettings {

 public:

  MPC() {};
  virtual ~MPC() {};

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
