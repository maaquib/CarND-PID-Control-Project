#ifndef PID_H
#define PID_H

#include <vector>

class PID {
private:
  enum PIDRunState { init, first, second };
  void Twiddle();
  void TuneParam(int index, double val);
  PIDRunState state;

  /*
   * Twiddle params
   */
  bool twiddle;
  double best_err;
  double err;
  int param_idx;
  int update_iters;
  int evaluate_iters;
  int initial_steps;
  std::vector<double> dp;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
