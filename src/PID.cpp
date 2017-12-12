#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  twiddle = false;

  update_iters = 0;
  evaluate_iters = 0;
  initial_steps = 100;

  err = 0.0;
  param_idx = 2;
  state = init;

  dp = {Kp * 0.1, Kd * 0.1, Ki * 0.1};
  best_err = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  update_iters++;
  evaluate_iters++;
  if (twiddle && update_iters > initial_steps) {
    err += (cte * cte) / evaluate_iters;
    Twiddle();
  }
}

double PID::TotalError() {
  return -(Kp * p_error + Kd * d_error + Ki * i_error);
}

void PID::Twiddle() {
  if (evaluate_iters > 1000) {
    // Print previous Ks before starting next optimizations
    cout << "Kp: " << Kp << endl;
    cout << "Kd: " << Kd << endl;
    cout << "Ki: " << Ki << endl;
    // END
    switch(state) {
      case init:
        evaluate_iters = 0;
        err = 0.0;
        param_idx = (param_idx + 1) % 3;
        TuneParam(param_idx, dp[param_idx]);
        state = first;
        break;
      case first:
        if (err < best_err) {
          best_err = err;
          dp[param_idx] *= 1.1;
          state = init;
        } else {
          TuneParam(param_idx, -2 * dp[param_idx]);
          state = second;
        }
        break;
      case second:
        if (err < best_err) {
          best_err = err;
          dp[param_idx] *= 1.1;
          state = init;
        } else {
          TuneParam(param_idx, dp[param_idx]);
          dp[param_idx] *= 0.9;
          state = init;
        }
        break;
    }
  }
}

void PID::TuneParam(int index, double val) {
  if (index == 0) {
    Kp += val;
  }
  if (index == 1) {
    Kd += val;
  }
  if (index == 2) {
    Ki += val;
  }
}
