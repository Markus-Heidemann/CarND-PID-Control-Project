#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() : is_initialized(false) {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double cte_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  cte_prev = cte_;
  cte_i = 0;

  is_initialized = true;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = Kp * cte;
  d_error = Kd * (cte - cte_prev);
  cte_i += cte;
  i_error = Ki * cte_i;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return (- p_error - i_error - d_error);  // TODO: Add your total error calc here!
}