#include "PID.h"

PID::PID() : is_initialized(false) {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double cte_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  cte_prev = cte_;
  cte_i = 0;

  is_initialized = true;
}

void PID::UpdateError(double cte) {
  p_error = Kp * cte;
  d_error = Kd * (cte - cte_prev);

  // If the vehicle crosses the track center, set the integral cte to zero to avoid overshooting
  cte_i = cte * cte_prev < 0 ? 0 : cte_i + cte;
  cte_i += cte;
  i_error = Ki * cte_i;
}

double PID::TotalError() {
  double total_error = - p_error - i_error - d_error;
  return total_error;
}