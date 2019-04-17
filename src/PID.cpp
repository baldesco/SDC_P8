#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
}

void PID::UpdateError(double cte)
{
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

double PID::TotalError()
{
  /**
   * Calculate and return the total error
   */
  return p_error + d_error + i_error;
}

double PID::NextAction()
{
  /**
   * Calculate and return the next control action based on the current error
   */
  return -1 * (Kp * p_error + Kd * d_error + Ki * i_error);
}