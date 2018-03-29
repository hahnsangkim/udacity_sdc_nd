#ifndef PID_H
#define PID_H

#define N 100

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double d_rate;
  double total_error;
  double best_err;
  
  /*
  * Coefficients
  */ 
  double p[3];
  double dp[3];

  double numSteps;
  bool first_run;
  bool second_run;
  int indx;
  const double TOL = 1e-5;

  double steering_drift;

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
  void Init(double Kp, double Ki, double Kd, double drift);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  double GetSteerValue();
  double GetSpeedValue();
  double GetCTERate();
  void twiddle();
};

#endif /* PID_H */
