#ifndef PID_H
#define PID_H

#include <vector>
#include <sstream>

class PID {
public:
  /*
  * Errors
  */
  double p_error; // Error of P-element
  double i_error; // Error of I-element
  double d_error; // Error of D-element

  /*
  * Coefficients
  */ 
  double Kp; // Coefficient for P-element
  double Ki; // Coefficient for I-element
  double Kd; // Coefficient for D-element

  /*
   * Others
   */
  std::string class_name;        // For Debug, to differentiate between classes
  bool reset_run;                // Information to main.cpp if the simulator should be resetted
  bool first_step;               // During first step of each run new koefficients, etc. will be calculated for twiddling
  unsigned long step;            // Counts number of simulation steps
  unsigned long maxstep;         // Range of steps [0,maxstep] where twiddling of coefficients is enabled;
                                 // Violation of maxcte beyond maxstep will set this parameter to the violation step and reset the run
  unsigned long roundsteplimit;  // Parameter to estimate one lap
                                 // @30MPH: an ideal lap has ~4500 steps, S-line driving leads more to 6000 steps
                                 // @70MPH: it takes several laps until 6000 steps are reached

  /*
   * Twiddle parameter
   */
  const std::vector<double*> tau = {&Kp, &Kd, &Ki};   // Pointer to coefficient; enables index-based addressing of coefficients
  bool twiddle_allowed;                               // Switches on twiddle-mode
  bool twiddle;                                       // Switches on twiddling-phase where coefficients are optimized
  std::vector<double> dp;                             // Range [-<param>,+<param>] in which the according coeffcient is changed

  double maxcte;                                      // Max error to be allowed
  double besterr;                                     // For debug: Best error reached
  double avgrunerr;                                   // Average error over number of steps used for twiddling
  double errsum;                                      // Sum of errors used for twiddling
  unsigned int phase;                                 // Phase of twiddling:
                                                      //    - addition of according dp value
                                                      //    - subtraction of double dp value
                                                      //    - after subtraction, preparing for addition of next coefficient
  unsigned int paramidx;                              // Index pointing to current coefficient and dp value for twiddling

  /*
   * Debug parameter
   */
  unsigned long cteinfullstepsizecnt;                 // Counts maxcte violations while step is smaller than maxstep
  unsigned long paramchangecnt;                       // Counts how often the coefficients have been changed in total
  unsigned long maxstepinccnt;                        // Counts increase of maxstep
  unsigned long totalstepcnt;                         // Counts total number of steps taken
  bool printedfinal;                                  // Only one-time printout of info "You finalized a lap ..."

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
  void Init(double Kp, double Ki, double Kd, bool twiddle, std::string class_name);

  /*
  * Reset full run [maxstep reached] (despite constant values) for twiddle.
  */
  void ResetRun();

  /*
  * Reset during run [here: not maxstep reached] for twiddle.
  */
  void Reset();

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
