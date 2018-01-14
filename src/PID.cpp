#include "PID.h"

#include <cmath>
#include <iostream>
#include <cstdio>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle, std::string class_name) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  PID::class_name = class_name;

  reset_run = false;
  first_step = true;

  roundsteplimit = 6000; 
  maxstep = 1000;
  step = 1;

  PID::twiddle_allowed = twiddle;
  maxcte = 4.0;
  //maxcte = 2.5;
  PID::twiddle = twiddle;
  dp = {1.0, 1.0, 1.0};
  phase = 0b001;
  paramidx = 0;

  avgrunerr = numeric_limits<double>::max();
  besterr = numeric_limits<double>::max()/maxstep;
  errsum = 0.0;

  cteinfullstepsizecnt = 0;
  paramchangecnt = 0;
  maxstepinccnt = 0;
  totalstepcnt = 0;
  printedfinal = false;
    
}

void PID::ResetRun() {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  reset_run = false;
  first_step = true;

  maxstep = step;
  step = 1;

  twiddle = true;
  phase = 0b001;
  paramidx = 0;

  avgrunerr = numeric_limits<double>::max();
  errsum = 0.0;

  cteinfullstepsizecnt = 0;
  paramchangecnt = 0;
  printedfinal = false;
}

void PID::Reset() {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  reset_run = true;
  first_step = true;

  step = 1;

  twiddle = true;
  
  avgrunerr = errsum / step;
  errsum = numeric_limits<double>::max();
  printedfinal = false;
}


void PID::UpdateError(double cte) {

  if (!first_step) {
  // In all but the first step:
  //    - All errors are updated
  //    - Step counts are increased
  //    - For debugging: Lap finalization is checked and printed
  //    - In twiddle-mode:
  //         - If in twiddle-phase: Calculation of errorsum as a sum of cte squares
  //         - If abs(cte) > maxcte: Checks where it happens (<maxstep, <roundsteplimit, >roundsteplimit) and updates debug counters & resets accordingly

    // Update of errors
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    // Update of step counters
    step++;
    totalstepcnt++;

    // Lap finalization (step > roundsteplimit)?
    if (twiddle_allowed && !printedfinal && (step > roundsteplimit)) {
      cout << "          *********************************************************************" << endl;
      cout << "          * You passed a full lap (more or less) , step > roundsteplimit=" << roundsteplimit << " *" << endl; 
      cout << "          *********************************************************************" << endl;
      cout << "             [" << class_name << "] (Kp=" << Kp << ", Kd=" << Kd << ", Ki=" << Ki << ", dp={" << dp[0] << "," << dp[1] << "," << dp[2] <<"}, besterr=" << besterr << ", err=" << errsum/(step - 1) << ")" << endl << endl;
      printedfinal = true;
    }
    

    // During twiddling: Calculate error sum, print out of info if simulator managed to get beyond maxstep (for debug)
    if (twiddle) {
 
      errsum += cte * cte;

      if (step > maxstep) {
        twiddle = false;
	cout << "**********" << endl;
	cout << "*** Info [" << class_name << "] (" << totalstepcnt << ":" << maxstepinccnt << ":" << paramchangecnt << ":" << cteinfullstepsizecnt << ", maxstep=" << maxstep << "): "
             << "Result for maxstep (dp={" << dp[0] << ", " << dp[1] << ", " << dp[2] << "}):" << endl
	     << "*** Kp=" << Kp << ", Kd=" << Kd << ", Ki=" << Ki << " (besterr=" << besterr << ", err=" << errsum/step << ")" << endl;
	cout << "**********" << endl ;
	}
    }

    // In twiddle-mode: Check if and where (abs(cte) > maxcte) and take according actions (increase according debug counters, error printout, according reset)
    if (twiddle_allowed && (abs(cte) > maxcte)) {
      if (step < maxstep) {
	// Violation occured during twiddling phase:
	// We start from the beginning with next coefficient, with same dp, maxstep, and avgrunerr, with reset of errors, step, errsum (max), ... 
        cteinfullstepsizecnt++;
        cout << "    *** Error [" << class_name << "] (" << totalstepcnt << ":" << maxstepinccnt << ":" << paramchangecnt << ":" << cteinfullstepsizecnt << ", step=" << step << "): "
             << "Max CTE, reset "
             << "(Kp=" << Kp << ", Kd=" << Kd << ", Ki=" << Ki << ", dp={" << dp[0] << "," << dp[1] << "," << dp[2] <<"})" << endl;
        Reset();
      } else {
	if (step <= roundsteplimit) {
	  // Violation occured outside twiddling phase but before finalizing one lap (more or less):
	  // We return to start with updated (extended till violation) maxstep, with same coefficients and dp but at first coefficient, with reset of errors, avgerrsum, step, errsum (zero), ...   
	  maxstepinccnt++;
	  cout << "**********" << endl;
          cout << "*** Error [" << class_name << "] (" << totalstepcnt << ":" << maxstepinccnt << ":" << paramchangecnt << ":" << cteinfullstepsizecnt << ", step=" << step << "): "
               << "Max CTE beyond maxstep (" << maxstep << "), setting to " << step << " and restart from init "
               << "(Kp=" << Kp << ", Kd=" << Kd << ", Ki=" << Ki << ", dp={" << dp[0] << "," << dp[1] << "," << dp[2] <<"})" << endl;
  	  cout << "**********" << endl;
	  ResetRun();
        } else {
	  // Violation occured after finalization of a lap (more or less):
          // We start from the beginning with next coefficient, with same dp, maxstep, and avgrunerr, with reset of errors, step, errsum (max), ...
	  // and turn into twiddling-phase again
	  cout << "**********" << endl;
          cout << "*** Error [" << class_name << "] (" << totalstepcnt << ":" << maxstepinccnt << ":" << paramchangecnt << ":" << cteinfullstepsizecnt << ", step=" << step << "): "
               << "Max CTE beyond maxstep (" << maxstep << ") = roundsteplimit, just reset "
               << "(Kp=" << Kp << ", Kd=" << Kd << ", Ki=" << Ki << ", dp={" << dp[0] << "," << dp[1] << "," << dp[2] <<"})" << endl;
  	  cout << "**********" << endl;
	  twiddle = true;
          Reset();
	}
      }
     }

  } else {
  // During first step:
  //    - Only P and I errors can be updated (no previous error for D-element)
  //    - In twiddle-phase: The actual coefficient (paramidx) is updated according to its operation (phase):
  //      (The phases habe been rearranged to enable continued simulation right after the switch statement)
  //         - 0b001: addition of according dp value
  //         - 0b010: subtraction of double dp value
  //         - 0b100: after subtraction, preparation for addition of next coefficient
    
    first_step = false;

    // Update of P and I errors
    p_error = cte;
    i_error += cte;
    
    if (twiddle) {

      switch(phase) {
      // after subtraction, preparation for addition of next coefficient
      case 0b100: if (avgrunerr < besterr) {
  	  	    besterr = avgrunerr;
		    dp[paramidx] *= 1.1;
	          } else {
		    *tau[paramidx] += dp[paramidx];
		    dp[paramidx] *= 0.9;
	          }
		  phase = 0b001;
      	          paramidx = (paramidx + 1) % 3;
      	          //paramidx = (paramidx + 1) % 2;
      // addition of according dp value
      case 0b001: *tau[paramidx] += dp[paramidx];
                  phase *= 2;
                  break;
      // subtraction of double dp value
      case 0b010: if (avgrunerr < besterr) {
   	    	    besterr = avgrunerr;
		    dp[paramidx] *= 1.1;
      	            paramidx = (paramidx + 1) % 3;
      	            //paramidx = (paramidx + 1) % 2;
                    *tau[paramidx] += dp[paramidx];
	          } else {
		    *tau[paramidx] -= 2 * dp[paramidx];
		    phase *= 2;
	          }
      }
      paramchangecnt++;
      //cout << "*** Info (" << paramchangecnt << ":" << cteinfullstepsizecnt << "): "
      //     << "New param (Kp=" << Kp << ", Kd=" << Kd << ", Ki=" << Ki << ")" << endl;
      if (Kp < 0) {
	// Avoiding useless simulation for Kp < 0; continuing with the next coefficient
	cout << "    *** Info [" << class_name << "] (" << totalstepcnt << ":" << maxstepinccnt << ":" << paramchangecnt << ":" << cteinfullstepsizecnt << "): "
	     << "Kp < 0, reset" << endl;
	Reset();
      }
    }
  }  
}

double PID::TotalError() {
  // Calculation of total deviation error of the differnt PID-elements
  return (- Kp*p_error - Kd*d_error - Ki*i_error);
}

