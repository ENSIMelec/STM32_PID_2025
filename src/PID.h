#ifndef PID_h
#define PID_h

class PID
{

public:
// Constants used in some of the functions below
#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

  // commonly used functions **************************************************************************
  PID(float *, float *, float *,             // * constructor.  links the PID to the Input, Output, and
      float, float, float, float, int, int); //   Setpoint.  Initial tuning parameters are also set here with SampleTime.
                                             //   (overload for specifying proportional mode)

  PID(float *, float *, float *,        // * constructor.  links the PID to the Input, Output, and
      float, float, float, float, int); //   Setpoint.  Initial tuning parameters are also set here with SampleTime

  void SetMode(int Mode); // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute(); // * performs the PID calculation.  it should be
                  //   called every SampleTime cycles. ON/OFF and
                  //   calculation frequency can be set using SetMode
                  //   SetSampleTime respectively

  void SetOutputLimits(float, float, float); // * clamps the output to a specific range. 0-255 by default, but
                                             //   it's likely the user will want to change this depending on
                                             //   the application
  void IncreaseOutputLimits(float);          // * Increase the output limits by a specific value

  // available but not commonly used functions ********************************************************
  void SetTunings(float, float, // * While most users will set the tunings once in the
                  float);       //   constructor, this function gives the user the option
                                //   of changing tunings during runtime for Adaptive control
  void SetTunings(float, float, // * overload for specifying proportional mode
                  float, int);

  void SetControllerDirection(int); // * Sets the Direction, or "Action" of the controller. DIRECT
                                    //   means the output will increase when error is positive. REVERSE
                                    //   means the opposite.  it's very unlikely that this will be needed
                                    //   once it is set in the constructor.
  void SetSampleTime(float);        // * sets the frequency, in seconds, with which
                                    //   the PID calculation is performed.  default is 0.01

  // Display functions ****************************************************************
  float GetKp();      // These functions query the pid for interal values.
  float GetKi();      //  they were created mainly for the pid front-end,
  float GetKd();      // where it's important to know what is actually
  int GetMode();      //  inside the PID.
  int GetDirection(); //
  float GetOutputLimitMax();
  float GetOutputLimitMin();

private:
  void Initialize();

  float dispKp; // * we'll hold on to the tuning parameters in user-entered
  float dispKi; //   format for display purposes
  float dispKd; //

  float kp; // * (P)roportional Tuning Parameter
  float ki; // * (I)ntegral Tuning Parameter
  float kd; // * (D)erivative Tuning Parameter

  int controllerDirection;
  int pOn;

  float *myInput;    // * Pointers to the Input, Output, and Setpoint variables
  float *myOutput;   //   This creates a hard link between the variables and the
  float *mySetpoint; //   PID, freeing the user from having to constantly tell us
                     //   what these values are.  with pointers we'll just know.

  float outputSum, lastInput;

  float SampleTime;
  float outMin, outMax, deadZone;
  bool inAuto, pOnE;
};
#endif