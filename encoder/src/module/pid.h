#ifndef PID_H
#define PID_H

class PID{
    public:
        #define ON 1
        #define OFF 0

        PID(double*, double*, double*, double, double, double, double, double, unsigned long, double, bool=false);

        void setMode(bool Mode);

        bool virtual compute();

        void setSampleTime();

        void setLimit(double Min, double Max);

        void setTuning(double Kp, double Ki, double Kd);

        void setSetpoint(double Setpoint);

        void setInput(double Input);

        double getOutput();

        bool isDone();

        void resetDone();

        void setMinMax(double);

        void resetMI();

        void resetMinMax();

        void resetTimedOut();
        bool isTimedOut();

    protected:
        
        double Kp;                  // * (P)roportional Tuning Parameter
        double Ki;                  // * (I)ntegral Tuning Parameter
        double Kd;                  // * (D)erivative Tuning Parameter

        double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
        double *myOutput;             //   This creates a hard link between the variables and the 
        double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                    //   what these values are.  with pointers we'll just know.

        double last_error;

        double min, max, acceptableError;
        bool mode, timeout, timedOut;

        unsigned long sampleTime;
                
        unsigned long lastTime, start;

        double last_I;
        double MI,MP;
        double virtual computeError(double, double);

        int done;
        bool sign;
        bool isAngle;
        bool done_from_timeout;
};

#endif