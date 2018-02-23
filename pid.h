#ifndef pid_h
#define pid_h
class pid{
    public:
        pid(double KP, double KI, double KD);
        double calculateOutput(unsigned long currentTime, double position, double setpoint);
    private:
        double KP;
        double KI;
        double KD;
        double lastError;
        double integral;
        double lastPosition;
        unsigned long lastTime;
        double limit(double input, double lowerbound, double upperbound);
};
#endif

