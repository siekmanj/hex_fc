#ifndef imu_h
#define imu_h
class IMU{
  public:
    IMU();
    void calibrate(int precision);
    void update();
    void centerGyro();

    double getPitch();
    double getRoll();
    double getYaw();
    
  private:
    double gyroX;
    double gyroY;
    double gyroZ;
    
    double accelX;
    double accelY;
    double accelZ;
    
    double gyroXdrift;
    double gyroYdrift;
    double gyroZdrift;

    double pitchEstimate;
    double rollEstimate;
    double yawEstimate;

    double accBiasRoll;
    double accBiasPitch;

    double convert_to_degrees;

    unsigned long last_t;
};
#endif
