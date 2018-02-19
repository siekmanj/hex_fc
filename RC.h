#ifndef rc_h
#define rc_h
class RC{
  public:
    RC();
    int getCh1();
    int getCh2();
    int getCh3();
    int getCh4();
    int getCh5();
    int getCh6();

    double getPitchSetpoint();
    double getRollSetpoint();
    double getYawSetpoint();
    int getThrottle();
    
    bool connected();
    unsigned long timeSinceLastUpdate();

    void interruptHandler();
    
   private:
    int receiver_ch1;
    int receiver_ch2;
    int receiver_ch3;
    int receiver_ch4;
    int receiver_ch5;
    int receiver_ch6;
    
    unsigned long timer_channel_1;
    unsigned long timer_channel_2;
    unsigned long timer_channel_3;
    unsigned long timer_channel_4;
    unsigned long timer_channel_5;
    unsigned long timer_channel_6;

    int last_channel_1;
    int last_channel_2;
    int last_channel_3;
    int last_channel_4;
    int last_channel_5;
    int last_channel_6;
    unsigned long timer_1;
    unsigned long timer_2;
    unsigned long timer_3;
    unsigned long timer_4;
    unsigned long timer_5;
    unsigned long timer_6;
    unsigned long timeOfLastRead;
};


#endif
