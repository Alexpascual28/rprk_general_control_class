#ifndef motors_h
#define motors_h

#include "Arduino.h"
#include <ARB.h>

// Define macros
#define PI 3.1415926535897932384626433832795

// PID

#define REG_RECEIVE_KP_A 30 // Receives proportional multiplier of motor A
#define REG_RECEIVE_KP_A_DEC 31 // Receives proportional multiplier of motor A (decimal part)

#define REG_RECEIVE_KP_B 32 // Receives proportional multiplier of motor B
#define REG_RECEIVE_KP_B_DEC 33 // Receives proportional multiplier of motor B (decimal part)

#define REG_RECEIVE_KI_A 34 // Receives integral multiplier of motor A
#define REG_RECEIVE_KI_A_DEC 35 // Receives integral multiplier of motor A (decimal part)

#define REG_RECEIVE_KI_B 36 // Receives integral multiplier of motor B
#define REG_RECEIVE_KI_B_DEC 37 // Receives integral multiplier of motor B (decimal part)

#define REG_RECEIVE_KD_A 38 // Receives derivative multiplier of motor A
#define REG_RECEIVE_KD_A_DEC 39 // Receives derivative multiplier of motor A (decimal part)

#define REG_RECEIVE_KD_B 40 // Receives derivative multiplier of motor B
#define REG_RECEIVE_KD_B_DEC 41 // Receives derivative multiplier of motor B (decimal part)

#define REG_RECEIVE_SETPOINT_A 42 // PID Setpoint of wheel A
#define REG_RECEIVE_SETPOINT_A_DEC 43 // PID Setpoint of wheel A (decimal part)

#define REG_RECEIVE_SETPOINT_B 44 // PID Setpoint of wheel B
#define REG_RECEIVE_SETPOINT_B_DEC 45 // PID Setpoint of wheel B (decimal part)

#define REG_RECEIVE_TIMECHANGE 46 // Millisecond delay between PID loop iterations
#define REG_RECEIVE_STEP_DISTANCE 47 // Distance per step
#define REG_RECEIVE_DISTANCE_BETWEEN_WHEELS 48 // Distance between wheels in centimetres
#define REG_RECEIVE_WHEEL_DIAMETER 49 // Wheel diameter

// POSE

#define REG_SEND_POSE_X 50 // Robot pose x
#define REG_SEND_POSE_X_CM 51 // Robot pose x (decimal part)

#define REG_SEND_POSE_Y 52 // Robot pose y
#define REG_SEND_POSE_Y_CM 53 // Robot pose y (decimal part)

#define REG_SEND_POSE_W 54 // Robot pose w
#define REG_SEND_POSE_W_CM 55 // Robot pose w (decimal part)

// DRIVE DATA

#define REG_RECEIVE_STOP_SIGNAL 56 // PID Stop signal
#define REG_RECEIVE_MSG_DRIVE 57 // Receive drive control data
#define REG_RECEIVE_GOAL_MARGIN 58 // PID goal margin

#define REG_SEND_MSG_DRIVE 59 // Receive drive control data
#define REG_SEND_EXIT_CODE 60 // Receive drive control data

class Motors {
  public:
    Motors();
    void initialize();
    void runMotors();
    
  private:
    // HELPER ENUMS
    typedef enum {CW,CCW} m_wheelDirection; // CW = 0, CCW = 1
    typedef enum {A,B} m_Motor; // A = 0, B = 1
    typedef enum {FORWARD, RIGHT, BACKWARD, LEFT} m_robotDirection;

    // STATIC MEMBER VARIABLE INITIALISATION
    static m_wheelDirection m_dirA;
    static m_wheelDirection m_dirB;
    
    // Variables to store step count from motors, must be volatlie to update from within ISR
    volatile static int m_stepsA;
    volatile static int m_stepsB;
  
    // ROBOT POSE
    double m_pose[3] = {0.0, 0.0, 0.0};

    // PID
    // Motor A tunings
    double m_kpA = 0.8;
    double m_kiA = 0.2;
    double m_kdA = 0.5;
    
    // Motor B tunings
    
    double m_kpB = 0.8;
    double m_kiB = 0.2;
    double m_kdB = 0.5;

    float m_goalMargin = 0.01;

    double m_errorSumA = 0;
    double m_errorSumB = 0;
    double m_lastErrorA = 0;
    double m_lastErrorB = 0;

    double m_setpointA, m_setpointB; // Motor goals as distance (setpoint)

    const unsigned long m_timeChange = 1; // 1 millisecond delay per PID compute
    const double m_stepDistance_cm = 5; // Distance per step
    
    // ODOMETRY
    
    const double m_distanceBetweenWheels = 20; // In cm
    const double m_wheelDiameter = 4.75;
    volatile int m_inputPrev = 0;

    // MEMBER METHODS

    // Initialisation methods
    void m_setPinModes(); // Sets pins and pin modes of all components
    void m_attachInterrupts(); // Attaches interrupts
    void m_initializeSerialRegisters(); // Initialize serial registers

    // PID functions
    void m_readPidTunningSettings();
    void m_readSetpoints();
    int m_computePID(double t_setpointA, double t_setpointB, bool stopAtGoal);

    double m_stepsToCentimetres(int t_steps);
    void m_resetErrors(m_Motor t_motor);

    void m_setMotorSpeed(m_Motor t_motor, double t_speedPWM);
    void m_calculateCurrentPose(double t_distanceA, double t_distanceB);

    // REGISTER DATA MANAGEMENT
    
    void m_send16BitNumber(int t_register1, int t_register2, int t_number);
    void m_sendDecimalToRegisters(int t_register1, int t_register2, float t_number);
    double m_read16BitNumber(int t_register1, int t_register2);
    double m_readDecimalNumberFromRegisters(int t_register1, int t_register2);
  
    // General movement
    void m_readDirectionInput();

    void m_rotateRobot(double t_angle_radians);
    void m_advanceRobot(double t_distance_cm);
    void m_moveRobot(m_robotDirection t_robotDirection, unsigned long t_duration);
    void m_aimRobot(m_robotDirection t_robotDirection);
    
    // Encoder interrupts
    static void m_ENCA_ISR();
    static void m_ENCB_ISR();
};

#endif
