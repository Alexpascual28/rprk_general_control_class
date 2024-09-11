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
#define REG_RECEIVE_WHEEL_DIAMETER_DEC 50 // Wheel diameter

// POSE

#define REG_SEND_POSE_X 51 // Robot pose x
#define REG_SEND_POSE_X_DEC 52 // Robot pose x (decimal part)

#define REG_SEND_POSE_Y 53 // Robot pose y
#define REG_SEND_POSE_Y_DEC 54 // Robot pose y (decimal part)

#define REG_SEND_POSE_W 55 // Robot pose w
#define REG_SEND_POSE_W_DEC 56 // Robot pose w (decimal part)

// DRIVE DATA

#define REG_RECEIVE_GOAL_MARGIN 57 // PID goal margin
#define REG_RECEIVE_GOAL_MARGIN_DEC 58 // PID goal margin

#define REG_RECEIVE_PID_SIGNAL 59 // PID Data signal
#define REG_SEND_EXIT_CODE 60 // Send exit code drive control data

#define REG_RECEIVE_CONTROL_MODE 61 // Receives control mode input

#define REG_RECEIVE_SPEED_DATA 62 // Receive speed data
#define REG_RECEIVE_MSG_DRIVE 63 // Receive drive control data
#define REG_SEND_MSG_DRIVE 64 // Send drive control data

// HANDSHAKE

#define REG_SEND_CONFIRM 65 // Send handshake signal
#define REG_RECEIVE_ACK 66 // Receive handshake signal

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

    // Handshake variables
    int m_timeout_ms = 5000;
  
    // ROBOT POSE
    double m_pose[3] = {0.0, 0.0, 0.0};

    // PID
    int m_pidControlMode = 0;
    
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

    int m_timeChange = 1; // 1 millisecond delay per PID compute
    int m_stepDistance_cm = 5; // Distance per step

    int m_pidSignal = 0; // PID data signal to control PID computing
    
    // ODOMETRY
    
    int m_distanceBetweenWheels = 20; // In cm
    double m_wheelDiameter = 4.75;

    volatile int m_speedValuePrev = 0;
    volatile int m_inputPrev = 0;

    // MEMBER METHODS

    // Initialisation methods
    void m_setPinModes(); // Sets pins and pin modes of all components
    void m_attachInterrupts(); // Attaches interrupts
    void m_initializeSerialRegisters(); // Initialize serial registers

    // REGISTER UPDATE
    void m_updateVariableFromRegister(int t_variable, int t_register);
    void m_updateDecimalVariableFromRegisters(double t_variable, int t_register1, int t_register2);

    // PID functions
    void m_readControlModeRegister();
    void m_readPidTunningSettings();
    void m_readSetpoints();
    void m_readOdometrySettings();
    void m_readPidSignal();
    
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
    void m_readSpeedLevelValue();
    void m_readDirectionInput();
    void m_adjustSpeed(int t_speedLevel);
    void m_motorSetDir(m_Motor t_motor, m_wheelDirection t_dir);

    // Direction control no PID
    void m_moveForward();
    void m_moveBackward();
    void m_moveLeft();
    void m_moveRight();
    void m_stopRobot();

    // Direction control PID
    void m_rotateRobot(double t_angle_radians);
    void m_advanceRobot(double t_distance_cm);
    void m_moveRobot(m_robotDirection t_robotDirection, int t_duration);
    void m_aimRobot(m_robotDirection t_robotDirection);
    void m_sendRobotPose(double t_pose[3]);
    
    // Encoder interrupts
    static void m_ENCA_ISR();
    static void m_ENCB_ISR();
};

#endif
