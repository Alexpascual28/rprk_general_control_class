#ifndef motors_h
#define motors_h

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>
#include <PID_v1.h>

// Define macros
#define PI 3.1415926535897932384626433832795

// MOTORS

#define REG_RECEIVE_DIR_MOTOR_A 30 // Motor A receive direction
#define REG_RECEIVE_PWM_MOTOR_A 31 // Motor A receive PWM
#define REG_SEND_DATA_MOTOR_A 32 // Motor A send data (current direction and speed)

#define REG_RECEIVE_DIR_MOTOR_B 33 // Motor B receive direction
#define REG_RECEIVE_PWM_MOTOR_B 34 // Motor B receive PWM
#define REG_SEND_DATA_MOTOR_B 35 // Motor B send data (current direction and speed)

#define REG_RECEIVE_SPEED_DATA 40 // Receive speed data
#define REG_RECEIVE_MSG_DRIVE 41 // Receive drive control data
#define REG_SEND_MSG_DRIVE 42 // Send drive control data (current state)

// ENCODERS

#define REG_SEND_DATA_ENCODER_A_1 36 // Encoder A send data
#define REG_SEND_DATA_ENCODER_A_2 37 // Encoder A send data
#define REG_SEND_DATA_ENCODER_B_1 38 // Encoder B send data
#define REG_SEND_DATA_ENCODER_B_2 39 // Encoder B send data

#define REG_SEND_DISTANCE_A 43 // Calculated distance in wheel A
#define REG_SEND_DISTANCE_A_DEC 44 // Calculated distance in wheel A (decimal part)

#define REG_SEND_DISTANCE_B 45 // Calculated distance in wheel B
#define REG_SEND_DISTANCE_B_DEC 46 // Calculated distance in wheel B (decimal part)

#define REG_SEND_SPEED_A 47 // Calculated speed of wheel A
#define REG_SEND_SPEED_A_DEC 48 // Calculated speed of wheel A (decimal part)

#define REG_SEND_SPEED_B 49 // Calculated speed of wheel B
#define REG_SEND_SPEED_B_DEC 50 // Calculated speed of wheel B (decimal part)

#define REG_RECEIVE_RESET_ENCODER_A 51 // Resets absolute steps of encoder A
#define REG_RECEIVE_RESET_ENCODER_B 52 // Resets absolute steps of encoder B

#define REG_RECEIVE_CONTROL_MODE 53 // Receives control mode input

#define REG_RECEIVE_KP_A 54 // Receives proportional multiplier of motor A
#define REG_RECEIVE_KP_B 55 // Receives proportional multiplier of motor B

#define REG_RECEIVE_KI_A 56 // Receives integral multiplier of motor A
#define REG_RECEIVE_KI_B 57 // Receives integral multiplier of motor B

#define REG_RECEIVE_KD_A 58 // Receives derivative multiplier of motor A
#define REG_RECEIVE_KD_B 59 // Receives derivative multiplier of motor B

#define REG_RECEIVE_SETPOINT_A 70 // PID Setpoint of wheel A
#define REG_RECEIVE_SETPOINT_A_DEC 71 // PID Setpoint of wheel A (decimal part)

#define REG_RECEIVE_SETPOINT_B 72 // PID Setpoint of wheel B
#define REG_RECEIVE_SETPOINT_B_DEC 73 // PID Setpoint of wheel B (decimal part)

class Motors {
  public:
    Motors();
    void initialize();
    void runMotors();
    
  private:
    // Robot pose
    float m_pose[3] = {0.0, 0.0, 0.0};

    // PID
    int m_controlModeInputPrev;
    bool m_pidControlMode;
    
    double m_kpA, m_kiA, m_kdA; // Motor A tunings
    double m_kpB, m_kiB, m_kdB; // Motor B tunings

    volatile static int m_stepsA; // Variables to store step count from motors,
    volatile static int m_stepsB; // must be volatile to update from within ISR
    int m_stepsAprev, m_stepsBprev; // Flags to check if steps have changed

    double m_stepsA_cm, m_stepsB_cm; // Steps converted to cm (input)
    double m_speedA, m_speedB; // Motor speeds (output)
    double m_setpointA, m_setpointB; // Motor goals as distance (setpoint)

    unsigned long m_lastTimeA, m_lastTimeB;
    double m_errorSumA, m_errorSumB;
    double m_lastErrorA, m_lastErrorB;

    // Define some enums to make specifying motor and direction easier
    typedef enum {CW,CCW} m_Direction; // CW = 0, CCW = 1
    typedef enum {A,B} m_Motor; // A = 0, B = 1
    
    // Global variables for motor direction so they can be read by encoder ISR to determine direction
    static m_Direction m_dirA;
    static m_Direction m_dirB;

    // Speed calculation variables
    const double m_averagePWMsignalA = 140;
    const double m_averageSpeedACMS = 8.356667;

    const double m_averagePWMsignalB = 140;
    const double m_averageSpeedBCMS = 8.042222;
    
    m_Direction m_prevDirA = CW;
    m_Direction m_prevDirB = CCW;
    
    // To check if the value from the direction registers has changed
    m_Direction m_registerDirAprev = CW;
    m_Direction m_registerDirBprev = CCW;
        
    // These values will not be reset unless a signal is received
    volatile static int m_absoluteStepsA;
    volatile static int m_absoluteStepsB;
    
    // Variables to store distance count from motors
    double m_distanceA = 0;
    double m_distanceB = 0;
    
    int m_speedAPWM_prev = 0;
    int m_speedBPWM_prev = 0;
    
    volatile int m_speedValuePrev = 0;
    volatile int m_inputPrev = 0;

    // MEMBER METHODS

    // Initialisation methods
    void m_setPinModes(); // Sets pins and pin modes of all components
    void m_initializeComponents(); // Initializes values
    void m_attachInterrupts(); // Attaches interrupts
    void m_initializeSerialRegisters(); // Initialize serial registers

    // PID functions
    void m_readControlModeRegister();
    void m_readPidTunningSettings();
    void m_runPidControl();
    void m_convertStepsToCM();
    void m_computePID(m_Motor t_motor);
    void m_motorSetSpeedCM(m_Motor t_motor, double t_speed);
    void m_motorSetDir(m_Motor t_motor, m_Direction t_dir);
  
    // Encoders
    void m_setAbsoluteEncoderSteps();
    double m_stepsToCentimetres(int t_steps);
    void m_sendEncoderStepsToRegisters(int t_register1, int t_register2, int t_steps);
    void m_sendDecimalToRegisters(int t_register1, int t_register2, float t_number);
    double m_readDecimalNumberFromRegisters(int t_register1, int t_register2);
    double m_read16BitNumber(int t_register1, int t_register2);
    void m_resetEncoders();

    // Wheels
    void m_readWheelDirections();
    void m_readPWMSignals();
  
    // General movement
    void m_readSpeedLevelValue();
    void m_readDirectionInput();
    void m_adjustSpeed(int t_speedLevel);

    // Direction control
    void m_moveForward();
    void m_moveBackward();
    void m_moveLeft();
    void m_moveRight();
    void m_stopRobot();
    
    // Encoder interrupts
    static void m_ENCA_ISR();
    static void m_ENCB_ISR();
};

#endif
