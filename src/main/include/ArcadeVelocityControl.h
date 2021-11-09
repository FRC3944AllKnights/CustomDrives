#ifndef ARCADEVELOCITYCONTROL_H
#define ARCADEVELOCITYCONTROL_H
#include <cmath>
#include <rev/CANSparkMax.h>

class ArcadeVelocityControl{
    public:
        ArcadeVelocityControl();
        void Drive(double Twist, double Y);
        void DriveInit();

    private:
        double deadbandremover(double value);

        //gear ratio
        double gearRatio = 10.75;
        double wheelCircumference = 0.479;

        rev::CANSparkMax Back_Right{1, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax Back_Left{2, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax Front_Right{3, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax Front_Left{4, rev::CANSparkMax::MotorType::kBrushless};

        //controller gain variables
        double kP = 9e-5;
        double kI = 1e-7;
        double kD = 0;
        double kIz = 0; 
        double kFF = 0.000015;

        //output parameters
        double kMaxOutput = 1.0; 
        double kMinOutput = -1.0;
        const double MaxRPM = 5700;

        //define the PID controller and encoder for the back right motor
        rev::CANPIDController Back_Right_PID = Back_Right.GetPIDController();
        rev::CANEncoder Back_Right_encoder = Back_Right.GetEncoder();

        rev::CANPIDController Back_Left_PID = Back_Left.GetPIDController();
        rev::CANEncoder Back_Left_encoder = Back_Left.GetEncoder();

        rev::CANPIDController Front_Right_PID = Front_Right.GetPIDController();
        rev::CANEncoder Front_Right_encoder = Front_Right.GetEncoder();

        rev::CANPIDController Front_Left_PID = Front_Left.GetPIDController();
        rev::CANEncoder Front_Left_encoder = Front_Left.GetEncoder();
};

#endif