#ifndef BALLINTAKE_H
#define BALLINTAKE_H
#include <cmath>
#include <rev/CANSparkMax.h>

class BallIntake{
    public:
        BallIntake();
        void IntakeVelocityControl(bool in, bool out);
        void IntakeVelocityControlInit();

    private:

        //gear ratio
        double gearRatio = 10;

        rev::CANSparkMax Intake{10, rev::CANSparkMax::MotorType::kBrushless};

        //controller gain variables
        double kP = 9e-5;
        double kI = 1e-7;
        double kD = 0;
        double kIz = 0; 
        double kFF = 0.000015;

        //output parameters
        double kMaxOutput = 1.0; 
        double kMinOutput = -1.0;
        const double MaxRPM = 100;

        //define the PID controller and encoder for the back right motor
        rev::CANPIDController Intake_PID = Intake.GetPIDController();
        rev::CANEncoder Intake_encoder = Intake.GetEncoder();

};

#endif