#include "BallIntake.h"

BallIntake::BallIntake(){

}

void BallIntake::IntakeVelocityControlInit(){
    Intake.RestoreFactoryDefaults();
    Intake_PID.SetP(kP);
    Intake_PID.SetI(kI);
    Intake_PID.SetD(kD);
    Intake_PID.SetIZone(kIz);
    Intake_PID.SetFF(kFF);
    Intake_PID.SetOutputRange(kMinOutput, kMaxOutput);


}
//fix everything bellow
void BallIntake::IntakeVelocityControl(bool in, bool out){
    double twist = deadbandremover(Twist*2);
    double y = deadbandremover(Y*2);

    double vTwist = twist/wheelCircumference*gearRatio*60;
    double vY = y/wheelCircumference*gearRatio*60;

    Back_Right_PID.SetReference((vY + vTwist), rev::ControlType::kVelocity);
    Front_Right_PID.SetReference((vY + vTwist), rev::ControlType::kVelocity);

    Back_Left_PID.SetReference(-(vY - vTwist), rev::ControlType::kVelocity);
    Front_Left_PID.SetReference(-(vY - vTwist), rev::ControlType::kVelocity);
}