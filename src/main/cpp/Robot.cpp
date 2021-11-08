#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/TimedRobot.h>
#include <cmath>

class Robot : public frc::TimedRobot {
 public:
  void TeleopInit() override {
    Back_Right.RestoreFactoryDefaults();
    Back_Right_PID.SetP(kP);
    Back_Right_PID.SetI(kI);
    Back_Right_PID.SetD(kD);
    Back_Right_PID.SetIZone(kIz);
    Back_Right_PID.SetFF(kFF);
    Back_Right_PID.SetOutputRange(kMinOutput, kMaxOutput);

    Back_Left.RestoreFactoryDefaults();
    Back_Left_PID.SetP(kP);
    Back_Left_PID.SetI(kI);
    Back_Left_PID.SetD(kD);
    Back_Left_PID.SetIZone(kIz);
    Back_Left_PID.SetFF(kFF);
    Back_Left_PID.SetOutputRange(kMinOutput, kMaxOutput);

    Front_Left.RestoreFactoryDefaults();
    Front_Left_PID.SetP(kP);
    Front_Left_PID.SetI(kI);
    Front_Left_PID.SetD(kD);
    Front_Left_PID.SetIZone(kIz);
    Front_Left_PID.SetFF(kFF);
    Front_Left_PID.SetOutputRange(kMinOutput, kMaxOutput);

    Front_Right.RestoreFactoryDefaults();
    Front_Right_PID.SetP(kP);
    Front_Right_PID.SetI(kI);
    Front_Right_PID.SetD(kD);
    Front_Right_PID.SetIZone(kIz);
    Front_Right_PID.SetFF(kFF);
    Front_Right_PID.SetOutputRange(kMinOutput, kMaxOutput);
  }

  void TeleopPeriodic() override {

    double twist = deadbandremover(m_stick.GetTwist()*2);
    double y = deadbandremover(m_stick.GetY()*2);

    double vTwist = twist/wheelCircumference*gearRatio*60;
    double vY = y/wheelCircumference*gearRatio*60;

    Back_Right_PID.SetReference((vY + vTwist), rev::ControlType::kVelocity);
    Front_Right_PID.SetReference((vY + vTwist), rev::ControlType::kVelocity);

    Back_Left_PID.SetReference(-(vY - vTwist), rev::ControlType::kVelocity);
    Front_Left_PID.SetReference(-(vY - vTwist), rev::ControlType::kVelocity);
  }

 private:
  frc::Joystick m_stick{0};
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

  //gear ratio
  double gearRatio = 10.75;
  double wheelCircumference = 0.479;

  //define the PID controller and encoder for the back right motor
  rev::CANPIDController Back_Right_PID = Back_Right.GetPIDController();
  rev::CANEncoder Back_Right_encoder = Back_Right.GetEncoder();

  rev::CANPIDController Back_Left_PID = Back_Left.GetPIDController();
  rev::CANEncoder Back_Left_encoder = Back_Left.GetEncoder();

  rev::CANPIDController Front_Right_PID = Front_Right.GetPIDController();
  rev::CANEncoder Front_Right_encoder = Front_Right.GetEncoder();

  rev::CANPIDController Front_Left_PID = Front_Left.GetPIDController();
  rev::CANEncoder Front_Left_encoder = Front_Left.GetEncoder();

  double deadbandremover(double value){
    double x;
    if(value > -0.1 and value < 0.1){
      value = 0.0;
    }

    if(value < 0){
      x = -1;
    }
    else{
      x = 1;
    }

    value = pow(value, 2.0);
    return value*x;
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif