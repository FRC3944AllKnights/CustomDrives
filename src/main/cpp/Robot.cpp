#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include "ArcadeVelocityControl.h"
#include "Elevator.h"

class Robot : public frc::TimedRobot {
 public:
  void TeleopInit() override {
    arcadeVelocity.DriveInit();
  }

  void TeleopPeriodic() override {
    arcadeVelocity.Drive(m_stick.GetTwist(), m_stick.GetY());
    ElevatorSus.ElevatorBalls(m_stick.GetRawButton(3), m_stick.GetRawButton(4));
  }



 private:
  frc::Joystick m_stick{0};
  ArcadeVelocityControl arcadeVelocity;

  Elevator ElevatorSus;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif