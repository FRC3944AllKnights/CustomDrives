#ifndef ELEVATOR_H
#define ELEVATOR_H
#include <cmath>
#include <rev/CANSparkMax.h>

class Elevator{
    public:
        
        Elevator();
        void ElevatorBalls(bool ButtThree, bool ButtFour);

    private:

        rev::CANSparkMax Elevator_motor{5, rev::CANSparkMax::MotorType::kBrushless};

};


#endif