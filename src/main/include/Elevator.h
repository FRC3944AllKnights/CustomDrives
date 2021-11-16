#ifndef ELEVATOR_H
#define ELEVATOR_H
#include <cmath>
#include <rev/CANSparkMax.h>

class Elevator{
    public:

        ArcadeVelocityControl();
        void Elevator(double Twist, double Y);
        void DriveInit();

};


#endif