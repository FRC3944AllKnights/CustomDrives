#include "Elevator.h"

Elevator::Elevator(){}



void Elevator::ElevatorBalls(bool ButtThree, bool ButtFour){

    if (ButtThree){
        Elevator_motor.Set(0.5);
    }
    else if (ButtFour){
        Elevator_motor.Set(-0.5);
    }
    else(){
        Elevator_motor.Set(0);
    }
}


