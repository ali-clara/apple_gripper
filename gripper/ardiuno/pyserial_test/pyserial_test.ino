/*
2024-03-17
Ali Jones

Demonstrates that pyserial works to control different function calls
*/

#include <string.h>

String input;

void setup(){
    Serial.begin(9600);
    while (!Serial) delay(10);
}

void loop(){
    input = Serial.readStringUntil("#");
    
    if (input == "vacuum:on"){
        gripperVacuumOn();
    }
    else if (input == "vacuum:off"){
        gripperVacuumOff();
    }
    else if (input == "fingers:on"){
        gripperFingersOn();
    }
    else if (input == "fingers:off"){
        gripperFingersOff();
    }

}

void gripperVacuumOn(void){
    Serial.println("Turning gripper vacuum on");
}

void gripperVacuumOff(void){
    Serial.println("Turning gripper vacuum off");
}

void gripperFingersOn(void){
    Serial.println("Engaging fingers");
}

void gripperFingersOff(void){
    Serial.println("Disengaging fingers");
}