#include <RobotController.h>
#include <iostream>

RobotController::RobotController(std::string name){
  this->robotname = name;
  this->velocity = 0;
  std::cout<<"Robot name: "<<robotname<<"endl";
}
void RobotController::setVelo(double vel){
    this->velocity=vel;
}
void RobotController::stop(){
    this->velocity=0;
}
double RobotController::getVelo(){
    return this->velocity;
}
