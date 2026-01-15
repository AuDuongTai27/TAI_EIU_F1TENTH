#include <RobotController.h>
#include <iostream>
#include <string>

using namespace std;

int main(){
  RobotController robot = RobotController("Nguyen Van A");
  robot.setVelo(2);
  cout<<"Velocity :"<<robot.getVelo(); // thêm () đối với bất kỳ method hay constructors nào trong một class
  return 0;
}