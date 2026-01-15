#ifndef ROBOTCONTROLLER_H // dung de bao ve nhieu lan include
#define ROBOTCONTROLLER_H

#include <iostream> // day la trong c++
#include <string>
#include <vector>
#include <cmath>

class RobotController {
  private:
    std::string robotname;
    double velocity;
  public:
    //Constructor
    RobotController(std::string name);
    
    //Method
    void setVelo (double v);
    double getVelo ();
    void stop();
};
#endif
