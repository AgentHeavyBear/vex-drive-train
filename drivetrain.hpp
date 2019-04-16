#include "main.h"
#include <vector>
#include <string>

namespace drivetrain {
  class drivetrain {
    private:
      std::vector<pros::Motor*> leftMotors;
      std::vector<pros::Motor*> rightMotors;
      std::vector<pros::Motor*> allMotors;

    public:

      //default constructor
      drivetrain() {}

      //drive train constructor accepts vectors of pointers to drive motors, left and right seperately
      drivetrain(std::vector<pros::Motor*> left, std::vector<pros::Motor*> right) {
        leftMotors = left;
        rightMotors = right;
        allMotors.insert( allMotors.end(), leftMotors.begin(), leftMotors.end() );
        allMotors.insert( allMotors.end(), rightMotors.begin(), rightMotors.end() );
      }

      //adds a motor to the left or right dt, for use if parametrized constructor was not used
      void add_left_motor(pros::Motor *motor) {
        leftMotors.push_back(motor);
        allMotors.push_back(motor);
      }
      void add_right_motor(pros::Motor *motor) {
        rightMotors.push_back(motor);
        allMotors.push_back(motor);
      }

      //===============================================//
      //==============MOVEMENT FUNCTIONS===============//
      //==============================================//

      // sets drive motors to drive at speed (127 to -127), all by default, pass right or left to only set those motors
      void drive(int speed, std::string which = "") {
        if (which == "RIGHT") {
          for ( unsigned int i = 0; i < rightMotors.size(); i++ ) {
            rightMotors[i]->move(speed);
          }
        }

        else if (which == "LEFT") {
          for ( unsigned int i = 0; i < leftMotors.size(); i++) {
            leftMotors[i]->move(speed);
          }
        }

        else {
          for ( unsigned int i = 0; i < allMotors.size(); i++ ) {
            allMotors[i]->move(speed);
          }
        }
      }

      // sets drive motors to voltage, all by default, pass right or left to only set those motors
      void drive_voltage(int volt, std::string which = "") {
          if (which == "RIGHT") {
            for ( unsigned int i = 0; i < rightMotors.size(); i++ ) {
              rightMotors[i]->move_voltage(volt);
            }
          }

          else if (which == "LEFT") {
            for ( unsigned int i = 0; i < leftMotors.size(); i++) {
              leftMotors[i]->move_voltage(volt);
            }
          }

          else {
            for ( unsigned int i = 0; i < allMotors.size(); i++ ) {
              allMotors[i]->move_voltage(volt);
            }
          }
      }

      // sets drive motors to velocity, all by default, pass right or left to only set those motors
      void drive_velocity(int vel, std::string which = "") {
        if (which == "RIGHT") {
          for ( unsigned int i = 0; i < rightMotors.size(); i++ ) {
            rightMotors[i]->move_voltage(vel);
          }
        }

        else if (which == "LEFT") {
          for ( unsigned int i = 0; i < leftMotors.size(); i++) {
            leftMotors[i]->move_voltage(vel);
          }
        }

        else {
          for ( unsigned int i = 0; i  < allMotors.size(); i++ ) {
            allMotors[i]->move_voltage(vel);
          }
        }
      }

      // sets robot driving for distance at velocity, pass right or left to only set those motors
      void drive_relative(double distance, int vel, std::string which = "") {
        if (which == "RIGHT") {
          for (unsigned int i = 0; i < rightMotors.size(); i++) {
            rightMotors[i]->move_relative(distance, vel);
          }
        }

        else if (which == "LEFT") {
          for (unsigned int i = 0; i < leftMotors.size(); i++) {
            leftMotors[i]->move_relative(distance, vel);
          }
        }

        else {
          for (unsigned int i = 0; i < allMotors.size(); i++) {
            allMotors[i]->move_relative(distance, vel);
          }
        }
      }

      // sets robot to turn using move function(127 to -127 scale), positive turns left
      void turn(int speed) {
        this->drive(-speed, "LEFT");
        this->drive(speed, "RIGHT");
      }
      // sets robot to turn at voltage, positive voltage turns left
      void turn_voltage(int volt) {
        this->drive_voltage(-volt, "LEFT");
        this->drive_voltage(volt, "RIGHT");
      }

      // sets robot to turn at velocity, positive turns left
      void turn_velocity(int vel) {
        this->drive_velocity(-vel, "LEFT");
        this->drive_velocity(vel, "RIGHT");
      }


      //===============================================//
      //=============TELEMETRY FUNCTIONS===============//
      //==============================================//




      //gets average value of selected motor encoders, ALL, RIGHT, or LEFT; defaults to ALL
      double get_position(std::string which = "") {
        double sum = 0;
        if (which == "RIGHT") {
          for (unsigned int i = 0; i < rightMotors.size(); i++) {
            sum += rightMotors[i]->get_position();
          }
          sum /= (double)rightMotors.size();
          return sum;
        }

        else if (which == "LEFT") {
          for (unsigned int i = 0; i < leftMotors.size(); i++) {
            sum += leftMotors[i]->get_position();
          }
          sum /= leftMotors.size();
          return sum;
        }

        else {
          for (unsigned int i = 0; i < allMotors.size(); i++) {
            sum += allMotors[i]->get_position();
          }
          sum /= allMotors.size();
          return sum;
        }
      }

      //sets the zero point of the selected motors to the current position, ALL, RIGHT, or LEFT; defaults to ALL
      void tare_position(std::string which = "") {
        if (which == "RIGHT") {
          for (unsigned int i = 0; i < rightMotors.size(); i++) {
            rightMotors[i]->tare_position();
          }
        }

        else if (which == "LEFT") {
          for (unsigned int i = 0; i < leftMotors.size(); i++) {
            leftMotors[i]->tare_position();
          }
        }

        else {
          for (unsigned int i = 0; i < allMotors.size(); i++) {
            allMotors[i]->tare_position();
          }
        }
      }

      // gets average velocity of selected motors, ALL, RIGHT, or LEFT; defaults to ALL
      double get_velocity(std::string which = "") {
        double sum = 0;
        if (which == "RIGHT") {
          for (unsigned int i = 0; i < rightMotors.size(); i++) {
            sum += rightMotors[i]->get_actual_velocity();
          }
          sum /= (double)rightMotors.size();
          return sum;
        }

        else if (which == "LEFT") {
          for (unsigned int i = 0; i < leftMotors.size(); i++) {
            sum += leftMotors[i]->get_actual_velocity();
          }
          sum /= leftMotors.size();
          return sum;
        }

        else {
          for (unsigned int i = 0; i < allMotors.size(); i++) {
            sum += allMotors[i]->get_actual_velocity();
          }
          sum /= allMotors.size();
          return sum;
        }
      }

      //returns 1 if all of selected motors are stopped and 0 otherwise, ALL, RIGHT, or LEFT; defaults to ALL
      std::int32_t is_stopped(std::string which = "") {
        if (which == "RIGHT") {
          for (unsigned int i = 0; i < rightMotors.size(); i++) {
            if (rightMotors[i]->is_stopped() == 0) {return 0;}
          }
          return 1;
        }

        else if (which == "LEFT") {
          for (unsigned int i = 0; i < leftMotors.size(); i++) {
            if (leftMotors[i]->is_stopped() == 0) {return 0;}
          }
          return 1;
        }

        else {
          for (unsigned int i = 0; i < allMotors.size(); i++) {
            if (allMotors[i]->is_stopped() == 0) {return 0;}
          }
          return 1;
        }
      }

      //returns 1 if selected motors are moving in positive, -1 if moving in negative, and returns 0 if motors are moving in conflicting directions,  ALL, RIGHT, or LEFT; defaults to ALL
      std::int32_t get_direction(std::string which = "") {
        std::int32_t dir;
        if (which == "RIGHT") {
          dir = rightMotors[0]->get_direction();
          for (unsigned int i = 0; i < rightMotors.size(); i++) {
            if (rightMotors[i]->get_direction() != dir) {return 0;}
          }
          return dir;
        }

        else if (which == "LEFT") {
          dir = leftMotors[0]->get_direction();
          for (unsigned int i = 0; i < leftMotors.size(); i++) {
            if (leftMotors[i]->get_direction() != dir) {return 0;}
          }
          return dir;
        }

        else {
          dir = allMotors[0]->get_direction();
          for (unsigned int i = 0; i < allMotors.size(); i++) {
            if (allMotors[i]->get_direction() != dir) {return 0;}
          }
          return dir;
        }
      }

      double get_temperature(std::string which = "") {
        double sum = 0;
        if (which=="LEFT") {
          for (unsigned int i = 0; i < leftMotors.size(); i++) {
            sum += leftMotors[i]->get_temperature();
            sum /= leftMotors.size();
          }
        }

        else if (which=="RIGHT") {
          for (unsigned int i = 0; i < rightMotors.size(); i++) {
            sum += rightMotors[i]->get_temperature();
            sum /= rightMotors.size();
          }
        }

        else {
          for (unsigned int i = 0; i < allMotors.size(); i++) {
            sum += allMotors[i]->get_temperature();
            sum /= allMotors.size();
          }
        }
        return sum;
      }


  }; // class drivetrain
} // namespace drivetrain
