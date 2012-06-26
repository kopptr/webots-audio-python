/*******************************************************************************************************/
/* File:         Robot.hpp                                                                             */
/* Date:         May 08                                                                                */
/* Description:  Header file for the OO interface                                                      */
/* Author:       fabien.rohrer@cyberbotics.com                                                         */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                                                */
/*******************************************************************************************************/

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/Connector.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Display.hpp>
#include <webots/Device.hpp>
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Pen.hpp>
#include <webots/Receiver.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Servo.hpp>
#include <map>

namespace webots {
  class Robot {
    public:
      Robot();
      virtual ~Robot();
      virtual int step(int ms);
      std::string getName() const;
      double getTime() const;
      int getMode() const;
      int getLicense() const;
      bool getSynchronization() const;
      std::string getProjectPath() const;
      double getBasicTimeStep() const;
      virtual void batterySensorEnable(int ms);
      virtual void batterySensorDisable();
      virtual double batterySensorGetValue();
      virtual void keyboardEnable(int ms);
      virtual void keyboardDisable();
      virtual int keyboardGetKey();
      enum {
        KEYBOARD_END=312,
        KEYBOARD_HOME,
        KEYBOARD_LEFT,
        KEYBOARD_UP,
        KEYBOARD_RIGHT,
        KEYBOARD_DOWN,
        KEYBOARD_PAGEUP=366,
        KEYBOARD_PAGEDOWN,
        KEYBOARD_NUMPAD_HOME=375,
        KEYBOARD_NUMPAD_LEFT,
        KEYBOARD_NUMPAD_UP,
        KEYBOARD_NUMPAD_RIGHT,
        KEYBOARD_NUMPAD_DOWN,
        KEYBOARD_NUMPAD_END=382,
        KEYBOARD_KEY=0x0000ffff,
        KEYBOARD_SHIFT=0x00010000,
        KEYBOARD_CONTROL=0x00020000,
        KEYBOARD_ALT=0x00040000
      };
      Accelerometer *getAccelerometer(const std::string &name);
      Camera *getCamera(const std::string &name);
      Compass *getCompass(const std::string &name);
      Connector *getConnector(const std::string &name);
      Display *getDisplay(const std::string &name);
      DistanceSensor *getDistanceSensor(const std::string &name);
      Emitter *getEmitter(const std::string &name);
      GPS *getGPS(const std::string &name);
      Gyro *getGyro(const std::string &name);
      LED *getLED(const std::string &name);
      LightSensor *getLightSensor(const std::string &name);
      Pen *getPen(const std::string &name);
      Receiver *getReceiver(const std::string &name);
      Servo *getServo(const std::string &name);
      TouchSensor *getTouchSensor(const std::string &name);

    protected:
      virtual Accelerometer *createAccelerometer(const std::string &name) const;
      virtual Camera *createCamera(const std::string &name) const;
      virtual Compass *createCompass(const std::string &name) const;
      virtual Connector *createConnector(const std::string &name) const;
      virtual Display *createDisplay(const std::string &name) const;
      virtual DistanceSensor *createDistanceSensor(const std::string &name) const;
      virtual Emitter *createEmitter(const std::string &name) const;
      virtual GPS *createGPS(const std::string &name) const;
      virtual Gyro *createGyro(const std::string &name) const;
      virtual LED *createLED(const std::string &name) const;
      virtual LightSensor *createLightSensor(const std::string &name) const;
      virtual Pen *createPen(const std::string &name) const;
      virtual Receiver *createReceiver(const std::string &name) const;
      virtual Servo *createServo(const std::string &name) const;
      virtual TouchSensor *createTouchSensor(const std::string &name) const;

    private:
      Device *getDevice(const std::string &name);
      void insertDevice(Device *device);
      std::map<const std::string,Device*> deviceList;
  };
}

#endif //ROBOT_HPP
