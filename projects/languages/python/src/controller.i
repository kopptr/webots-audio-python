/*******************************************************************************************************/
/* File:         controller.i                                                                          */
/* Date:         May 08                                                                                */
/* Description:  Swig interface which maps the OO C++ files into a python module called "controller"   */
/* Author:       fabien.rohrer@cyberbotics.com                                                         */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                                                */
/*******************************************************************************************************/

%module controller

%{
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/Connector.hpp>
#include <webots/Device.hpp>
#include <webots/DifferentialWheels.hpp>
#include <webots/Display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/ImageRef.hpp>
#include <webots/LED.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Microphone.hpp>
#include <webots/utils/Motion.hpp>
#include <webots/Node.hpp>
#include <webots/Pen.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/Servo.hpp>
#include <webots/Speaker.hpp>
#include <webots/Supervisor.hpp>
#include <webots/TouchSensor.hpp>

using namespace std;
%}

//----------------------------------------------------------------------------------------------
//  Miscellaneous - controller module's level
//----------------------------------------------------------------------------------------------

//handling std::string
%include "std_string.i"

// manage double arrays
%typemap(out) const double * {
  int len,i;
  string test("$name");
  if (test == "getSFVec2f" || test == "getMFVec2f")
    len = 2;
  else if (test == "getSFRotation")
    len = 4;
  else if (test == "getOrientation")
    len = 9;
  else
    len = 3;
  $result = PyList_New(len);
  for (i = 0; i < len; i++) {
    PyList_SetItem($result,i,PyFloat_FromDouble($1[i]));
  }
}
%typemap(in) const double [ANY] {
  if (!PyList_Check($input)) {
    PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList'\n");
    return NULL;
  }
  int len = PyList_Size($input);
  $1 = (double*)malloc(len*sizeof(double));
  for (int i=0;i<len;i++){
    $1[i]=PyFloat_AsDouble(PyList_GetItem($input,i));
  }
}
%typemap(in) const int * {
  if (!PyList_Check($input)) {
    PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList'\n");
    return NULL;
  }
  int len = PyList_Size($input);
  $1 = (int*)malloc(len*sizeof(int));
  for (int i=0;i<len;i++){
    $1[i]=PyInt_AsLong(PyList_GetItem($input,i));
  }
}

//----------------------------------------------------------------------------------------------
//  Device
//----------------------------------------------------------------------------------------------

%include <webots/Device.hpp>

//----------------------------------------------------------------------------------------------
//  Accelerometer
//----------------------------------------------------------------------------------------------

%include <webots/Accelerometer.hpp>

//----------------------------------------------------------------------------------------------
//  Camera
//----------------------------------------------------------------------------------------------

%typemap(out) unsigned char * {
  int width=arg1->getWidth();
  int height=arg1->getHeight();
  $result=PyString_FromStringAndSize((const char*)$1,3*width*height);
}

%typemap(out) float * {
  int width=arg1->getWidth();
  int height=arg1->getHeight();
  int len=width*height;
  $result = PyList_New(len);
  for (int x = 0; x < len; x++) {
    PyList_SetItem($result,x,PyFloat_FromDouble($1[x]));
  }
}

%extend webots::Camera {
  PyObject* getImageArray(){
    const unsigned char* im = $self->getImage();
    int width = $self->getWidth();
    int height = $self->getHeight();
    int x,y,ch;
    PyObject* ret = PyList_New(width);
    for (x = 0; x < width; x++) {
      PyObject* dim2 = PyList_New(height);
      PyList_SetItem(ret,x,dim2);
      for (y = 0; y < height; y++) {
        PyObject* dim3 = PyList_New(3);
        PyList_SetItem(dim2,y,dim3);
        for (ch = 0; ch < 3; ch++) {
          PyList_SetItem(dim3,ch,PyInt_FromLong((unsigned int)(im[3*(x+y*width)+ch])));
        }
      }
    }
    return ret;
  }

  PyObject* getRangeImageArray(){
    const float* im = $self->getRangeImage();
    int width = $self->getWidth();
    int height = $self->getHeight();
    int x,y;
    PyObject* ret = PyList_New(width);
    for (x = 0; x < width; x++) {
      PyObject* dim2 = PyList_New(height);
      PyList_SetItem(ret,x,dim2);
      for (y = 0; y < height; y++) {
        PyObject* v = PyFloat_FromDouble(im[x+y*width]);
        PyList_SetItem(dim2,y,v);
      }
    }
    return ret;
  }

  static PyObject* rangeImageGetValue(PyObject *im, double near, double far, int width, int x, int y){
    if(!PyList_Check(im)){
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList'\n");
      return NULL;
    }
    PyObject* value=PyList_GetItem(im,y*width+x);
    if(!PyFloat_Check(value)){
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList' of 'PyFloat'\n");
      return NULL;
    }
    fprintf(stderr,"Warning: Camera.rangeImageGetValue is deprecated, please use Camera.rangeImageGetDepth instead\n");
    return value;
  }

  static PyObject* rangeImageGetDepth(PyObject *im, int width, int x, int y){
    if(!PyList_Check(im)){
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList'\n");
      return NULL;
    }
    PyObject* value=PyList_GetItem(im,y*width+x);
    if(!PyFloat_Check(value)){
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList' of 'PyFloat'\n");
      return NULL;
    }
    return value;
  }

  static PyObject* imageGetRed(PyObject *im, int width, int x, int y){
    if(!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetRed', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char* s=(unsigned char*) PyString_AsString(im);
    return PyInt_FromLong(s[3*(y*width+x)]);
  }

  static PyObject* imageGetGreen(PyObject *im, int width, int x, int y){
    if(!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetGreen', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char* s=(unsigned char*) PyString_AsString(im);
    return PyInt_FromLong(s[3*(y*width+x)+1]);
  }

  static PyObject* imageGetBlue(PyObject *im, int width, int x, int y){
    if(!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetBlue', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char* s=(unsigned char*) PyString_AsString(im);
    return PyInt_FromLong(s[3*(y*width+x)+2]);
  }

  static PyObject* imageGetGrey(PyObject *im, int width, int x, int y){
    if(!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetGrey', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char* s=(unsigned char*) PyString_AsString(im);
    return PyInt_FromLong((s[3*(y*width+x)]+s[3*(y*width+x)+1]+s[3*(y*width+x)+2])/3);
  }
};

%include <webots/Camera.hpp>

%typemap(out) unsigned char *;
%typemap(out) float *;

//----------------------------------------------------------------------------------------------
//  Compass
//----------------------------------------------------------------------------------------------

%include <webots/Compass.hpp>

//----------------------------------------------------------------------------------------------
//  Connector
//----------------------------------------------------------------------------------------------

%include <webots/Connector.hpp>

//----------------------------------------------------------------------------------------------
//  Display
//----------------------------------------------------------------------------------------------

%typemap(in) const void * {
  if (!PyList_Check($input) && !PyString_Check($input)) {
    PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList' or 'PyString'\n");
    return NULL;
  }
  if (PyList_Check($input)) {
    int len1 = PyList_Size($input);
    PyObject *l2=PyList_GetItem($input,0);
    if (!PyList_Check(l2)) {
      PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList' of 'PyList'\n");
      return NULL;
    }
    int len2 = PyList_Size(l2);
    PyObject *l3=PyList_GetItem(l2,0);
    if (!PyList_Check(l3)) {
      PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList' of 'PyList' of 'PyList'\n");
      return NULL;
    }
    int len3 = PyList_Size(l3);
    $1 = (void *)malloc(len1*len2*len3*sizeof(unsigned char));
    for (int i=0;i<len1;i++) for(int j=0;j<len2;j++) for(int k=0;k<len3;k++) {
      ((unsigned char *)$1)[(i*len2*len3)+(j*len3)+k]=(unsigned char)
       PyInt_AsLong(PyList_GetItem(PyList_GetItem(PyList_GetItem($input,i),j),k));
    }
  } else { // PyString case
    $1=PyString_AsString($input);
  }
}

%rename (internalImageNew) imageNew(int width,int height,const void *data,int format) const;
%rename (internalDrawPolygon) drawPolygon(const int *x,const int *y,int size);
%rename (internalFillPolygon) fillPolygon(const int *x,const int *y,int size);

%extend webots::Display {
  %pythoncode %{
    def imageNew(self,data,format):
      return self.internalImageNew(len(data),len(data[0]),data,format)
    def drawPolygon(self, x ,y):
      self.internalDrawPolygon(x,y,min(len(x),len(y)))
    def fillPolygon(self, x ,y):
      self.internalFillPolygon(x,y,min(len(x),len(y)))
  %}
}

%include <webots/ImageRef.hpp>
%include <webots/Display.hpp>

//----------------------------------------------------------------------------------------------
//  Distance sensor
//----------------------------------------------------------------------------------------------

%include <webots/DistanceSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Emitter
//----------------------------------------------------------------------------------------------

%typemap(in) (const void *data, int size) {
  $1 = PyString_AsString($input);
  $2 = PyString_Size($input);
}

%include <webots/Emitter.hpp>

%typemap(in) (const void *data, int size);

//----------------------------------------------------------------------------------------------
//  Field
//----------------------------------------------------------------------------------------------

%ignore webots::Field::findField(WbFieldRef ref);
%ignore webots::Field::cleanup();

%include <webots/Field.hpp>

//----------------------------------------------------------------------------------------------
//  GPS
//----------------------------------------------------------------------------------------------

%include <webots/GPS.hpp>

//----------------------------------------------------------------------------------------------
//  Gyro
//----------------------------------------------------------------------------------------------

%include <webots/Gyro.hpp>

//----------------------------------------------------------------------------------------------
//  LED
//----------------------------------------------------------------------------------------------

%include <webots/LED.hpp>

//----------------------------------------------------------------------------------------------
//  LightSensor
//----------------------------------------------------------------------------------------------

%include <webots/LightSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Microphone
//----------------------------------------------------------------------------------------------

%typemap(out) const void * {
  $result = PyString_FromStringAndSize((const char*) $1,arg1->getDataSize());
}

%include <webots/Microphone.hpp>

%typemap(out) const void *;

//----------------------------------------------------------------------------------------------
//  Motion
//----------------------------------------------------------------------------------------------

%include <webots/utils/Motion.hpp>

//----------------------------------------------------------------------------------------------
//  Node
//----------------------------------------------------------------------------------------------

%ignore webots::Node::findNode(WbNodeRef ref);
%ignore webots::Node::cleanup();

%include <webots/Node.hpp>

//----------------------------------------------------------------------------------------------
//  Pen
//----------------------------------------------------------------------------------------------

%include <webots/Pen.hpp>

//----------------------------------------------------------------------------------------------
//  Receiver
//----------------------------------------------------------------------------------------------

%typemap(out) const void * {
  $result = PyString_FromStringAndSize((const char*) $1,arg1->getDataSize());
}

%include <webots/Receiver.hpp>

%typemap(out) const void *;

//----------------------------------------------------------------------------------------------
//  Servo
//----------------------------------------------------------------------------------------------

%include <webots/Servo.hpp>

//----------------------------------------------------------------------------------------------
//  Speaker
//----------------------------------------------------------------------------------------------

%typemap(in) (const void *data, int size) {
  $1 = PyString_AsString($input);
  $2 = PyString_Size($input);
}

%include <webots/Speaker.hpp>

%typemap(in) (const void *data, int size);

//----------------------------------------------------------------------------------------------
//  TouchSensor
//----------------------------------------------------------------------------------------------

%include <webots/TouchSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Robot
//----------------------------------------------------------------------------------------------

%ignore webots::Robot::getAccelerometer(const std::string &name);
%ignore webots::Robot::getCamera(const std::string &name);
%ignore webots::Robot::getCompass(const std::string &name);
%ignore webots::Robot::getConnector(const std::string &name);
%ignore webots::Robot::getDisplay(const std::string &name);
%ignore webots::Robot::getDistanceSensor(const std::string &name);
%ignore webots::Robot::getEmitter(const std::string &name);
%ignore webots::Robot::getGPS(const std::string &name);
%ignore webots::Robot::getGyro(const std::string &name);
%ignore webots::Robot::getLED(const std::string &name);
%ignore webots::Robot::getLightSensor(const std::string &name);
%ignore webots::Robot::getMicrophone(const std::string &name);
%ignore webots::Robot::getPen(const std::string &name);
%ignore webots::Robot::getReceiver(const std::string &name);
%ignore webots::Robot::getServo(const std::string &name);
%ignore webots::Robot::getSpeaker(const std::string &name);
%ignore webots::Robot::getTouchSensor(const std::string &name);

%extend webots::Robot {
  %pythoncode %{
    devices = {}
    def createAccelerometer(self, name):
      return Accelerometer(name)
    def getAccelerometer(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      accelerometer = self.createAccelerometer(name)
      self.devices[name] = accelerometer
      return accelerometer
    def createCamera(self, name):
      return Camera(name)
    def getCamera(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      camera = self.createCamera(name)
      self.devices[name] = camera
      return camera
    def createCompass(self, name):
      return Compass(name)
    def getCompass(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      compass = self.createCompass(name)
      self.devices[name] = compass
      return compass
    def createConnector(self, name):
      return Connector(name)
    def getConnector(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      connector = self.createConnector(name)
      self.devices[name] = connector
      return connector
    def createDisplay(self, name):
      return Display(name)
    def getDisplay(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      display = self.createDisplay(name)
      self.devices[name] = display
      return display
    def createDistanceSensor(self, name):
      return DistanceSensor(name)
    def getDistanceSensor(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      distanceSensor = self.createDistanceSensor(name)
      self.devices[name] = distanceSensor
      return distanceSensor
    def createEmitter(self, name):
      return Emitter(name)
    def getEmitter(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      emitter = self.createEmitter(name)
      self.devices[name] = emitter
      return emitter
    def createGPS(self, name):
      return GPS(name)
    def getGPS(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      gps = self.createGPS(name)
      self.devices[name] = gps
      return gps
    def createGyro(self, name):
      return Gyro(name)
    def getGyro(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      gyro = self.createGyro(name)
      self.devices[name] = gyro
      return gyro
    def createLED(self, name):
      return LED(name)
    def getLED(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      led = self.createLED(name)
      self.devices[name] = led
      return led
    def createLightSensor(self, name):
      return LightSensor(name)
    def getLightSensor(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      lightSensor = self.createLightSensor(name)
      self.devices[name] = lightSensor
      return lightSensor
    def createMicrophone(self, name):
      return Microphone(name)
    def getMicrophone(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      microphone = self.createMicrophone(name)
      self.devices[name] = microphone
      return microphone
    def createPen(self, name):
      return Pen(name)
    def getPen(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      pen = self.createPen(name)
      self.devices[name] = pen
      return pen
    def createReceiver(self, name):
      return Receiver(name)
    def getReceiver(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      receiver = self.createReceiver(name)
      self.devices[name] = receiver
      return receiver
    def createServo(self, name):
      return Servo(name)
    def getServo(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      servo = self.createServo(name)
      self.devices[name] = servo
      return servo
    def createSpeaker(self, name):
      return Speaker(name)
    def getSpeaker(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      speaker = self.createSpeaker(name)
      self.devices[name] = speaker
      return speaker
    def createTouchSensor(self, name):
      return TouchSensor(name)
    def getTouchSensor(self,name):
      if (self.devices.has_key(name)):
        return self.devices[name]
      if (not Device.exists(name)):
        return None
      touchSensor = self.createTouchSensor(name)
      self.devices[name] = touchSensor
      return touchSensor
  %}
}

%include <webots/Robot.hpp>

//----------------------------------------------------------------------------------------------
//  DifferentialWheels
//----------------------------------------------------------------------------------------------

%include <webots/DifferentialWheels.hpp>

//----------------------------------------------------------------------------------------------
//  Supervisor
//----------------------------------------------------------------------------------------------

%include <webots/Supervisor.hpp>
