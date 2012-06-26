WEBOTS        = ../../../..

OSTYPE ?= $(shell uname)

ifeq ($(OSTYPE),linux)
OSTYPE=Linux
endif

ifneq ($(findstring MINGW,$(OSTYPE)),)
OSTYPE=Windows
endif

ifeq ($(OSTYPE),msys)
OSTYPE=Windows
endif

CPPSRC        = Accelerometer.cpp \
                Camera.cpp \
                Compass.cpp \
                Connector.cpp \
                Device.cpp \
                DifferentialWheels.cpp \
                Display.cpp \
                DistanceSensor.cpp \
                Emitter.cpp \
                Field.cpp \
                GPS.cpp \
                Gyro.cpp \
                LED.cpp \
                LightSensor.cpp \
					 Microphone.cpp \
                Motion.cpp \
                Node.cpp \
                Pen.cpp \
                Receiver.cpp \
                Robot.cpp \
                Servo.cpp \
					 Speaker.cpp \
                Supervisor.cpp \
                TouchSensor.cpp

CPPOBJS        = $(CPPSRC:.cpp=.o)
DEPENDENCIES   = $(CPPSRC:.cpp=.d)

ifeq ($(OSTYPE),Windows)
LDFLAGS       = -shared -Wl,--out-implib,$(WEBOTS)/lib/libCppController.a
LIBCONTROLLER = $(WEBOTS)/lib/Controller.dll
SHAREDLIBS    = $(LIBCONTROLLER)
CPPFLAGS      = -c -Wall -mwindows
CPPINCLUDES   = -I"$(WEBOTS)/include/controller/c" -I"$(WEBOTS)/include/controller/cpp"
TARGET        = $(WEBOTS)/lib/CppController.dll
DEF           = CppController.def
LIB           = CppController.lib
PEXPORTS_EXE  = $(shell which pexports)
LIB_EXE       = $(shell which lib)
endif

ifeq ($(OSTYPE),Darwin)
LDFLAGS       = -arch i386 -arch x86_64 -dynamiclib -install_name libCppController.dylib -compatibility_version 1.0 -current_version 1.0.0
SHAREDLIBS    = -L"$(WEBOTS)/lib" -lController
CPPFLAGS      = -c -fPIC -Wall -arch i386 -arch x86_64
CPPINCLUDES   = -I"$(WEBOTS)/include/controller/c" -I"$(WEBOTS)/include/controller/cpp"
LIBCONTROLLER = $(WEBOTS)/lib/libController.dylib
TARGET        = $(WEBOTS)/lib/libCppController.dylib
endif

ifeq ($(OSTYPE),Linux)
LDFLAGS       = -shared
SHAREDLIBS    = -L"$(WEBOTS)/lib" -lController
CPPFLAGS      = -c -fpic -Wall
CPPINCLUDES   = -I"$(WEBOTS)/include/controller/c" -I"$(WEBOTS)/include/controller/cpp"
LIBCONTROLLER = $(WEBOTS)/lib/libController.so
TARGET        = $(WEBOTS)/lib/libCppController.so
endif

all debug profile release compil:$(TARGET)

$(TARGET):$(CPPOBJS) $(LIBCONTROLLER)
	$(CXX) $(LDFLAGS) $(SHAREDLIBS) $(CPPOBJS) -o $@ > /dev/null 2>&1;
ifeq ($(OSTYPE),Windows)
ifneq ($(PEXPORTS_EXE),)
ifneq ($(LIB_EXE),)
	pexports $(TARGET) > $(DEF)
	lib.exe /machine:IX86 /def:$(DEF) /out:$(LIB) > /dev/null 2>&1;
	mv $(LIB) $(WEBOTS)/lib/
else
	@echo "Warning: lib.exe is missing"
endif
else
	@echo "Warning: pexports.exe is missing"
endif
endif

$(LIBCONTROLLER):
	@echo "Warning: $(LIBCONTROLLER) doesn't exist, recompile it first."

-include $(DEPENDENCIES)

%.o:%.cpp
	$(CXX) $(CPPFLAGS) $(CPPINCLUDES) -o $@ $<
	$(CXX) $(CPPINCLUDES) -MM $< > $*.d

clean:
	rm -f $(WEBOTS)/lib/*Cpp* *.def *.exp $(CPPOBJS) $(DEPENDENCIES)

