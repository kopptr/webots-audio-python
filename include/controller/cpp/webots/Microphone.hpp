/**********************************************************************************/
/* File:          Microphone.hpp                                                  */
/* Date:          June 12                                                         */
/* Description:   Header file for the OO interface                                */
/* Authors:       Tim Kopp                                                        */
/* Copyright (c) 2012 SRC Inc. - www.srcinc.com                                   */
/**********************************************************************************/

#ifndef MICROPHONE_HPP
#define MICROPHONE_HPP

#include <webots/Device.hpp>

namespace webots {
   class Microphone : public Device {
      public:
         Microphone(const std::string &name) : Device(name) {} // Use Robot::getMicrophone() instead
         virtual ~Microphone() {}
         virtual void enable(int ms);
         virtual void disable();
         const void *getSampleData();
         int getSampleSize() const;
   };
}

#endif //MICROPHONE_HPP

