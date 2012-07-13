/**********************************************************************************/
/* File:          Speaker.hpp                                                  */
/* Date:          June 12                                                         */
/* Description:   Header file for the OO interface                                */
/* Author:        Tim Kopp                                                        */
/* Copyright (c) 2012 SRC Inc. - www.srcinc.com                                   */
/**********************************************************************************/

#ifndef SPEAKER_HPP
#define SPEAKER_HPP

#include <webots/Device.hpp>

namespace webots {
   class Speaker : public Device {
      public:
         Speaker(const std::string &name) : Device(name) {} // Use Robot::getSpeaker() instead
         virtual ~Speaker() {}
         virtual void emitSample(const void *data, int size);
   };
}

#endif //SPEAKER_HPP


