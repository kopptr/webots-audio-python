/**********************************************************************************/
/* File:          Microphone.cpp                                                  */
/* Date:          June 12                                                         */
/* Description:   Header file for the OO interface                                */
/* Authors:       Tim Kopp                                                        */
/* Copyright (c) 2012 SRC Inc. - www.srcinc.com                                   */
/**********************************************************************************/

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/Microphone.hpp>
#include <webots/microphone.h>

using namespace webots;

void Microphone::enable(int ms) {
   wb_microphone_enable(getTag(), ms);
}

void Microphone::disable() {
   wb_microphone_disable(getTag());
}

const void *Microphone::getSampleData() {
   return wb_microphone_get_sample_data(getTag());
}

int Microphone::getSampleSize() const {
   return wb_microphone_get_sample_size(getTag());
}

