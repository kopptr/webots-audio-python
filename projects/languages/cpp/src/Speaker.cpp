/**********************************************************************************/
/* File:          Speaker.hpp                                                  */
/* Date:          June 12                                                         */
/* Description:   Header file for the OO interface                                */
/* Author:        Tim Kopp                                                        */
/* Copyright (c) 2012 SRC Inc. - www.srcinc.com                                   */
/**********************************************************************************/

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/Speaker.hpp>
#include <webots/speaker.h>

using namespace webots;

void Speaker::emitSample(const void *data, int size) {
   return wb_speaker_emit_sample(getTag(), data, size);
}
