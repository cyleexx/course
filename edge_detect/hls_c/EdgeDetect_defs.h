/**************************************************************************
 *                                                                        *
 *  Edge Detect Design Walkthrough for HLS                                *
 *                                                                        *
 *  Software Version: 1.0                                                 *
 *                                                                        *
 *  Release Date    : Tue Jan 14 15:40:43 PST 2020                        *
 *  Release Type    : Production Release                                  *
 *  Release Build   : 1.0.0                                               *
 *                                                                        *
 *  Copyright 2020, Siemens                                               *
 *                                                                        *
 **************************************************************************
 *  Licensed under the Apache License, Version 2.0 (the "License");       *
 *  you may not use this file except in compliance with the License.      *
 *  You may obtain a copy of the License at                               *
 *                                                                        *
 *      http://www.apache.org/licenses/LICENSE-2.0                        *
 *                                                                        *
 *  Unless required by applicable law or agreed to in writing, software   *
 *  distributed under the License is distributed on an "AS IS" BASIS,     *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or       *
 *  implied.                                                              *
 *  See the License for the specific language governing permissions and   *
 *  limitations under the License.                                        *
 **************************************************************************
 *                                                                        *
 *  The most recent version of this package is available at github.       *
 *                                                                        *
 *************************************************************************/
#pragma once

#include <ac_int.h>
#include <ac_fixed.h>
#include <ac_channel.h>
#include <ac_math/ac_abs.h>
//#include <ac_math/ac_sqrt_pwl.h>
//#include <ac_math/ac_atan2_cordic.h>

//#define USE_CIRCULARBUF

namespace EdgeDetect_IP 
{
  const int maxImageWidth = 640; 
  const int maxImageHeight = 480; 
  
  const int kernel[3] = {1, 0, -1};
    
  // Define some bit-accurate types to use in this model
  typedef uint8                  pixelType;    // input pixel is 0-255
  typedef uint16                 pixelType2x;  // two pixels packed
  typedef int9                   gradType;     // Derivative is max range -255 to 255
  typedef uint18                 sqType;       // Result of 9-bit x 9-bit
  typedef ac_fixed<19,19,false>  sumType;      // Result of 18-bit + 18-bit fixed pt integer for squareroot
  typedef uint8                  magType;      // 9-bit unsigned magnitute result
  typedef ac_fixed<8,3,true>     angType;      // 3 integer bit, 5 fractional bits for quantized angle -pi to pi

//new defined types  
  typedef int36                  gradType4x;   // 4x Derivatives max range -255 to 255
  typedef uint32                 pixelType4x;  // four pixels packed
  typedef ac_int<64,false>       pixelType8x;   // 8x 8-bit pixels data
  typedef uint32                 magType4x;    // four 8-bit unsigned magnitute result
  struct                 Stream_t {
    ac_int<32,false> pix; 
    bool   sof; 
    bool   eol;
  };     // 4x 8-bit pixels data
  
    
  // Compute number of bits for max image size count, used internally and in testbench
  typedef ac_int<ac::nbits<maxImageWidth+1>::val,false> maxWType;
  typedef ac_int<ac::nbits<maxImageHeight+1>::val,false> maxHType;
}


