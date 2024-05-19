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

#include "EdgeDetect_defs.h"
#include <mc_scverify.h>

namespace EdgeDetect_IP 
{
  class EdgeDetect_HorDer
  {
  public:
    EdgeDetect_HorDer() {}
  
    #pragma hls_design interface
    void CCS_BLOCK(run)(ac_channel<pixelType4x>    &pix_chan1,
                        maxWType                &widthIn,
                        maxHType                &heightIn,
                        ac_channel<pixelType4x>    &pix_chan2,
                        ac_channel<gradType4x>  &dx_chan)
    {
      // pixel buffers store pixel history
      gradType grad, grad0, grad1, grad2;
      gradType grad_buf0, grad_buf1, grad_buf2;
      gradType4x grad4;
      
      pixelType4x   pix4;
      pixelType4x   pix4_buf;
      pixelType pix_buf0, pix_buf1;

      HROW: for (maxHType y = 0; ; y++) {
        #pragma hls_pipeline_init_interval 1
        HCOL: for (maxWType x = 0; ; x+=4) { // HCOL has one extra iteration to ramp-up window
          if (x <= maxWType(widthIn - 4)) {
            pix4 = pix_chan1.read(); // Read streaming interface
          }

          // Calculate derivative
          if (x == 0) {
            //grad0 = pix4.slc<8>(0)*kernel[0] + pix4.slc<8>(0)*kernel[1]  + pix4.slc<8>(8)*kernel[2];
            grad0 = 0;
          } else {
            grad0 = pix_buf1*kernel[0] + pix4.slc<8>(0)*kernel[1]  + pix4.slc<8>(8)*kernel[2];
          }
          
          grad1 = pix4.slc<8>(0)*kernel[0] + pix4.slc<8>(8)*kernel[1]  + pix4.slc<8>(16)*kernel[2];
          grad2 = pix4.slc<8>(8)*kernel[0] + pix4.slc<8>(16)*kernel[1] + pix4.slc<8>(24)*kernel[2];
          
          if (x == widthIn) {
            //grad = pix_buf0*kernel[0] + pix_buf1*kernel[1] + pix_buf1*kernel[2];
            grad = 0;
          } else {
            grad  = pix_buf0*kernel[0] + pix_buf1*kernel[1] + pix4.slc<8>(0)*kernel[2];
          }
          

          if(x != 0) {
            grad4.set_slc<9>(0, grad_buf0);
            grad4.set_slc<9>(9, grad_buf1);
            grad4.set_slc<9>(18, grad_buf2);
            grad4.set_slc<9>(27, grad);
            pix_chan2.write(pix4_buf);
            dx_chan.write(grad4);
          }

          grad_buf0 = grad0;
          grad_buf1 = grad1;
          grad_buf2 = grad2;
          
          pix_buf0  = pix4.slc<8>(16);
          pix_buf1  = pix4.slc<8>(24);
          
          pix4_buf = pix4;

          // programmable width exit condition
          if (x == widthIn) { // cast to maxWType for RTL code coverage
            break;
          }
        }
        
        // programmable height exit condition
        if (y == maxHType(heightIn-1)) { // cast to maxHType for RTL code coverage
          break;
        }
      }
    }
  };

}


