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
  class EdgeDetect_MagAng
  {
  public:
    EdgeDetect_MagAng() {}
  
    #pragma hls_design interface
    void CCS_BLOCK(run)(ac_channel<gradType4x> &dx_chan,
                        ac_channel<gradType4x> &dy_chan,
                        ac_channel<pixelType4x>   &pix_chan2,
                        maxWType               &widthIn,
                        maxHType               &heightIn,
                        bool                   &sw_in,
                        uint32                 &crc32_pix_in,
                        uint32                 &crc32_dat_out,
                        ac_channel<Stream_t>   &dat_out)
    {
      gradType4x dx, dy;
      //pixelType  sum_abs;
      pixelType   dx_abs, dy_abs;
      pixelType4x   sum; // 4x Sum of two absolute derivatives ranging from 2 to 255
      pixelType4x pix4;
      Stream_t dat;

      uint32 crc32_pix_in_tmp = 0XFFFFFFFF;
      uint32 crc32_dat_out_tmp = 0XFFFFFFFF;

      MROW: for (maxHType y = 0; ; y++) {
        #pragma hls_pipeline_init_interval 1
        MCOL: for (maxWType x = 0; ; x+=4) {
          dy = dy_chan.read();
          dx = dx_chan.read();
          #pragma unroll yes
          for (int i = 0; i <= 3; i++){
            ac_math::ac_abs(dx.slc<9>(9*i), dx_abs);
            ac_math::ac_abs(dy.slc<9>(9*i), dy_abs);
            uint9 sum_abs = dx_abs + dy_abs;
            ac_fixed<8, 8, false, AC_TRN,AC_SAT> sum_abs_clip = sum_abs; 
            magType clip_tmp = (magType) sum_abs_clip.to_uint();
            sum.set_slc<8>(8*i, clip_tmp);
          }
          // Catapult's math library piecewise linear implementation of sqrt and atan2
          pix4 = pix_chan2.read();

          if (sw_in == 0){
            dat.pix = pix4;
          } else {
            dat.pix = sum;
          }
          dat.sof = (x == 0 && y == 0);
          dat.eol = (x == maxWType(widthIn - 4));

          dat_out.write(dat);
          
          crc32_pix_in_tmp = calc_crc32<32>(crc32_pix_in_tmp, pix4);
          crc32_dat_out_tmp = calc_crc32<32>(crc32_dat_out_tmp, dat.pix);
          
          if (x == maxWType(widthIn - 4)) { // cast to maxWType for RTL code coverage
            break;
          }
        }

        // programmable height exit condition
        if (y == maxHType(heightIn-1)) { // cast to maxHType for RTL code coverage
          break;
        }
      }
      crc32_pix_in = ~crc32_pix_in_tmp;
      crc32_dat_out = ~crc32_dat_out_tmp;
      
    }
  private:
    template <int len>
    uint32 calc_crc32(uint32 crc_in, ac_int<len, false> dat_in)
    {
      const uint32 CRC_POLY = 0xEDB88320;
      uint32 crc_tmp = crc_in;

      #pragma hls_unroll yes
      for(int i=0; i<len; i++)
      {
        uint1 tmp_bit = crc_tmp[0] ^ dat_in[i];

        uint31 mask;

        #pragma hls_unroll yes
        for(int i=0; i<31; i++)
          mask[i] = tmp_bit & CRC_POLY[i];

        uint31 crc_tmp_h31 = crc_tmp.slc<31>(1);
    
        crc_tmp_h31 ^= mask;
        
        crc_tmp.set_slc(31,tmp_bit);
        crc_tmp.set_slc(0,crc_tmp_h31);
      }
      return crc_tmp;
    }
  };  
}


