#pragma once

#include <cstdint>
#include <vector>

#include "Time.h"
//#include "psl/slam/SlamResult.h"

namespace psl {

//每个点的深度数据
struct TofDepthPoint {
  float X;
  float Y;
  float Z;
  float noise;  // HFF
  uint16_t grayValue;
  uint8_t depthConfidence;
};

struct TofDepthData {
  int tofID;                    // id号
  uint32_t exposureTime;        //曝光时间 
  uint8_t sn[8];                //序列号，serial number
  int width;                    //宽 224
  int height;                   //高 172
  uint64_t frameIndex;          //帧号
  Time time_stamp;  //时间戳
  TofDepthPoint data[224*109];
//  psl::SlamResult pose;
};

}  // psl