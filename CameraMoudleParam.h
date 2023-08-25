#pragma once

#include <map>
#include <string>
#include "Resolution.h"
#include "CameraParam.h"

namespace psl
{
struct CameraMoudleParam
{
    std::map<Resolution, CameraParam> _left_camera;
    std::map<Resolution, CameraParam> _right_camera;
};
}  // namespace psl
