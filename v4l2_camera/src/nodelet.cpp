/*!
 *  \file	nodelet.cpp
 */
#include "TU/CameraArrayNode.h"
#include "TU/V4L2CameraArray.h"
#include <pluginlib/class_list_macros.h>

namespace TU
{
    using v4l2_camera_nodelet = CameraArrayNodelet<V4L2CameraArray>;
}

PLUGINLIB_EXPORT_CLASS(TU::v4l2_camera_nodelet, nodelet::Nodelet);
