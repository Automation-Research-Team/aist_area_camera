/*!
 *  \file	nodelet.cpp
 */
#include "TU/CameraArrayNode.h"
#include "TU/IIDCCameraArray.h"
#include <pluginlib/class_list_macros.h>

namespace TU
{
    using iidc_camera_nodelet = CameraArrayNodelet<IIDCCameraArray>;
}

PLUGINLIB_EXPORT_CLASS(TU::iidc_camera_nodelet, nodelet::Nodelet);
