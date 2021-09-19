/*!
 *  \file	nodelet.cpp
 */
#include <aist_area_camera/CameraArrayNode.h>
#include <pluginlib/class_list_macros.h>
#include "TU/V4L2CameraArray.h"

namespace aist_area_camera
{
    using v4l2_camera_nodelet = CameraArrayNodelet<TU::V4L2CameraArray>;
}

PLUGINLIB_EXPORT_CLASS(aist_area_camera::v4l2_camera_nodelet,
		       nodelet::Nodelet);
