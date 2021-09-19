/*!
 *  \file	nodelet.cpp
 */
#include <aist_area_camera/CameraArrayNode.h>
#include <pluginlib/class_list_macros.h>
#include "TU/IIDCCameraArray.h"

namespace aist_area_camera
{
    using iidc_camera_nodelet = CameraArrayNodelet<TU::IIDCCameraArray>;
}

PLUGINLIB_EXPORT_CLASS(aist_area_camera::iidc_camera_nodelet,
		       nodelet::Nodelet);
