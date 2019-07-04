/*!
 *  \file	IIDCCameraArrayNode.cpp
 */
#include "TU/CameraArrayNode.h"
#include "TU/IIDCCameraArray.h"

namespace TU
{
constexpr static u_int	OFFSET_ABS_VAL = 0x2;
    
/************************************************************************
*  class CameraArrayNode<IIDCCameraArray>				*
************************************************************************/
template <> void
CameraArrayNode<IIDCCameraArray>::add_parameters()
{
    const auto&	camera = _cameras[0];

  // Add format commands.
    for (const auto& formatName : IIDCCamera::formatNames)
    {
	const auto	inq = camera.inquireFrameRate(formatName.format);

	ReconfServer::Enums	enums;
	IIDCCamera::FrameRate	frameRate;
	for (const auto& frameRateName : IIDCCamera::frameRateNames)
	    if (inq & frameRateName.frameRate)
	    {
		enums.add(frameRateName.name, frameRateName.frameRate);
		frameRate = frameRateName.frameRate;
	    }
	enums.end();

	if (!enums.empty())
	{
	    if (formatName.format == camera.getFormat())
		frameRate = camera.getFrameRate();
	    
	    const auto	parent = _reconf_server.addGroup(formatName.name);
	    _reconf_server.addParam(formatName.format,
				    "Frame Rate", "Select frame rate.", enums,
				    frameRate, parent);
	}
    }

  // Add feature commands.
    for (const auto& featureName : IIDCCamera::featureNames)
    {
	const auto	feature	= featureName.feature;
	const auto	name	= featureName.name;
	const auto	inq	= camera.inquireFeatureFunction(feature);

	if (!((inq & IIDCCamera::Presence) &&
	      (inq & IIDCCamera::Manual)   &&
	      (inq & IIDCCamera::ReadOut)))
	    continue;

	const auto	parent = _reconf_server.addGroup(name, true, false);
	switch (feature)
	{
	  case IIDCCamera::TRIGGER_MODE:
	  {
	    ReconfServer::Enums  enums;
	    for (const auto& triggerModeName : IIDCCamera::triggerModeNames)
		if (inq & triggerModeName.triggerMode)
		    enums.add(triggerModeName.name,
			      triggerModeName.triggerMode);
	    enums.end();

	    _reconf_server.addParam(feature,
				    "Value", "Select trigger mode.", enums,
				    camera.getTriggerMode(), parent);
	  }
	    break;

	  case IIDCCamera::WHITE_BALANCE:
	  {
	    u_int	min, max;
	    camera.getMinMax(feature, min, max);
	    u_int	ub, vr;
	    camera.getWhiteBalance(ub, vr);
		
	    _reconf_server.addParam<int>(feature,
					 "U/B", "White bal.(U/V)",
					 min, max, ub, parent);
	    _reconf_server.addParam<int>(feature + IIDCCAMERA_OFFSET_VR,
					 "V/R", "White bal.(V/R)",
					 min, max, vr, parent);

	    if (inq & IIDCCamera::Abs_Control)
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		float	ub, vr;
		camera.getWhiteBalance(ub, vr);

		_reconf_server.addParam<double>(feature + OFFSET_ABS_VAL,
						"Abs U/B", "White bal.(U/V)",
						min, max, ub, parent);
		_reconf_server.addParam<double>(feature + IIDCCAMERA_OFFSET_VR
							+ OFFSET_ABS_VAL,
						"Abs V/R", "White bal.(V/R)",
						min, max, vr, parent);
	    }
	  }
	    break;

	  default:
	  {
	    u_int	min, max;
	    camera.getMinMax(feature, min, max);
	    _reconf_server.addParam<int>(feature, "Value", name, min, max,
					 camera.getValue(feature), parent);
	    
	    if (inq & IIDCCamera::Abs_Control)
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		_reconf_server.addParam<double>(
					feature + OFFSET_ABS_VAL,
					"Abs_Value", name, min, max,
					camera.getValue<float>(feature),
					parent);
	    }
	  }
	    break;
	}

	if (inq & IIDCCamera::OnOff)
	    _reconf_server.addParam(feature + IIDCCAMERA_OFFSET_ONOFF,
				    "On", "Feature is enabled.",
				    camera.isActive(feature), parent);

	if (inq & IIDCCamera::Auto)
	    if (feature == IIDCCamera::TRIGGER_MODE)
		_reconf_server.addParam(feature + IIDCCAMERA_OFFSET_AUTO,
					"Positive",
					"Positive trigger polarity",
					camera.getTriggerPolarity(), parent);
	    else
		_reconf_server.addParam(feature + IIDCCAMERA_OFFSET_AUTO,
					"Auto",
					"Feature value is set automatically.",
					camera.isAuto(feature), parent);

	if (inq & IIDCCamera::Abs_Control)
	    _reconf_server.addParam(feature + IIDCCAMERA_OFFSET_ABS,
				    "Abs",
				    "Feature value is specified in an absolute one.",
				    camera.isAbsControl(feature), parent);
    }
}

template <> void
CameraArrayNode<IIDCCameraArray>::set_feature(
    camera_t& camera, const ReconfServer::Param& param) const
{
    if (param.level >= IIDCCamera::YUV444_160x120 &&
	param.level <= IIDCCamera::Format_7_7)
    {
	TU::setFormat(camera, param.level, param.value<int>());
    }
    else
    {
	const auto	level = (param.type_info() == typeid(double) ?
				 param.level - OFFSET_ABS_VAL : param.level);
	u_int		val;
	float		fval;
	TU::getFeature(camera, level, val, fval);

	if (param.type_info() == typeid(bool))
	    TU::setFeature(camera, level, param.value<bool>(), fval);
	else if (param.type_info() == typeid(int))
	    TU::setFeature(camera, level, param.value<int>(), fval);
	else if (param.type_info() == typeid(double))
	    TU::setFeature(camera, level, val, param.value<double>());
    }
}
    
template <> void
CameraArrayNode<IIDCCameraArray>::get_feature(
    const camera_t& camera, ReconfServer::Param& param) const
{
    if (param.level >= IIDCCamera::YUV444_160x120 &&
	param.level <= IIDCCamera::Format_7_7)
    {
	if (param.level == camera.getFormat())
	    param.setValue(camera.getFrameRate());
    }
    else
    {
	const auto	level = (param.type_info() == typeid(double) ?
				 param.level - OFFSET_ABS_VAL : param.level);
	u_int		val;
	float		fval;
	TU::getFeature(camera, level, val, fval);
	  
	if (param.type_info() == typeid(bool))
	    param.setValue(bool(val));
	else if (param.type_info() == typeid(int))
	    param.setValue(int(val));
	else if (param.type_info() == typeid(double))
	    param.setValue(double(fval));
    }
}
    
template <> void
CameraArrayNode<IIDCCameraArray>::publish_image(
    const camera_t& camera, const header_t& header,
    const image_transport::Publisher& pub) const
{
    using namespace	sensor_msgs;

    switch (camera.pixelFormat())
    {
      case IIDCCamera::MONO_8:
	switch (camera.bayerTileMapping())
	{
	  case IIDCCamera::RGGB:
	    publish_image<uint8_t>(camera, header, pub,
				   image_encodings::BAYER_RGGB8);
	    break;
	  case IIDCCamera::BGGR:
	    publish_image<uint8_t>(camera, header, pub,
				   image_encodings::BAYER_BGGR8);
	    break;
	  case IIDCCamera::GRBG:
	    publish_image<uint8_t>(camera, header, pub,
				   image_encodings::BAYER_GRBG8);
	    break;
	  case IIDCCamera::GBRG:
	    publish_image<uint8_t>(camera, header, pub,
				   image_encodings::BAYER_GBRG8);
	    break;
	  default:
	    publish_image<uint8_t>(camera, header, pub,
				   image_encodings::MONO8);
	    break;
	}
	break;

      case IIDCCamera::RAW_8:
	publish_image<uint8_t>(camera, header, pub, image_encodings::MONO8);
	break;

      case IIDCCamera::YUV_411:
	publish_image<YUV411>(camera, header, pub, image_encodings::YUV422);
	break;
	
      case IIDCCamera::MONO_16:
      case IIDCCamera::RAW_16:
	publish_image<uint16_t>(camera, header, pub, image_encodings::MONO16);
	break;

      case IIDCCamera::SIGNED_MONO_16:
	publish_image<int16_t>(camera, header, pub, image_encodings::MONO16);
	break;

      case IIDCCamera::YUV_422:
	publish_image<YUYV422>(camera, header, pub, image_encodings::YUV422);
	break;
	
      case IIDCCamera::YUV_444:
	publish_image<YUV444>(camera, header, pub, image_encodings::YUV422);
	break;

      case IIDCCamera::RGB_24:
	publish_image<RGB>(camera, header, pub, image_encodings::RGB8);
	break;
	
      default:
	ROS_WARN_STREAM("Unsupported image encoding!");
	break;
    }
}

}	// namespace TU
