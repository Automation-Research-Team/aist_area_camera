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
template <> bool
CameraArrayNode<IIDCCameraArray>::one_shot_cb(std_srvs::Trigger::Request&  req,
					      std_srvs::Trigger::Response& res)
{
    if (_cameras[0].inContinuousShot())
    {
	res.success = false;
	res.message = "failed. [in continuous shot mode]";
	ROS_WARN_STREAM(res.message);
	return;
    }

    for (auto& camera : _cameras)
	camera.oneShot().snap();

    publish();

    res.success = true;
    res.message = "succeded.";

    return true;
}

template <> void
CameraArrayNode<IIDCCameraArray>::embed_timestamp(bool enable)
{
    for (auto& camera : _cameras)
	camera.embedTimestamp(enable);
}
    
template <> void
CameraArrayNode<IIDCCameraArray>::add_parameters()
{
    const auto&	camera = _cameras[0];

  // Add format commands.
    for (const auto& formatName : IIDCCamera::formatNames)
    {
	const auto	inq = camera.inquireFrameRate(formatName.format);

	std::map<std::string, int>	enums;
	IIDCCamera::FrameRate		frameRate;
	for (const auto& frameRateName : IIDCCamera::frameRateNames)
	    if (inq & frameRateName.frameRate)
	    {
		enums.emplace(frameRateName.name, frameRateName.frameRate);
		frameRate = frameRateName.frameRate;
	    }

	if (!enums.empty())
	{
	    if (formatName.format == camera.getFormat())
		frameRate = camera.getFrameRate();

	    _ddr.registerEnumVariable<int>(
		formatName.name, frameRate,
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, formatName.format, _1),
		"Select frame rate.", enums);
	}
    }

  // Add feature commands.
    for (const auto& featureName : IIDCCamera::featureNames)
    {
	const auto	feature	= featureName.feature;
	const auto	name	= std::string(featureName.name);
	const auto	inq	= camera.inquireFeatureFunction(feature);

	if (!((inq & IIDCCamera::Presence) &&
	      (inq & IIDCCamera::Manual)   &&
	      (inq & IIDCCamera::ReadOut)))
	    continue;

	switch (feature)
	{
	  case IIDCCamera::TRIGGER_MODE:
	  {
	    std::map<std::string, int>	enums;
	    for (const auto& triggerModeName : IIDCCamera::triggerModeNames)
		if (inq & triggerModeName.triggerMode)
		    enums.emplace(triggerModeName.name,
				  triggerModeName.triggerMode);

	    _ddr.registerEnumVariable<int>(
		name, camera.getTriggerMode(),
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature, _1),
		"Select trigger mode.", enums);
	  }
	    break;

	  case IIDCCamera::WHITE_BALANCE:
	  {
	    u_int	min, max;
	    camera.getMinMax(feature, min, max);
	    u_int	ub, vr;
	    camera.getWhiteBalance(ub, vr);

	    _ddr.registerVariable<int>(
		"UB", ub,
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature, _1),
		"White bal.(U/V)", min, max);
	    _ddr.registerVariable<int>(
		"VR", vr,
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature + IIDCCAMERA_OFFSET_VR, _1),
		"White bal.(V/R)", min, max);

	    if (inq & IIDCCamera::Abs_Control)
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		float	ub, vr;
		camera.getWhiteBalance(ub, vr);

		_ddr.registerVariable<double>(
		    "Abs_UB", ub,
		    boost::bind(&CameraArrayNode::set_feature_cb<double>,
				this, feature + OFFSET_ABS_VAL, _1),
		    "White bal.(U/V)", min, max);
		_ddr.registerVariable<double>(
		    "Abs_VR", vr,
		    boost::bind(&CameraArrayNode::set_feature_cb<double>,
				this,
				feature + IIDCCAMERA_OFFSET_VR+OFFSET_ABS_VAL,
				_1),
		    "White bal.(V/R)", min, max);
	    }
	  }
	    break;

	  default:
	  {
	    u_int	min, max;
	    camera.getMinMax(feature, min, max);
	    _ddr.registerVariable<int>(
		name, camera.getValue(feature),
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature, _1),
		name, min, max);

	    if (inq & IIDCCamera::Abs_Control)
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		_ddr.registerVariable<double>(
		    name + "_Abs", camera.getValue<float>(feature),
		    boost::bind(&CameraArrayNode::set_feature_cb<double>,
				this, feature + OFFSET_ABS_VAL, _1),
		    name + "(abs. value)", min, max);
	    }
	  }
	    break;
	}

	if (inq & IIDCCamera::OnOff)
	    _ddr.registerVariable<bool>(
		name + "_OnOff", camera.isActive(feature),
		boost::bind(&CameraArrayNode::set_feature_cb<bool>,
			    this, feature + IIDCCAMERA_OFFSET_ONOFF, _1),
		"Feature is enabled.");

	if (inq & IIDCCamera::Auto)
	    if (feature == IIDCCamera::TRIGGER_MODE)
		_ddr.registerVariable<bool>(
		    name + "_Positive", camera.getTriggerPolarity(),
		    boost::bind(&CameraArrayNode::set_feature_cb<bool>,
				this, feature + IIDCCAMERA_OFFSET_AUTO, _1),
		    "Positive trigger polarity");
	    else
		_ddr.registerVariable<bool>(
		    name + "_Auto", camera.isAuto(feature),
		    boost::bind(&CameraArrayNode::set_feature_cb<bool>,
				this, feature + IIDCCAMERA_OFFSET_AUTO, _1),
		    "Feature value is set automatically.");

	if (inq & IIDCCamera::Abs_Control)
	    _ddr.registerVariable<bool>(
		name + "_Abs", camera.isAbsControl(feature),
		boost::bind(&CameraArrayNode::set_feature_cb<bool>,
			    this, feature + IIDCCAMERA_OFFSET_ABS, _1),
		"Feature value is specified in an absolute one.");
    }
}

template <> template <class T> void
CameraArrayNode<IIDCCameraArray>::set_feature_cb(int feature, T val)
{
    if (feature >= IIDCCamera::YUV444_160x120 &&
	feature <= IIDCCamera::Format_7_7)
    {
	if (_n < _cameras.size())
	    TU::setFormat(_cameras[_n], feature, val);
	else
	    for (auto& camera : _cameras)
		TU::setFormat(camera, feature, val);
    }
    else
    {
	if (std::is_floating_point<T>::value)
	{
	    feature -= OFFSET_ABS_VAL;

	    if (_n < _cameras.size())
	    {
		u_int		uval;
		float		fval;
		TU::getFeature(_cameras[_n], feature, uval, fval);
		TU::setFeature(_cameras[_n], feature, uval, val);
	    }
	    else
	    {
		u_int		uval;
		float		fval;
		TU::getFeature(_cameras[0], feature, uval, fval);
		for (auto& camera : _cameras)
		    TU::setFeature(camera, feature, uval, val);
	    }
	}
	else
	{
	    if (_n < _cameras.size())
	    {
		u_int		uval;
		float		fval;
		TU::getFeature(_cameras[_n], feature, uval, fval);
		TU::setFeature(_cameras[_n], feature, val,  fval);
	    }
	    else
	    {
		u_int		uval;
		float		fval;
		TU::getFeature(_cameras[0], feature, uval, fval);
		for (auto& camera : _cameras)
		    TU::setFeature(camera, feature, val, fval);
	    }
	}
    }
}

template <> void
CameraArrayNode<IIDCCameraArray>::publish(
    const camera_t& camera, const header_t& header, const cmodel_t& cmodel,
    const image_transport::CameraPublisher& pub) const
{
    using namespace	sensor_msgs;

    switch (camera.pixelFormat())
    {
      case IIDCCamera::MONO_8:
	switch (camera.bayerTileMapping())
	{
	  case IIDCCamera::RGGB:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_RGGB8);
	    break;
	  case IIDCCamera::BGGR:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_BGGR8);
	    break;
	  case IIDCCamera::GRBG:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_GRBG8);
	    break;
	  case IIDCCamera::GBRG:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_GBRG8);
	    break;
	  default:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::MONO8);
	    break;
	}
	break;

      case IIDCCamera::RAW_8:
	publish<uint8_t>(camera, header, cmodel, pub, image_encodings::MONO8);
	break;

      case IIDCCamera::YUV_411:
	publish<YUV411>(camera, header, cmodel, pub, image_encodings::YUV422);
	break;

      case IIDCCamera::MONO_16:
      case IIDCCamera::RAW_16:
	publish<uint16_t>(camera, header, cmodel, pub,
			  image_encodings::MONO16);
	break;

      case IIDCCamera::SIGNED_MONO_16:
	publish<int16_t>(camera, header, cmodel, pub, image_encodings::MONO16);
	break;

      case IIDCCamera::YUV_422:
	publish<YUYV422>(camera, header, cmodel, pub, image_encodings::YUV422);
	break;

      case IIDCCamera::YUV_444:
	publish<YUV444>(camera, header, cmodel, pub, image_encodings::YUV422);
	break;

      case IIDCCamera::RGB_24:
	publish<RGB>(camera, header, cmodel, pub, image_encodings::RGB8);
	break;

      default:
	ROS_WARN_STREAM("Unsupported image encoding!");
	break;
    }
}

}	// namespace TU
