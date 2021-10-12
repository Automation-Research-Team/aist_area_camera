/*!
 *  \file	IIDCCameraArrayNode.cpp
 */
#include <aist_area_camera/CameraArrayNode.h>
#include "TU/IIDCCameraArray.h"

namespace aist_area_camera
{
constexpr static u_int	OFFSET_ABS_VAL = 0x2;

/************************************************************************
*  static functions							*
************************************************************************/
static void
set_feature(TU::IIDCCamera& camera, int feature, int val)
{
    u_int	uval;
    float	fval;
    TU::getFeature(camera, feature, uval, fval);
    TU::setFeature(camera, feature, val,  fval);
}

static void
set_feature(TU::IIDCCamera& camera, int feature, double val)
{
    u_int	uval;
    float	fval;
    TU::getFeature(camera, feature - OFFSET_ABS_VAL, uval, fval);
    TU::setFeature(camera, feature - OFFSET_ABS_VAL, uval, val);
}

/************************************************************************
*  class CameraArrayNode<TU::IIDCCameraArray>				*
************************************************************************/
template <> bool
CameraArrayNode<TU::IIDCCameraArray>::one_shot_cb(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
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
CameraArrayNode<TU::IIDCCameraArray>::embed_timestamp(bool enable)
{
    for (auto& camera : _cameras)
	camera.embedTimestamp(enable);
}

template <> void
CameraArrayNode<TU::IIDCCameraArray>::add_parameters()
{
    const auto&	camera = _cameras[0];

  // Add format commands.
    int	n = 0;
    for (const auto& formatName : camera_t::formatNames)
    {
	const auto	inq = camera.inquireFrameRate(formatName.format);

	// if (formatName.format >= camera_t::Format_7_0)
	//     continue;

	std::map<std::string, int>	enums;
	camera_t::FrameRate		frameRate;
	for (const auto& frameRateName : camera_t::frameRateNames)
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
		std::string(formatName.name) + "/frame_rate", frameRate,
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, formatName.format, _1),
		"Select frame rate", enums);
	}
    }

  // Add feature commands.
    for (const auto& featureName : camera_t::featureNames)
    {
	const auto	feature	= featureName.feature;
	const auto	name	= std::string(featureName.name);
	const auto	inq	= camera.inquireFeatureFunction(feature);

	if (!((inq & camera_t::Presence) &&
	      (inq & camera_t::Manual)   &&
	      (inq & camera_t::ReadOut)))
	    continue;

	switch (feature)
	{

	  case camera_t::TRIGGER_MODE:
	  {
	    std::map<std::string, int>	enums;
	    for (const auto& triggerModeName : camera_t::triggerModeNames)
		if (inq & triggerModeName.triggerMode)
		    enums.emplace(triggerModeName.name,
				  triggerModeName.triggerMode);

	    if (!enums.empty())
		_ddr.registerEnumVariable<int>(
		    name + "/value", camera.getTriggerMode(),
		    boost::bind(&CameraArrayNode::set_feature_cb<int>,
				this, feature, _1),
		    "Trigger mode selection", enums, "", name);
	  }
	    break;

	  case camera_t::WHITE_BALANCE:
	  {
	    u_int	min, max;
	    camera.getMinMax(feature, min, max);
	    u_int	ub, vr;
	    camera.getWhiteBalance(ub, vr);

	    _ddr.registerVariable<int>(
		name + "/UB", ub,
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature, _1),
		"White balance(U/V)", min, max, name);
	    _ddr.registerVariable<int>(
		name + "/VR", vr,
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature + TU::IIDCCAMERA_OFFSET_VR, _1),
		"White balance(V/R)", min, max, name);

	    if (inq & camera_t::Abs_Control)
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		float	ub, vr;
		camera.getWhiteBalance(ub, vr);

		_ddr.registerVariable<double>(
		    name + "/UB_abs", ub,
		    boost::bind(&CameraArrayNode::set_feature_cb<double>,
				this, feature + OFFSET_ABS_VAL, _1),
		    "White balance(U/V) in absolute values",
		    min, max, name);
		_ddr.registerVariable<double>(
		    name + "/VR_abs", vr,
		    boost::bind(&CameraArrayNode::set_feature_cb<double>,
				this,
				feature + TU::IIDCCAMERA_OFFSET_VR
					+ OFFSET_ABS_VAL,
				_1),
		    "White balance(V/R) in absolute values",
		    min, max, name);
	    }
	  }
	    break;

	  default:
	  {
	    u_int	min, max;
	    camera.getMinMax(feature, min, max);
	    _ddr.registerVariable<int>(
		name + "/value", camera.getValue(feature),
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature, _1),
		"Feature values", min, max, name);

	    if (inq & camera_t::Abs_Control)
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		_ddr.registerVariable<double>(
		    name + "/abs_value", camera.getValue<float>(feature),
		    boost::bind(&CameraArrayNode::set_feature_cb<double>,
				this, feature + OFFSET_ABS_VAL, _1),
		    "Absolute feature values", min, max, name);
	    }
	  }
	    break;
	}

	if (inq & camera_t::OnOff)
	    _ddr.registerVariable<bool>(
		name + "/active", camera.isActive(feature),
		boost::bind(&CameraArrayNode::set_feature_cb<bool>, this,
			    feature + TU::IIDCCAMERA_OFFSET_ONOFF, _1),
		"Feature activity", false, true, name);

	if (inq & camera_t::Auto)
	    if (feature == camera_t::TRIGGER_MODE)
		_ddr.registerVariable<bool>(
		    name + "/polarity", camera.getTriggerPolarity(),
		    boost::bind(&CameraArrayNode::set_feature_cb<bool>, this,
				feature + TU::IIDCCAMERA_OFFSET_AUTO, _1),
		    "Positive polarity", false, true, name);
	    else
		_ddr.registerVariable<bool>(
		    name + "/auto", camera.isAuto(feature),
		    boost::bind(&CameraArrayNode::set_feature_cb<bool>, this,
				feature + TU::IIDCCAMERA_OFFSET_AUTO, _1),
		    "Feature values set automatically", false, true, name);

	if (inq & camera_t::Abs_Control)
	    _ddr.registerVariable<bool>(
		name + "/abs", camera.isAbsControl(feature),
		boost::bind(&CameraArrayNode::set_feature_cb<bool>, this,
			    feature + TU::IIDCCAMERA_OFFSET_ABS, _1),
		"Feature in absolute values", false, true, name);
    }
}

template <> template <class T> void
CameraArrayNode<TU::IIDCCameraArray>::set_feature_cb(int feature, T val)
{
    if (feature >= camera_t::YUV444_160x120 &&
	feature <= camera_t::Format_7_7)
    {
	if (_n < _cameras.size())
	    TU::setFormat(_cameras[_n], feature, val);
	else
	    for (auto& camera : _cameras)
		TU::setFormat(camera, feature, val);
    }
    else
    {
	if (_n < _cameras.size())
	    set_feature(_cameras[_n], feature, val);
	else
	    for (auto& camera : _cameras)
		set_feature(camera, feature, val);
    }
}

template <> void
CameraArrayNode<TU::IIDCCameraArray>::publish(
    const camera_t& camera, const header_t& header, const cmodel_t& cmodel,
    const image_transport::CameraPublisher& pub) const
{
    using namespace	sensor_msgs;

    switch (camera.pixelFormat())
    {
      case camera_t::MONO_8:
	switch (camera.bayerTileMapping())
	{
	  case camera_t::RGGB:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_RGGB8);
	    break;
	  case camera_t::BGGR:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_BGGR8);
	    break;
	  case camera_t::GRBG:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_GRBG8);
	    break;
	  case camera_t::GBRG:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::BAYER_GBRG8);
	    break;
	  default:
	    publish<uint8_t>(camera, header, cmodel, pub,
			     image_encodings::MONO8);
	    break;
	}
	break;

      case camera_t::RAW_8:
	publish<uint8_t>(camera, header, cmodel, pub, image_encodings::MONO8);
	break;

      case camera_t::YUV_411:
	publish<TU::YUV411>(camera, header, cmodel, pub,
			    image_encodings::YUV422);
	break;

      case camera_t::MONO_16:
      case camera_t::RAW_16:
	publish<uint16_t>(camera, header, cmodel, pub,
			  image_encodings::MONO16);
	break;

      case camera_t::SIGNED_MONO_16:
	publish<int16_t>(camera, header, cmodel, pub, image_encodings::MONO16);
	break;

      case camera_t::YUV_422:
	publish<TU::YUYV422>(camera, header, cmodel, pub,
			     image_encodings::YUV422);
	break;

      case camera_t::YUV_444:
	publish<TU::YUV444>(camera, header, cmodel, pub,
			    image_encodings::YUV422);
	break;

      case camera_t::RGB_24:
	publish<TU::RGB>(camera, header, cmodel, pub, image_encodings::RGB8);
	break;

      default:
	ROS_WARN_STREAM("Unsupported image encoding!");
	break;
    }
}

}	// namespace aist_area_camera
