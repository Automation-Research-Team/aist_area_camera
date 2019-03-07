/*!
 *  \file	main.cpp
 */
#include "TU/CameraArrayNode.h"
#include "TU/IIDCCameraArray.h"

namespace TU
{
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
	std::ostringstream
			edit_method;
	bool		init = true;
	
	edit_method << "{\'enum\': [";
	for (const auto& frameRateName : IIDCCamera::frameRateNames)
	    if (inq & frameRateName.frameRate)
	    {
		if (init)
		    init = false;
		else
		    edit_method << ", ";

		edit_method << "{\'value\': "   << frameRateName.frameRate
			    << ", \'name\': \'" << frameRateName.name
			    << "\'}";
	    }
	edit_method << "]}";
	
	const auto	parent = _reconf_server.addGroup(
					ReconfServer::DEFAULT_GROUP,
					formatName.name, true);
	_reconf_server.addParam<int>(parent, formatName.format,
				     "Frame Rate", "Select frame rate.",
				     edit_method.str(),
				     IIDCCamera::FrameRate_x,
				     IIDCCamera::FrameRate_1_875,
				     camera.getFrameRate());
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

	const auto	parent = _reconf_server.addGroup(
					ReconfServer::DEFAULT_GROUP,
					name, true);
	switch (feature)
	{
	  case IIDCCamera::TRIGGER_MODE:
	  {
	    std::ostringstream	edit_method;
	    bool		init = true;
	      
	    edit_method << "{\'enum\': [";
	    for (const auto& triggerModeName : IIDCCamera::triggerModeNames)
		if (inq & triggerModeName.triggerMode)
		{
		    if (init)
			init = false;
		    else
			edit_method << ", ";

		    edit_method << "{\'value\': "
				<< triggerModeName.triggerMode
				<<  ", \'name\': \'" << triggerModeName.name
				<< "\'}";
		}
	    edit_method << "]}";

	    _reconf_server.addParam<int>(parent, feature,
					 "Trigger Mode", "Select trigger mode.",
					 edit_method.str(),
					 IIDCCamera::Trigger_Mode15,
					 IIDCCamera::Trigger_Mode0,
					 camera.getTriggerMode());
	  }
	    break;

	  case IIDCCamera::WHITE_BALANCE:
	    if (camera.isAbsControl(feature))
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		float	ub, vr;
		camera.getWhiteBalance(ub, vr);

		_reconf_server.addParam<double>(
		    parent, feature, name, name, "",
		    min, max, ub);
		_reconf_server.addParam<double>(
		    parent, feature + IIDCCAMERA_OFFSET_VR,
		    "White bal.(V/R)", "White bal.(V/R)", "",
		    min, max, vr);
	    }
	    else
	    {
		u_int	min, max;
		camera.getMinMax(feature, min, max);
		u_int	ub, vr;
		camera.getWhiteBalance(ub, vr);
		
		_reconf_server.addParam<int>(parent, feature,
					     name, name, "",
					     min, max, ub);
		_reconf_server.addParam<int>(parent,
					     feature + IIDCCAMERA_OFFSET_VR,
					     "White bal.(V/R)",
					     "White bal.(V/R)", "",
					     min, max, vr);
	    }
	    break;

	  default:
	    if (camera.isAbsControl(feature))
	    {
		float	min, max;
		camera.getMinMax(feature, min, max);
		_reconf_server.addParam<double>(
		    parent, feature, name, name, "",
		    min, max,
		    camera.getValue<float>(feature));
	    }
	    else
	    {
		u_int	min, max;
		camera.getMinMax(feature, min, max);
		_reconf_server.addParam<int>(parent, feature,
					     name, name, "", min, max,
					     camera.getValue(feature));
	    }
	    break;
	}
	
	if (inq & IIDCCamera::OnOff)
	    _reconf_server.addParam(parent, feature + IIDCCAMERA_OFFSET_ONOFF,
				    "On", "Feature is enabled.", "", false, true,
				    camera.isActive(feature));

	if (inq & IIDCCamera::Auto)
	    if (feature == IIDCCamera::TRIGGER_MODE)
		_reconf_server.addParam(parent, feature + IIDCCAMERA_OFFSET_AUTO,
					"(+)", "Trigger polarity is Positive.", "",
					false, true, camera.getTriggerPolarity());
	    else
		_reconf_server.addParam(parent, feature + IIDCCAMERA_OFFSET_AUTO,
					"Auto", "Feature value is set automatically.",
					"", false, true, camera.isAuto(feature));

	if (inq & IIDCCamera::Abs_Control)
	    _reconf_server.addParam(parent, feature + IIDCCAMERA_OFFSET_ABS,
				    "Abs",
				    "Feature value is specified as an absolute one.",
				    "", false, true, camera.isAbsControl(feature));
    }
}

template <> void
CameraArrayNode<IIDCCameraArray>::set_feature(
    camera_t& camera, const ReconfServer::Param& param) const
{
    switch (param.level)
    {
      case IIDCCamera::MONO_8:
      case IIDCCamera::RAW_8:
      case IIDCCamera::YUV_411:
      case IIDCCamera::MONO_16:
      case IIDCCamera::RAW_16:
      case IIDCCamera::SIGNED_MONO_16:
      case IIDCCamera::YUV_422:
      case IIDCCamera::YUV_444:
      case IIDCCamera::RGB_24:
	TU::setFormat(camera, param.level, param.value<int>());
	break;
	
      default:
	if (param.type_info() == typeid(bool))
	    TU::setFeature(camera, param.level, param.value<bool>(), 0);
	else if (param.type_info() == typeid(int))
	    TU::setFeature(camera, param.level, param.value<int>(), 0);
	else if (param.type_info() == typeid(double))
	    TU::setFeature(camera, param.level, 0, param.value<double>());
	break;
    }
}
    
template <> void
CameraArrayNode<IIDCCameraArray>::get_feature(
    const camera_t& camera, ReconfServer::Param& param) const
{
    switch (param.level)
    {
      case IIDCCamera::MONO_8:
      case IIDCCamera::RAW_8:
      case IIDCCamera::YUV_411:
      case IIDCCamera::MONO_16:
      case IIDCCamera::RAW_16:
      case IIDCCamera::SIGNED_MONO_16:
      case IIDCCamera::YUV_422:
      case IIDCCamera::YUV_444:
      case IIDCCamera::RGB_24:
	if (camera.getFormat() == IIDCCamera::uintToFormat(param.level))
	    param.setValue(camera.getFrameRate());
	break;
	
      default:
      {
	u_int	val;
	float	fval;
	TU::getFeature(camera, param.level, val, fval);
	  
	if (param.type_info() == typeid(bool))
	    param.setValue(bool(val));
	else if (param.type_info() == typeid(int))
	    param.setValue(int(val));
	else if (param.type_info() == typeid(double))
	    param.setValue(double(fval));
      }
	break;
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

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "iidc_camera");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
        TU::CameraArrayNode<TU::IIDCCameraArray>	node;
        node.run();
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
        return 1;
    }
    catch (...)
    {
	ROS_ERROR_STREAM("Unknown error.");
        return 1;
    }

    return 0;
}
