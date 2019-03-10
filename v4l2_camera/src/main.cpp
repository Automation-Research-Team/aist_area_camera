/*!
 *  \file	main.cpp
 */
#include <boost/foreach.hpp>
#include "TU/CameraArrayNode.h"
#include "TU/V4L2CameraArray.h"

namespace TU
{
/************************************************************************
*  class CameraArrayNode<V4L2CameraArray>				*
************************************************************************/
template <> void
CameraArrayNode<V4L2CameraArray>::add_parameters()
{
    const auto&	camera = _cameras[0];

  // Add pixel format commands.
    const auto		pixelFormats = camera.availablePixelFormats();
    
    BOOST_FOREACH (auto pixelFormat, pixelFormats)
    {
	const auto	parent = _reconf_server.addGroup(
					ReconfServer::DEFAULT_GROUP,
					camera.getName(pixelFormat),
					true, true);

	ReconfServer::Enums	enums;
	int			idx     = 0;
	int			current = 0;
	BOOST_FOREACH (const auto& frameSize,
		       camera.availableFrameSizes(pixelFormat))
	{
	    enums.add(frameSize, idx);

	    if (camera.pixelFormat() == pixelFormat	 &&
		frameSize.width.involves(camera.width()) &&
		frameSize.height.involves(camera.height()))
		current = idx;

	    ++idx;
	}
	enums.end();
	
	_reconf_server.addParam<int>(parent, pixelFormat,
				     "Frame Size", "Select frame size.",
				     enums.str(), 0, idx-1, current);
    }

  // Add feature commands.
    BOOST_FOREACH (auto feature, camera.availableFeatures())
    {
	const auto	name	  = camera.getName(feature);
	const auto	menuItems = camera.availableMenuItems(feature);

	if (menuItems.first == menuItems.second)
	{
	    int	min, max, step;
	    camera.getMinMaxStep(feature, min, max, step);

	    if (min == 0 && max == 1)	// toglle button
		_reconf_server.addParam<bool>(ReconfServer::DEFAULT_GROUP,
					      feature,
					      name, name, "", false, true,
					      camera.getValue(feature));
	    else			// slider
		_reconf_server.addParam<int>(ReconfServer::DEFAULT_GROUP,
					     feature,
					     name, name, "", min, max,
					     camera.getValue(feature));
	}
	else				// menu button
	{
	    ReconfServer::Enums	enums;
	    BOOST_FOREACH (const auto& menuItem, menuItems)
	    {
		enums.add(menuItem.name, menuItem.index);
	    }
	    enums.end();

	    _reconf_server.addParam<int>(ReconfServer::DEFAULT_GROUP, feature,
					 name, name, enums.str(),
					 menuItems.first->index,
					 (menuItems.second - 1)->index,
					 camera.getValue(feature));
	}
    }
}

template <> void
CameraArrayNode<V4L2CameraArray>::set_feature(
    camera_t& camera, const ReconfServer::Param& param) const
{
    switch (param.level)
    {
      case V4L2Camera::BGR24:
      case V4L2Camera::RGB24:
      case V4L2Camera::BGR32:
      case V4L2Camera::RGB32:
      case V4L2Camera::GREY:
      case V4L2Camera::Y16:
      case V4L2Camera::YUYV:
      case V4L2Camera::UYVY:
      case V4L2Camera::SBGGR8:
      case V4L2Camera::SGBRG8:
      case V4L2Camera::SGRBG8:
#ifdef V4L2_PIX_FMT_SRGGB8
      case V4L2Camera::SRGGB8:
#endif
	TU::setFormat(camera, param.level, param.value<int>());
	break;
	
      default:
	if (param.type_info() == typeid(bool))
	    TU::setFeature(camera, param.level, param.value<bool>());
	else if (param.type_info() == typeid(int))
	    TU::setFeature(camera, param.level, param.value<int>());
	break;
    }
}
    
template <> void
CameraArrayNode<V4L2CameraArray>::get_feature(
    const camera_t& camera, ReconfServer::Param& param) const
{
    switch (param.level)
    {
      case V4L2Camera::BGR24:
      case V4L2Camera::RGB24:
      case V4L2Camera::BGR32:
      case V4L2Camera::RGB32:
      case V4L2Camera::GREY:
      case V4L2Camera::Y16:
      case V4L2Camera::YUYV:
      case V4L2Camera::UYVY:
      case V4L2Camera::SBGGR8:
      case V4L2Camera::SGBRG8:
      case V4L2Camera::SGRBG8:
#ifdef V4L2_PIX_FMT_SRGGB8
      case V4L2Camera::SRGGB8:
#endif
      {
	const auto pixelFormat = V4L2Camera::uintToPixelFormat(param.level);

	int	idx = 0;
	BOOST_FOREACH (const auto& frameSize,
		       camera.availableFrameSizes(pixelFormat))
	{
	    if (camera.pixelFormat() == pixelFormat	 &&
		frameSize.width.involves(camera.width()) &&
		frameSize.height.involves(camera.height()))
		param.setValue(idx);
	    ++idx;
	}
      }
	break;
	
      default:
	if (param.type_info() == typeid(bool))
	    param.setValue(bool(camera.getValue(
				    V4L2Camera::uintToFeature(param.level))));
	else if (param.type_info() == typeid(int))
	    param.setValue(int(camera.getValue(
				   V4L2Camera::uintToFeature(param.level))));
	break;
    }
}
    
template <> void
CameraArrayNode<V4L2CameraArray>::publish_image(
    const camera_t& camera, const header_t& header,
    const image_transport::Publisher& pub) const
{
    using namespace	sensor_msgs;

    switch (camera.pixelFormat())
    {
      case V4L2Camera::BGR24:
	publish_image<BGR>(camera, header, pub, image_encodings::BGR8);
	break;
      case V4L2Camera::RGB24:
	publish_image<RGB>(camera, header, pub, image_encodings::RGB8);
	break;
      case V4L2Camera::BGR32:
	publish_image<BGRA>(camera, header, pub, image_encodings::BGRA8);
	break;
      case V4L2Camera::RGB32:
	publish_image<ARGB>(camera, header, pub, image_encodings::RGBA8);
	break;
      case V4L2Camera::GREY:
	publish_image<uint8_t>(camera, header, pub, image_encodings::MONO8);
	break;
      case V4L2Camera::Y16:
	publish_image<uint16_t>(camera, header, pub, image_encodings::MONO16);
	break;
      case V4L2Camera::YUYV:
	publish_image<YUYV422>(camera, header, pub, image_encodings::YUV422);
	break;
      case V4L2Camera::UYVY:
	publish_image<YUV422>(camera, header, pub, image_encodings::YUV422);
	break;
      case V4L2Camera::SBGGR8:
	publish_image<uint8_t>(camera, header, pub,
			       image_encodings::BAYER_BGGR8);
	break;
      case V4L2Camera::SGBRG8:
	publish_image<uint8_t>(camera, header, pub,
			       image_encodings::BAYER_GBRG8);
	break;
      case V4L2Camera::SGRBG8:
	publish_image<uint8_t>(camera, header, pub,
			       image_encodings::BAYER_GRBG8);
	break;
#ifdef V4L2_PIX_FMT_SRGGB8
      case V4L2Camera::SRGGB8:
	publish_image<uint8_t>(camera, header, pub,
			       image_encodings::BAYER_RGGB8);
	break;
#endif
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
    ros::init(argc, argv, "v4l2_camera");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
        TU::CameraArrayNode<TU::V4L2CameraArray>	node;
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
