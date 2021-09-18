/*!
 *  \file	V4L2CameraArrayNode.cpp
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
    BOOST_FOREACH (auto pixelFormat, camera.availablePixelFormats())
    {
	std::map<std::string, int>	enums;
        int				idx     = 0;
        int				current = 0;
        BOOST_FOREACH (const auto& frameSize,
                       camera.availableFrameSizes(pixelFormat))
        {
	    std::ostringstream	s;
	    s << frameSize;
	    enums.emplace(s.str(), idx);
	    
	    if (camera.pixelFormat() == pixelFormat      &&
		frameSize.width.involves(camera.width()) &&
		frameSize.height.involves(camera.height()))
		current = idx;

	    ++idx;
        }

        _ddr.registerEnumVariable<int>(
	    V4L2Camera::getShortName(pixelFormat), current,
	    boost::bind(&CameraArrayNode::set_feature_cb<int>,
			this, pixelFormat, _1),
	    "Select frame size.", enums);
    }

  // Add feature commands.
    BOOST_FOREACH (auto feature, camera.availableFeatures())
    {
	const auto	name = V4L2Camera::getShortName(feature);
	const auto	menuItems = camera.availableMenuItems(feature);
	
	if (menuItems.first == menuItems.second)
	{
	    int	min, max, step;
	    camera.getMinMaxStep(feature, min, max, step);

	    if (min == 0 && max == 1)	// toglle button
		_ddr.registerVariable<bool>(
		    name, camera.getValue(feature),
		    boost::bind(&CameraArrayNode::set_feature_cb<bool>,
				this, feature, _1),
		    camera.getName(feature));
	    else			// slider
		_ddr.registerVariable<int>(
		    name, camera.getValue(feature),
		    boost::bind(&CameraArrayNode::set_feature_cb<int>,
				this, feature, _1),
		    camera.getName(feature), min, max);
	}
	else				// menu button
	{
	    std::map<std::string, int>	enums;
	    BOOST_FOREACH (const auto& menuItem, menuItems)
	    {
		enums.emplace(menuItem.name, menuItem.index);
	    }

	    _ddr.registerEnumVariable<int>(
		name, camera.getValue(feature),
		boost::bind(&CameraArrayNode::set_feature_cb<int>,
			    this, feature, _1),
		camera.getName(feature), enums);
	}
    }
}

template <> template <class T> void
CameraArrayNode<V4L2CameraArray>::set_feature_cb(int feature, T val)
{
    switch (feature)
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
	if (_n < _cameras.size())
	    TU::setFormat(_cameras[_n], feature, val);
	else
	    for (auto& camera : _cameras)
		TU::setFormat(camera, feature, val);
	break;

      default:
	if (_n < _cameras.size())
	    TU::setFeature(_cameras[_n], feature, val);
	else
	    for (auto& camera : _cameras)
		TU::setFeature(camera, feature, val);
	break;
    }
}

template <> void
CameraArrayNode<V4L2CameraArray>::publish(
    const camera_t& camera, const header_t& header, const cmodel_t& cmodel,
    const image_transport::CameraPublisher& pub) const
{
    using namespace	sensor_msgs;

    switch (camera.pixelFormat())
    {
      case V4L2Camera::BGR24:
	publish<BGR>(camera, header, cmodel, pub, image_encodings::BGR8);
	break;
      case V4L2Camera::RGB24:
	publish<RGB>(camera, header, cmodel, pub, image_encodings::RGB8);
	break;
      case V4L2Camera::BGR32:
	publish<BGRA>(camera, header, cmodel, pub, image_encodings::BGRA8);
	break;
      case V4L2Camera::RGB32:
	publish<ARGB>(camera, header, cmodel, pub, image_encodings::RGBA8);
	break;
      case V4L2Camera::GREY:
	publish<uint8_t>(camera, header, cmodel, pub, image_encodings::MONO8);
	break;
      case V4L2Camera::Y16:
	publish<uint16_t>(camera, header, cmodel, pub,
			  image_encodings::MONO16);
	break;
      case V4L2Camera::YUYV:
	publish<YUYV422>(camera, header, cmodel, pub, image_encodings::YUV422);
	break;
      case V4L2Camera::UYVY:
	publish<YUV422>(camera, header, cmodel, pub, image_encodings::YUV422);
	break;
      case V4L2Camera::SBGGR8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_BGGR8);
	break;
      case V4L2Camera::SGBRG8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_GBRG8);
	break;
      case V4L2Camera::SGRBG8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_GRBG8);
	break;
#ifdef V4L2_PIX_FMT_SRGGB8
      case V4L2Camera::SRGGB8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_RGGB8);
	break;
#endif
      default:
	ROS_WARN_STREAM("Unsupported image encoding!");
	break;
    }
}

}	// namespace TU
