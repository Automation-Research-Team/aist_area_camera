/*!
 *  \file	V4L2CameraArrayNode.cpp
 */
#include <boost/foreach.hpp>
#include <aist_area_camera/CameraArrayNode.h>
#include "TU/V4L2CameraArray.h"

namespace aist_area_camera
{
/************************************************************************
*  class CameraArrayNode<TU::V4L2CameraArray>				*
************************************************************************/
template <> void
CameraArrayNode<TU::V4L2CameraArray>::add_parameters()
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
	    camera_t::getShortName(pixelFormat) + "/frame_size", current,
	    boost::bind(&CameraArrayNode::set_feature_cb<int>,
			this, pixelFormat, _1),
	    "Select frame size", enums);
    }

  // Add feature commands.
    BOOST_FOREACH (auto feature, camera.availableFeatures())
    {
	const auto	name = camera_t::getShortName(feature);
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
CameraArrayNode<TU::V4L2CameraArray>::set_feature_cb(int feature, T val)
{
    switch (feature)
    {
      case camera_t::BGR24:
      case camera_t::RGB24:
      case camera_t::BGR32:
      case camera_t::RGB32:
      case camera_t::GREY:
      case camera_t::Y16:
      case camera_t::YUYV:
      case camera_t::UYVY:
      case camera_t::SBGGR8:
      case camera_t::SGBRG8:
      case camera_t::SGRBG8:
#ifdef V4L2_PIX_FMT_SRGGB8
      case camera_t::SRGGB8:
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
CameraArrayNode<TU::V4L2CameraArray>::publish(
    const camera_t& camera, const header_t& header, const cmodel_t& cmodel,
    const image_transport::CameraPublisher& pub) const
{
    using namespace	sensor_msgs;
    
    switch (camera.pixelFormat())
    {
      case camera_t::BGR24:
	publish<TU::BGR>(camera, header, cmodel, pub, image_encodings::BGR8);
	break;
      case camera_t::RGB24:
	publish<TU::RGB>(camera, header, cmodel, pub, image_encodings::RGB8);
	break;
      case camera_t::BGR32:
	publish<TU::BGRA>(camera, header, cmodel, pub, image_encodings::BGRA8);
	break;
      case camera_t::RGB32:
	publish<TU::ARGB>(camera, header, cmodel, pub, image_encodings::RGBA8);
	break;
      case camera_t::GREY:
	publish<uint8_t>(camera, header, cmodel, pub, image_encodings::MONO8);
	break;
      case camera_t::Y16:
	publish<uint16_t>(camera, header, cmodel, pub,
			  image_encodings::MONO16);
	break;
      case camera_t::YUYV:
	publish<TU::YUYV422>(camera, header, cmodel, pub,
			     image_encodings::YUV422);
	break;
      case camera_t::UYVY:
	publish<TU::YUV422>(camera, header, cmodel, pub,
			    image_encodings::YUV422);
	break;
      case camera_t::SBGGR8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_BGGR8);
	break;
      case camera_t::SGBRG8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_GBRG8);
	break;
      case camera_t::SGRBG8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_GRBG8);
	break;
#ifdef V4L2_PIX_FMT_SRGGB8
      case camera_t::SRGGB8:
	publish<uint8_t>(camera, header, cmodel, pub,
			 image_encodings::BAYER_RGGB8);
	break;
#endif
      default:
	ROS_WARN_STREAM("Unsupported image encoding!");
	break;
    }
}

}	// namespace aist_area_camera
