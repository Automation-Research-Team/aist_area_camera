/*!
 *  \file	main.cpp
 */
#include <boost/foreach.hpp>
#include "CameraArrayNode.h"
#include "TU/V4L2CameraArray.h"

namespace TU
{
/************************************************************************
*   class CameraArrayNode<V4L2CameraArray>				*
************************************************************************/
template <> void
CameraArrayNode<V4L2CameraArray>::add_parameters()
{
    const auto&	camera = _cameras[0];

  // Add pixel format commands.
    const auto		pixelFormats = camera.availablePixelFormats();
    std::ostringstream	edit_method;
    bool		init = true;
    
    edit_method << "{\'enum\': [";
    BOOST_FOREACH (auto pixelFormat, pixelFormats)
    {
	if (init)
	    init = false;
	else
	    edit_method << ", ";

	edit_method << "{\'value\': "  << pixelFormat << ", "
		    <<  "\'name\': \'" << camera.getName(pixelFormat) << "\'}";
    }
    edit_method << "]}";
    _reconf_server.addParam(0, "pixel_format", "pixe format",
			    edit_method.str(),
			    std::numeric_limits<int>::min(),
			    std::numeric_limits<int>::max(),
			    int(camera.pixelFormat()));

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
		_reconf_server.addParam(feature, name, name, "", false, true,
					bool(camera.getValue(feature)));
	    else			// slider
		_reconf_server.addParam(feature, name, name, "", min, max,
					camera.getValue(feature));
	}
	else				// menu button
	{
	    std::ostringstream	edit_method;
	    bool		init = true;
	    
	    edit_method << "{\'enum\': [";
	    BOOST_FOREACH (const auto& menuItem, menuItems)
	    {
		if (init)
		    init = false;
		else
		    edit_method << ", ";

		edit_method << "{\'value\': "  << menuItem.index << ", "
			    <<  "\'name\': \'" << menuItem.name  << "\'}";
	    }
	    edit_method << "]}";

	    _reconf_server.addParam(feature, name, name, edit_method.str(),
				    0,
				    int(std::distance(menuItems.first,
						      menuItems.second)) - 1,
				    camera.getValue(feature));
	}
    }
}

template <> void
CameraArrayNode<V4L2CameraArray>::reconf_callback(
    const ReconfServer::Params& params, uint32_t level)
{
    std::cerr << "reconf_callback() called. level=" << level << std::endl;
}
    
template <> void
CameraArrayNode<V4L2CameraArray>::publish_image(
    const camera_t& camera, image_t& image,
    const image_transport::Publisher& pub) const
{
    using namespace	sensor_msgs;

    switch (camera.pixelFormat())
    {
      case V4L2Camera::UYVY:
      case V4L2Camera::YUYV:
	image.encoding = image_encodings::YUV422;
	break;
#ifdef V4L2_PIX_FMT_SRGGB8
      case V4L2Camera::SRGGB8:
	image.encoding = image_encodings::BAYER_RGGB8;
	break;
#endif
      case V4L2Camera::SBGGR8:
	image.encoding = image_encodings::BAYER_BGGR8;
	break;
      case V4L2Camera::SGBRG8:
	image.encoding = image_encodings::BAYER_GBRG8;
	break;
      case V4L2Camera::SGRBG8:
	image.encoding = image_encodings::BAYER_GRBG8;
	break;
      case V4L2Camera::GREY:
	image.encoding = image_encodings::MONO8;
	break;
      case V4L2Camera::Y16:
	image.encoding = image_encodings::MONO16;
	break;
      case V4L2Camera::BGR24:
	image.encoding = image_encodings::BGR8;
	break;
      case V4L2Camera::RGB24:
	image.encoding = image_encodings::RGB8;
	break;
      case V4L2Camera::BGR32:
	image.encoding = image_encodings::BGRA8;
	break;
      case V4L2Camera::RGB32:
	image.encoding = image_encodings::RGBA8;
	break;
      default:
	ROS_WARN_STREAM("Unsupported image encoding!");
	return;
    }

    image.step = image.width
	       *  image_encodings::numChannels(image.encoding)
	       * (image_encodings::bitDepth(image.encoding)/8);
    image.data.resize(image.step * image.height);

    camera.captureRaw(image.data.data());

    if (camera.pixelFormat() == V4L2Camera::YUYV)
	std::copy(reinterpret_cast<const YUYV422*>(image.data.data()),
		  reinterpret_cast<const YUYV422*>(image.data.data() +
						   image.data.size()),
		  reinterpret_cast<YUV422*>(image.data.data()));

    pub.publish(image);
}

}	// namespace TU

/************************************************************************
*   global functions                                                    *
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
