/*!
 *  \file	main.cpp
 */
#include "CameraArrayNode.h"
#include "TU/V4L2CameraArray.h"

namespace TU
{
/************************************************************************
*   class CameraArrayNode<V4L2CameraArray>				*
************************************************************************/
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
