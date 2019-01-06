/*!
 *  \file	main.cpp
 */
#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "TU/V4L2CameraArray.h"
#include "TU/Camera++.h"


namespace v4l2_camera
{
/************************************************************************
*   class CameraArray                                                   *
************************************************************************/
class CameraArray : TU::V4L2CameraArray
{
  private:
    using super    = TU::V4L2CameraArray;
    using cinfo_t  = sensor_msgs::CameraInfo;
    using cmodel_t = TU::Camera<TU::IntrinsicWithDistortion<
				    TU::IntrinsicBase<double> > >;

  public:
		CameraArray()						;

    void	run()							;

  private:
    void	tick()							;

  private:
    ros::NodeHandle				_nh;
    image_transport::ImageTransport		_it;
    std::vector<image_transport::Publisher>	_pubs;
    std::vector<ros::Publisher>			_cinfo_pubs;
    int						_max_skew;
    std::vector<cmodel_t>			_cmodels;
};

CameraArray::CameraArray()
    :_nh("~"),
     _it(_nh),
     _max_skew(0)
{
  // Restore camera configurations and create cameras.
    std::string	camera_name;
    _nh.param<std::string>("camera_name", camera_name,
			   TU::V4L2CameraArray::DEFAULT_CAMERA_NAME);
    setName(camera_name.c_str()).restore();

  // Set maximum skew allowed for synchronizing multiple cameras.
    _nh.param("max_skew", _max_skew, 0);

  // Create image publishers.
    for (size_t i = 0; i < size(); ++i)
    {
	const auto	device_name = "device" + std::to_string(i);

	_pubs.emplace_back(_it.advertise((device_name + "/image").c_str(), 1));
	_cinfo_pubs.emplace_back(
	    _nh.advertise<cinfo_t>((device_name + "/camera_info").c_str(), 1));
    }

  // Load calibration.
    std::ifstream	in(calibFile().c_str());
    _cmodels.resize(size());
    for (auto& cmodel : _cmodels)
    {
	if (!in)
	    break;

	cmodel_t::matrix34_type	P;
	cmodel_t::element_type	d1, d2;
	in >> P >> d1 >> d2;

	cmodel.setProjection(P);
	cmodel.setDistortion(d1, d2);
    }

  // Start cameras.
    for (auto& camera : *this)
	camera.continuousShot(true);
}

void
CameraArray::run()
{
    ros::Rate looprate(200);	// 200Hz

    while (ros::ok())
    {
	tick();
	ros::spinOnce();
	looprate.sleep();
    }
}

void
CameraArray::tick()
{
    using namespace	sensor_msgs;

    if (_max_skew > 0)
	syncedSnap(static_cast<super&>(*this),
		   std::chrono::nanoseconds(_max_skew));
    else
	for (auto& camera : *this)
	    camera.snap();

    for (size_t i = 0; i < size(); ++i)
    {
	const auto&	camera = (*this)[i];
	const auto	nsec = camera.getTimestamp().time_since_epoch();
	Image		image;

	image.header.stamp	= {nsec.count() / 1000000000,
				   nsec.count() % 1000000000};
	image.header.frame_id	= "camera" + std::to_string(i);
	image.is_bigendian	= 0;
	image.height		= camera.height();
	image.width		= camera.width();

	switch (camera.pixelFormat())
	{
	  case TU::V4L2Camera::UYVY:
	  case TU::V4L2Camera::YUYV:
	    image.encoding = image_encodings::YUV422;
	    break;
#ifdef V4L2_PIX_FMT_SRGGB8
	  case TU::V4L2Camera::SRGGB8:
	    image.encoding = image_encodings::BAYER_RGGB8;
	    break;
#endif
	  case TU::V4L2Camera::SBGGR8:
	    image.encoding = image_encodings::BAYER_BGGR8;
	    break;
	  case TU::V4L2Camera::SGBRG8:
	    image.encoding = image_encodings::BAYER_GBRG8;
	    break;
	  case TU::V4L2Camera::SGRBG8:
	    image.encoding = image_encodings::BAYER_GRBG8;
	    break;
	  case TU::V4L2Camera::GREY:
	    image.encoding = image_encodings::MONO8;
	    break;
	  case TU::V4L2Camera::Y16:
	    image.encoding = image_encodings::MONO16;
	    break;
	  case TU::V4L2Camera::BGR24:
	    image.encoding = image_encodings::BGR8;
	    break;
	  case TU::V4L2Camera::RGB24:
	    image.encoding = image_encodings::RGB8;
	    break;
	  case TU::V4L2Camera::BGR32:
	    image.encoding = image_encodings::BGRA8;
	    break;
	  case TU::V4L2Camera::RGB32:
	    image.encoding = image_encodings::RGBA8;
	    break;
	  default:
	    ROS_WARN_STREAM("Unsupported image encoding!");
	    continue;
	}

	image.step = image.width
		   *  image_encodings::numChannels(image.encoding)
		   * (image_encodings::bitDepth(image.encoding)/8);
	image.data.resize(image.step * image.height);

	camera.captureRaw(image.data.data());

	if (camera.pixelFormat() == TU::V4L2Camera::YUYV)
	    std::copy(reinterpret_cast<const TU::YUYV422*>(image.data.data()),
		      reinterpret_cast<const TU::YUYV422*>(image.data.data() +
							   image.data.size()),
		      reinterpret_cast<TU::YUV422*>(image.data.data()));

	_pubs[i].publish(image);

      // Publish camera info.
	const auto&	cmodel = _cmodels[i];
	cinfo_t		cinfo;
	cinfo.header.stamp	= image.header.stamp;
	cinfo.header.frame_id	= image.header.frame_id;
	cinfo.height		= camera.height();
	cinfo.width		= camera.width();
	cinfo.distortion_model	= "plumb_bob";

	cinfo.D.resize(2);
	cinfo.D[0] = cmodel.d1();
	cinfo.D[1] = cmodel.d2();

	const auto	K = cmodel.K();
	std::copy(K.data(), K.data() + 9, cinfo.K.data());

	const auto	Rt = cmodel.Rt();
	std::copy(Rt.data(), Rt.data() + 9, cinfo.R.data());

	const auto	P = cmodel.P();
	std::copy(P.data(), P.data() + 12, cinfo.P.data());

	cinfo.binning_x = cinfo.binning_y  = 0;
	cinfo.roi.width = cinfo.roi.height = 0;

	_cinfo_pubs[i].publish(cinfo);
    }
}

}	// namespace v4l2_camera

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
        v4l2_camera::CameraArray cameras;
        cameras.run();
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
