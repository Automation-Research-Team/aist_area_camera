/*!
 *  \file	CameraArrayNode.h
 */
#ifndef TU_ROS_CAMERAARRAYNODE_H
#define TU_ROS_CAMERAARRAYNODE_H

#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "TU/Camera++.h"

namespace TU
{
/************************************************************************
*   class CameraArray<CAMERAS>                                          *
************************************************************************/
template <class CAMERAS>
class CameraArrayNode
{
  private:
    using camera_t = typename CAMERAS::value_type;
    using image_t  = sensor_msgs::Image;
    using cinfo_t  = sensor_msgs::CameraInfo;
    using cmodel_t = Camera<IntrinsicWithDistortion<
				IntrinsicBase<double> > >;

  public:
		CameraArrayNode()					;

    void	run()							;

  private:
    void	publish_image(const camera_t& camera,
			      image_t& image,
			      const image_transport::Publisher& pub)
								const	;
    void	publish_cinfo(const image_t&  image,
			      const cmodel_t& cmodel,
			      const ros::Publisher& pub)	const	;
    void	tick()							;

  private:
    ros::NodeHandle				_nh;
    CAMERAS					_cameras;
    image_transport::ImageTransport		_it;
    std::vector<image_transport::Publisher>	_pubs;
    std::vector<ros::Publisher>			_cinfo_pubs;
    int						_max_skew;
    std::vector<cmodel_t>			_cmodels;
};

template <class CAMERAS>
CameraArrayNode<CAMERAS>::CameraArrayNode()
    :_nh("~"),
     _it(_nh),
     _max_skew(0)
{
  // Restore camera configurations and create cameras.
    std::string	camera_name;
    _nh.param<std::string>("camera_name", camera_name,
			   CAMERAS::DEFAULT_CAMERA_NAME);
    _cameras.setName(camera_name.c_str()).restore();

  // Set maximum skew allowed for synchronizing multiple cameras.
    _nh.param("max_skew", _max_skew, 0);

  // Create publishers.
    for (size_t i = 0; i < _cameras.size(); ++i)
    {
	const auto	device_name = "device" + std::to_string(i);

	_pubs.emplace_back(_it.advertise((device_name + "/image").c_str(), 1));
	_cinfo_pubs.emplace_back(
	    _nh.advertise<cinfo_t>((device_name + "/camera_info").c_str(), 1));
    }

  // Load calibration.
    std::ifstream	in(_cameras.calibFile().c_str());
    _cmodels.resize(_cameras.size());
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
    for (auto& camera : _cameras)
	camera.continuousShot(true);
}

template <class CAMERAS> void
CameraArrayNode<CAMERAS>::run()
{
    ros::Rate looprate(200);	// 200Hz

    while (ros::ok())
    {
	tick();
	ros::spinOnce();
	looprate.sleep();
    }
}

template <class CAMERAS> void
CameraArrayNode<CAMERAS>::tick()
{
    if (_max_skew > 0)
	syncedSnap(_cameras, std::chrono::nanoseconds(_max_skew));
    else
	for (auto& camera : _cameras)
	    camera.snap();

    image_t	image;

    for (size_t i = 0; i < _cameras.size(); ++i)
    {
	const auto&	camera = _cameras[i];
	const auto	nsec = camera.getTimestamp().time_since_epoch();

	image.header.stamp	= {nsec.count() / 1000000000,
				   nsec.count() % 1000000000};
	image.header.frame_id	= "camera" + std::to_string(i);
	image.is_bigendian	= 0;
	image.height		= camera.height();
	image.width		= camera.width();

	publish_image(camera, image, _pubs[i]);
	publish_cinfo(image, _cmodels[i], _cinfo_pubs[i]);
    }
}

template <class CAMERAS> void
CameraArrayNode<CAMERAS>::publish_cinfo(const image_t&  image,
					const cmodel_t& cmodel,
					const ros::Publisher& cinfo_pub) const
{
    cinfo_t	cinfo;
    cinfo.header.stamp		= image.header.stamp;
    cinfo.header.frame_id	= image.header.frame_id;
    cinfo.height		= image.height;
    cinfo.width			= image.width;
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

    cinfo_pub.publish(cinfo);
}

}
#endif	// !TU_ROS_CAMERAARRAYNODE_H
