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
#include "TU/ReconfServer.h"

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

    constexpr static int	SELECT_CAMERA	= 0;
    
  public:
		CameraArrayNode()					;

    void	run()							;

  private:
    void	tick()							;
    void	add_parameters()					;
    void	reconf_callback(const ReconfServer::Params& new_params,
				const ReconfServer::Params& old_params)	;
    void	publish_image(const camera_t& camera,
			      image_t& image,
			      const image_transport::Publisher& pub)
								const	;
    void	publish_cinfo(const image_t&  image,
			      const cmodel_t& cmodel,
			      const ros::Publisher& pub)	const	;
    void	set_feature(camera_t& camera,
			    const ReconfServer::Param& param)	const	;
    void	get_feature(const camera_t& camera,
			    ReconfServer::Param& param)		const	;

  private:
    ros::NodeHandle				_nh;
    CAMERAS					_cameras;
    size_t					_n;	// camera # selected
    image_transport::ImageTransport		_it;
    std::vector<image_transport::Publisher>	_pubs;
    std::vector<ros::Publisher>			_cinfo_pubs;
    int						_max_skew;
    std::vector<cmodel_t>			_cmodels;

    ReconfServer				_reconf_server;
};

template <class CAMERAS>
CameraArrayNode<CAMERAS>::CameraArrayNode()
    :_nh("~"),
     _n(0),
     _it(_nh),
     _max_skew(0),
     _reconf_server(_nh)
{
  // Restore camera configurations and create cameras.
    std::string	camera_name;
    _nh.param<std::string>("camera_name", camera_name,
			   CAMERAS::DEFAULT_CAMERA_NAME);
    _cameras.setName(camera_name.c_str()).restore();
    _n = _cameras.size();				// Select all cameras.

  // Set maximum skew allowed for synchronizing multiple cameras.
    _nh.param("max_skew", _max_skew, 0);

  // Create publishers.
    for (size_t i = 0; i < _cameras.size(); ++i)
    {
	const auto	device_name = "camera" + std::to_string(i);

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

  // Setup dynamic reconfigure
    if (_cameras.size() > 1)
    {
	std::ostringstream	edit_method;
	edit_method << "{\'enum\': [";
	for (size_t i = 0; i < _cameras.size(); ++i)
	{
	    edit_method << "{\'value\': "  << i << ", "
			<<  "\'name\': \'camera" << i << "\'}, ";
	}
	edit_method << "{\'value\': "  << _cameras.size() << ", "
		    <<  "\'name\': \'all\'}]}";
	_reconf_server.addParam(ReconfServer::DEFAULT_GROUP, SELECT_CAMERA,
				"select_camera", "select_camera",
				edit_method.str(),
				0, int(_cameras.size()), int(_n));
    }
    add_parameters();
    _reconf_server.setCallback(boost::bind(&reconf_callback, this, _1, _2));

  // Start cameras.
    ros::Duration(0.5).sleep();
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
CameraArrayNode<CAMERAS>::reconf_callback(
    const ReconfServer::Params& new_params,
    const ReconfServer::Params& old_params)
{
    bool	refresh = false;
    auto	old_param = old_params.begin();
    for (const auto& new_param : new_params)
    {
	if (*new_param != **old_param)
	{
	    ROS_DEBUG_STREAM(*new_param);

	    if (new_param->level == SELECT_CAMERA)
	    {
		_n = new_param->value<int>();
		refresh = true;
	    }
	    else
	    {
		if (_n < _cameras.size())
		    set_feature(_cameras[_n], *new_param);
		else
		    for (auto& camera : _cameras)
			set_feature(camera, *new_param);
	    }
	}

	++old_param;
    }

    if (refresh)
    {
	for (const auto& new_param : new_params)
	    if (new_param->level != SELECT_CAMERA && _n < _cameras.size())
		get_feature(_cameras[_n], *new_param);
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

    cinfo.D.resize(4);
    cinfo.D[0] = cmodel.d1();
    cinfo.D[1] = cmodel.d2();
    cinfo.D[2] = cinfo.D[3] = 0;

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
