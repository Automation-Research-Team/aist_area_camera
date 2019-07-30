/*!
 *  \file	CameraArrayNode.h
 */
#ifndef TU_ROS_CAMERAARRAYNODE_H
#define TU_ROS_CAMERAARRAYNODE_H

#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "TU/Camera++.h"
#include "TU/ReconfServer.h"
#include "TU/Image++.h"

namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static void
fix_yuyv(void* begin, void* end)
{
}

template <> void
fix_yuyv<YUYV422>(void* begin, void* end)
{
    std::copy(static_cast<const YUYV422*>(begin),
	      static_cast<const YUYV422*>(end), static_cast<YUV422*>(begin));
}

/************************************************************************
*  class CameraArrayNode<CAMERAS>					*
************************************************************************/
template <class CAMERAS>
class CameraArrayNode
{
  private:
    using camera_t = typename CAMERAS::value_type;
    using header_t = std_msgs::Header;
    using image_t  = sensor_msgs::Image;
    using image_p  = sensor_msgs::ImagePtr;
    using cinfo_t  = sensor_msgs::CameraInfo;
    using cinfo_p  = sensor_msgs::CameraInfoPtr;
    using cmodel_t = Camera<IntrinsicWithDistortion<
				 IntrinsicBase<double> > >;

    constexpr static int	CONTINUOUS_SHOT	= 1;
    constexpr static int	SELECT_CAMERA	= 2;

  public:
		CameraArrayNode(const std::string& name)		;

    void	run()							;
    void	tick()							;
    double	rate()						const	;

  private:
    void	add_parameters()					;
    bool	one_shot_callback(std_srvs::Trigger::Request&  req,
				  std_srvs::Trigger::Response& res)	;
    void	reconf_callback(const ReconfServer::Params& new_params,
				const ReconfServer::Params& old_params)	;
    void	publish()					const	;
    void	publish(const camera_t& camera,
			const header_t& header,
			const cmodel_t& cmodel,
			const image_transport::CameraPublisher& pub)
								const	;
    template <class T>
    void	publish(const camera_t& camera,
			const header_t& header,
			const cmodel_t& cmodel,
			const image_transport::CameraPublisher& pub,
			const std::string& encoding)		const	;
    void	set_feature(camera_t& camera,
			    const ReconfServer::Param& param)	const	;
    void	get_feature(const camera_t& camera,
			    ReconfServer::Param& param)		const	;

  private:
    ros::NodeHandle					_nh;
    CAMERAS						_cameras;
    size_t						_n;  // selected camera
    mutable std::vector<uint8_t>			_image;
    image_transport::ImageTransport			_it;
    std::vector<image_transport::CameraPublisher>	_pubs;
    int							_max_skew;
    bool						_convert_to_rgb;
    double						_rate;
    std::vector<cmodel_t>				_cmodels;
    const ros::ServiceServer				_one_shot_server;
    ReconfServer					_reconf_server;
};

template <class CAMERAS>
CameraArrayNode<CAMERAS>::CameraArrayNode(const std::string& name)
    :_nh(name),
     _n(0),
     _it(_nh),
     _max_skew(0),
     _convert_to_rgb(false),
     _rate(100.0),
     _one_shot_server(
	 _nh.advertiseService("one_shot", &one_shot_callback, this)),
     _reconf_server(_nh)
{
  // Restore camera configurations and create cameras.
    std::string	camera_name;
    _nh.param<std::string>("camera_name", camera_name,
			   CAMERAS::DEFAULT_CAMERA_NAME);
    _cameras.setName(camera_name.c_str()).restore();
    if (_cameras.size() == 0)
	throw std::runtime_error("No cameras found.");
    _n = _cameras.size();				// Select all cameras.

  // Set maximum skew allowed for synchronizing multiple cameras.
    _nh.param("max_skew", _max_skew, 0);

  // Set whether converting color formats to RGB or not.
    _nh.param("convert_to_rgb", _convert_to_rgb, false);

  // Set rate.
    _nh.param("rate", _rate, 100.0);

  // Create publishers.
    for (size_t i = 0; i < _cameras.size(); ++i)
    {
	const auto	device_name = "camera" + std::to_string(i);

	_pubs.emplace_back(
	    _it.advertiseCamera((device_name + "/image").c_str(), 1));
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
    _reconf_server.addParam(CONTINUOUS_SHOT,
			    "continuous_shot",
			    "Start/stop streaming images",
			    true);
    if (_cameras.size() > 1)
    {
	ReconfServer::Enums	enums;
	for (size_t i = 0; i < _cameras.size(); ++i)
	    enums.add("camera" + std::to_string(i), i);
	enums.add("all", _cameras.size());
	enums.end();
	_reconf_server.addParam(SELECT_CAMERA,
				"select_camera",
				"Select camera to be controlled.",
				enums, _n);
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
    ros::Rate		looprate(_rate);

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

    if (!_cameras[0].inContinuousShot())
	return;

    if (_max_skew > 0)
	syncedSnap(_cameras, std::chrono::nanoseconds(_max_skew));
    else
	for (auto& camera : _cameras)
	    camera.snap();

    publish();
}

template <class CAMERAS> double
CameraArrayNode<CAMERAS>::rate() const
{
    return _rate;
}

template <class CAMERAS> bool
CameraArrayNode<CAMERAS>::one_shot_callback(std_srvs::Trigger::Request&  req,
					    std_srvs::Trigger::Response& res)
{
    res.success = false;
    res.message = "one_shot is not supported.";
    ROS_WARN_STREAM(res.message);

    return true;
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

	    switch (new_param->level)
	    {
	      case CONTINUOUS_SHOT:
		for (auto& camera : _cameras)
		    camera.continuousShot(new_param->value<bool>());
		break;

	      case SELECT_CAMERA:
		_n = new_param->value<int>();
		refresh = true;
		break;

	      default:
		if (_n < _cameras.size())
		    set_feature(_cameras[_n], *new_param);
		else
		    for (auto& camera : _cameras)
			set_feature(camera, *new_param);
		break;
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
CameraArrayNode<CAMERAS>::publish() const
{
    for (size_t i = 0; i < _cameras.size(); ++i)
    {
	const auto&	camera = _cameras[i];
	const auto	nsec   = camera.getTimestamp().time_since_epoch();
	header_t	header;
	header.stamp	= {nsec.count() / 1000000000,
			   nsec.count() % 1000000000};
	header.frame_id	= "camera" + std::to_string(i);

	publish(camera, header, _cmodels[i], _pubs[i]);
    }
}

template <class CAMERAS> template <class T> void
CameraArrayNode<CAMERAS>::publish(const camera_t& camera,
				  const header_t& header,
				  const cmodel_t& cmodel,
				  const image_transport::CameraPublisher& pub,
				  const std::string& encoding) const
{
    using namespace	sensor_msgs;

    image_p	image(new image_t);
    image->header = header;
    image->height = camera.height();
    image->width  = camera.width();

    try
    {
	if (_convert_to_rgb)
	{
	    if (encoding == image_encodings::BAYER_BGGR8 ||
		encoding == image_encodings::BAYER_GBRG8 ||
#ifdef V4L2_PIX_FMT_SRGGB8
		encoding == image_encodings::BAYER_RGGB8 ||
#endif
		encoding == image_encodings::BAYER_GRBG8)
	    {
		image->encoding = image_encodings::RGB8;
		image->step	= image->width
				*  image_encodings::numChannels(image->encoding)
				* (image_encodings::bitDepth(image->encoding)/8);
		image->data.resize(image->step * image->height);

		camera.captureBayerRaw(image->data.data());
	    }
	    else if (encoding == image_encodings::BGR8  ||
		     encoding == image_encodings::BGRA8 ||
		     encoding == image_encodings::RGBA8 ||
		     encoding == image_encodings::YUV422)
	    {
		image->encoding = image_encodings::RGB8;
		image->step	= image->width
				*  image_encodings::numChannels(image->encoding)
				* (image_encodings::bitDepth(image->encoding)/8);
		image->data.resize(image->step * image->height);

		_image.resize(camera.width() * camera.height() * sizeof(T));
		camera.captureRaw(_image.data());

		constexpr auto	N = iterator_value<
		    pixel_iterator<const T*> >::npixels;
		const auto		npixels = image->width * image->height;
		std::copy_n(make_pixel_iterator(
				reinterpret_cast<const T*>(_image.data())),
			    npixels/N,
			    make_pixel_iterator(
				reinterpret_cast<RGB*>(image->data.data())));
	    }
	    else
	    {
		image->encoding = encoding;
		image->step	= image->width
				*  image_encodings::numChannels(image->encoding)
				* (image_encodings::bitDepth(image->encoding)/8);
		image->data.resize(image->step * image->height);
		camera.captureRaw(image->data.data());
	    }
	}
	else
	{
	    image->encoding = encoding;
	    image->step     = image->width
			    *  image_encodings::numChannels(image->encoding)
			    * (image_encodings::bitDepth(image->encoding)/8);
	    image->data.resize(image->step * image->height);
	    camera.captureRaw(image->data.data());
	    fix_yuyv<T>(image->data.data(),
			image->data.data() + image->data.size());
	}
    }
    catch (const std::exception& err)
    {
	ROS_WARN_STREAM(err.what());
	return;
    }

    cinfo_p	cinfo(new cinfo_t);
    cinfo->header	    = header;
    cinfo->height	    = camera.height();
    cinfo->width	    = camera.width();
    cinfo->distortion_model = "plumb_bob";

    cinfo->D.resize(4);
    cinfo->D[0] = cmodel.d1();
    cinfo->D[1] = cmodel.d2();
    cinfo->D[2] = cinfo->D[3] = 0;

    const auto	K = cmodel.K();
    std::copy(K.data(), K.data() + 9, cinfo->K.data());

    const auto	Rt = cmodel.Rt();
    std::copy(Rt.data(), Rt.data() + 9, cinfo->R.data());

    const auto	P = cmodel.P();
    std::copy(P.data(), P.data() + 12, cinfo->P.data());

    cinfo->binning_x = cinfo->binning_y  = 0;
    cinfo->roi.width = cinfo->roi.height = 0;

    pub.publish(image, cinfo);
}

/************************************************************************
*  class CameraArrayNodelet<CAMERAS>					*
************************************************************************/
template <class CAMERAS>
class CameraArrayNodelet : public nodelet::Nodelet
{
  public:
    CameraArrayNodelet()						{}

    virtual void	onInit()					;
    void		timer_callback(const ros::TimerEvent&)		;

  private:
    ros::NodeHandle					_nh;
    boost::shared_ptr<CameraArrayNode<CAMERAS> >	_node;
    ros::Timer						_timer;
};

template <class CAMERAS> void
CameraArrayNodelet<CAMERAS>::onInit()
{
    NODELET_INFO("CameraArrayNodeklet<CAMERAS>::onInit()");

    _nh = getNodeHandle();
    _node.reset(new CameraArrayNode<CAMERAS>(getName()));
    _timer = _nh.createTimer(ros::Duration(1.0/_node->rate()),
			     &CameraArrayNodelet::timer_callback, this);

}

template <class CAMERAS> void
CameraArrayNodelet<CAMERAS>::timer_callback(const ros::TimerEvent&)
{
    _node->tick();
}

}	// namespace TU
#endif	// !TU_ROS_CAMERAARRAYNODE_H
