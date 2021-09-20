/*!
 *  \file	CameraArrayNode.h
 */
#ifndef AIST_AREA_CAMERA_CAMERAARRAYNODE_H
#define AIST_AREA_CAMERA_CAMERAARRAYNODE_H

#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "TU/Camera++.h"
#include "TU/Image++.h"

namespace aist_area_camera
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static void
fix_yuyv(void* begin, void* end)
{
}

template <> void
fix_yuyv<TU::YUYV422>(void* begin, void* end)
{
    std::copy(static_cast<const TU::YUYV422*>(begin),
	      static_cast<const TU::YUYV422*>(end),
	      static_cast<TU::YUV422*>(begin));
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
    using cmodel_t = TU::Camera<TU::IntrinsicWithDistortion<
				    TU::IntrinsicBase<double> > >;

  public:
		CameraArrayNode(const ros::NodeHandle& nh)		;

    void	run()							;
    void	tick()							;
    double	rate()						const	;

  private:
    void	embed_timestamp(bool enable)				{}
    void	add_parameters()					;
    bool	one_shot_cb(std_srvs::Trigger::Request&  req,
			    std_srvs::Trigger::Response& res)		;
    void	continuous_shot_cb(bool enable)				;
    template <class T>
    void	set_feature_cb(int feature, T val)			;
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

  private:
    ros::NodeHandle					_nh;
    CAMERAS						_cameras;
    int							_n;  // selected camera
    mutable std::vector<uint8_t>			_image;
    image_transport::ImageTransport			_it;
    std::vector<image_transport::CameraPublisher>	_pubs;
    int							_max_skew;
    bool						_convert_to_rgb;
    double						_rate;
    std::vector<cmodel_t>				_cmodels;
    const ros::ServiceServer				_one_shot_server;
    ddynamic_reconfigure::DDynamicReconfigure		_ddr;
};

template <class CAMERAS>
CameraArrayNode<CAMERAS>::CameraArrayNode(const ros::NodeHandle& nh)
    :_nh(nh),
     _n(0),
     _it(_nh),
     _max_skew(_nh.param("max_skew", 0)),
     _convert_to_rgb(_nh.param("convert_to_rgb", false)),
     _rate(_nh.param("rate", 100.0)),
     _one_shot_server(_nh.advertiseService("one_shot", &one_shot_cb, this)),
     _ddr(_nh)
{
  // Restore camera configurations and create cameras.
    const auto	camera_name = _nh.param<std::string>(
				"camera_name", CAMERAS::DEFAULT_CAMERA_NAME);
    _cameras.setName(camera_name.c_str()).restore();
    if (_cameras.size() == 0)
	throw std::runtime_error("No cameras found.");
    _n = _cameras.size();				// Select all cameras.

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
    _ddr.registerVariable<bool>("continuous_shot", true,
				boost::bind(
				    &CameraArrayNode::continuous_shot_cb,
				    this, _1),
				"Start/stop streaming images", false, true,
				"camera_control");
    if (_cameras.size() > 1)
    {
	std::map<std::string, int>	enums;
	for (size_t i = 0; i < _cameras.size(); ++i)
	    enums.emplace("camera" + std::to_string(i), i);
	enums.emplace("all", _cameras.size());
	_ddr.registerEnumVariable<int>("select_camera", &_n,
				       "Select camera to be activated", enums,
				       "", "camera_control");
    }
    add_parameters();
    _ddr.publishServicesTopics();

  // Embed timestamp in images. (only for PointGrey's IIDC cameras)
    embed_timestamp(true);
    
  // Start cameras.
    ros::Duration(0.5).sleep();
    for (auto& camera : _cameras)
	camera.continuousShot(true);
}

template <class CAMERAS> void
CameraArrayNode<CAMERAS>::run()
{
    ros::Rate	looprate(_rate);

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
CameraArrayNode<CAMERAS>::one_shot_cb(std_srvs::Trigger::Request&  req,
				      std_srvs::Trigger::Response& res)
{
    res.success = false;
    res.message = "one_shot is not supported.";
    ROS_WARN_STREAM(res.message);

    return true;
}

template <class CAMERAS> void
CameraArrayNode<CAMERAS>::continuous_shot_cb(bool enable)
{
    for (auto& camera : _cameras)
	camera.continuousShot(enable);
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
		encoding == image_encodings::BAYER_RGGB8 ||
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

		constexpr auto	N = TU::iterator_value<
		    TU::pixel_iterator<const T*> >::npixels;
		const auto		npixels = image->width * image->height;
		std::copy_n(TU::make_pixel_iterator(
				reinterpret_cast<const T*>(_image.data())),
			    npixels/N,
			    TU::make_pixel_iterator(
				reinterpret_cast<TU::RGB*>(
				    image->data.data())));
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
    void		timer_cb(const ros::TimerEvent&)		;

  private:
    boost::shared_ptr<CameraArrayNode<CAMERAS> >	_node;
    ros::Timer						_timer;
};

template <class CAMERAS> void
CameraArrayNodelet<CAMERAS>::onInit()
{
    NODELET_INFO("CameraArrayNodelet<CAMERAS>::onInit()");

    const auto	nh = getPrivateNodeHandle();
    _node.reset(new CameraArrayNode<CAMERAS>(nh));
    _timer = nh.createTimer(ros::Duration(1.0/_node->rate()),
			    &CameraArrayNodelet::timer_cb, this);

}

template <class CAMERAS> void
CameraArrayNodelet<CAMERAS>::timer_cb(const ros::TimerEvent&)
{
    _node->tick();
}

}	// namespace aist_area_camera
#endif	// !AIST_AREA_CAMERA_CAMERAARRAYNODE_H
