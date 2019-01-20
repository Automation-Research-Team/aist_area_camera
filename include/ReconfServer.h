/*!
 *  \file	ReconfServer.h
 */
#ifndef __RECONFSERVER_H__
#define __RECONFSERVER_H__

#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/config_tools.h>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/any.hpp>
#include <cctype>

namespace TU
{
/************************************************************************
*  statcic functions							*
************************************************************************/
template <class T>
static std::string	type_name()			;
template <>
std::string		type_name<bool>()		{ return "bool"; }
template <>
std::string		type_name<int>()		{ return "int"; }
template <>
std::string		type_name<double>()		{ return "double"; }
template <>
std::string		type_name<std::string>()	{ return "string"; }

/************************************************************************
*  class ReconfServere							*
************************************************************************/
class ReconfServer
{
  public:
    using Param			= boost::any;
    using Params		= std::vector<Param>;
    
  private:
    using ConfigDescription	= dynamic_reconfigure::ConfigDescription;
    using ConfigTools		= dynamic_reconfigure::ConfigTools;
    
    class AbstractParamDescription
	: public dynamic_reconfigure::ParamDescription
    {
      public:
			AbstractParamDescription(
			    uint32_t		id_,
			    const std::string&	name_,
			    const std::string&	type_,
			    uint32_t		level_,
			    const std::string&	description_,
			    const std::string&	edit_method_)
			    :id(id_)
	    {
			    name	= name_;
			    type	= type_;
			    level	= level_;
			    description	= description_;
			    edit_method	= edit_method_;
			}

	virtual void	clamp(Param& param,
			      const Param& min, const Param& max) const	= 0;
	virtual uint32_t
			calcLevel(uint32_t comb_level,
				  const Param& param1,
				  const Param& param2)		  const	= 0;
	virtual void	fromServer(const ros::NodeHandle& nh,
				   Param& param)		  const	= 0;
	virtual void	toServer(const ros::NodeHandle& nh,
				 const Param& param)		  const	= 0;
	virtual bool	fromMessage(const dynamic_reconfigure::Config& msg,
				    Param& param)		  const	= 0;
	virtual void	toMessage(dynamic_reconfigure::Config& msg,
				  const Param& param)		  const	= 0;

      public:
	const uint32_t	id;
    };
    
    using ParamDescriptions
	    = std::vector<boost::shared_ptr<const AbstractParamDescription> >;

    template <class T>
    class ParamDescription final : public AbstractParamDescription
    {
      public:
			ParamDescription(uint32_t	    id_,
					 const std::string& name_,
					 const std::string& type_,
					 uint32_t	    level_,
					 const std::string& description_,
					 const std::string& edit_method_)
			    :AbstractParamDescription(id_,
						      name_, type_, level_,
						      description_,
						      edit_method_)
			{
			    ROS_INFO_STREAM("ParamDescription[" << name <<
					    "]: type=" << type
					    << ",level=" << level);
			}

	virtual void	clamp(Param& param,
			      const Param& min, const Param& max) const
			{
			    auto	val    = boost::any_cast<T&>(param);
			    const auto	minval = boost::any_cast<T>(min);
			    if (val < minval)
				val = minval;

			    const auto	maxval = boost::any_cast<T>(max);
			    if (val > maxval)
				val = maxval;
			}

	virtual uint32_t
			calcLevel(uint32_t comb_level,
				  const Param& param1,
				  const Param& param2) const
			{
			    if (boost::any_cast<T>(param1) !=
				boost::any_cast<T>(param2))
				return comb_level | level;
			    else
				return comb_level;
			}

	virtual void	fromServer(const ros::NodeHandle& nh,
				   Param& param) const
			{
			    nh.getParam(name, boost::any_cast<T&>(param));
			}

	virtual void	toServer(const ros::NodeHandle& nh,
				 const Param& param) const
			{
			    nh.setParam(name,
					boost::any_cast<const T&>(param));
			}

	virtual bool	fromMessage(const dynamic_reconfigure::Config& msg,
				    Param& param) const
			{
			    return ConfigTools::getParameter(
					msg, name, boost::any_cast<T&>(param));
			}

	virtual void	toMessage(dynamic_reconfigure::Config& msg,
				  const Param& param) const
			{
			    ConfigTools::appendParameter(
				msg, name, boost::any_cast<const T&>(param));
			}
    };

    class Group final : public dynamic_reconfigure::Group
    {
      public:
		Group(const std::string&	name_,
		      const std::string&	type_,
		      int			parent_,
		      int			id_,
		      bool			state_,
		      const ParamDescriptions&	param_descriptions_)
		    :state(state_)
		{
		    name   = name_;
		    type   = type_;
		    parent = parent_;
		    id	   = id_;

		    for (auto p : param_descriptions_)
			parameters.emplace_back(*p);

		    ROS_INFO_STREAM("GroupDescription["
				    << name << "]: type="
				    << type << ",id=" << id);
		}
	
	bool	fromMessage(const dynamic_reconfigure::Config& msg)
		{
		    return ConfigTools::getGroupState(msg, name, *this);
		}

	void	toMessage(dynamic_reconfigure::Config& msg) const
		{
		    ConfigTools::appendGroup(msg, name, id, parent, *this);
		}
	
      public:
	bool	state;
    };

    using Groups	= std::vector<boost::shared_ptr<const Group> >;
    using CallbackType	= boost::function<void(const Params&, uint32_t level)>;

  public:
		ReconfServer(const ros::NodeHandle& nh = ros::NodeHandle("~"))
		    :_nh(nh),
		     _set_parameters_srv(
			 _nh.advertiseService(
			     "set_parameters",
			     &ReconfServer::setConfigCallback, this)),
		     _parameter_descriptions_pub(
			 _nh.advertise<dynamic_reconfigure::ConfigDescription>(
			     "parameter_descriptions", 1, true)),
		     _parameter_update_pub(
			 _nh.advertise<dynamic_reconfigure::Config>(
			     "parameter_updates", 1, true))
		{
		}

    template <class T>
    void	addParam(uint32_t id,
			 const std::string& name,
			 const std::string& description,
			 const std::string& edit_method,
			 const T& min, const T& max, const T& dflt)
		{
		    auto	canonical_name = name;
		    std::for_each(canonical_name.begin(), canonical_name.end(),
				  [](auto&& c)
				  {
				      if (!isalnum(c))
					  c = '_';
				  });

		    _param_descriptions.emplace_back(
			new ParamDescription<T>(
			    id, canonical_name, type_name<T>(),
			    1 << _param_descriptions.size(),
			    description, edit_method));
		    _min.   emplace_back(min);
		    _max.   emplace_back(max);
		    _dflt.  emplace_back(dflt);
		}

    void	setCallback(const CallbackType& callback)
		{
		    boost::recursive_mutex::scoped_lock lock(_mutex);

		    _groups.emplace_back(new Group("Default", "", 0, 0, true,
						   _param_descriptions));
		    
		  // Copy over _min _max _default and publish.
		    ConfigDescription	description_message;
		    toMessage(description_message.min,  _min);
		    toMessage(description_message.max,  _max);
		    toMessage(description_message.dflt, _dflt);
		    for (auto g : _groups)
			description_message.groups.emplace_back(*g);

		    _parameter_descriptions_pub.publish(description_message);

		    _callback = callback;

		  // At startup we need to load the configuration with all
		  // level bits set. (Everything has changed.)
		    _params = _dflt;
		    callCallback(_params, ~0);
		}

    void	clearCallback()
		{
		    boost::recursive_mutex::scoped_lock lock(_mutex);

		    _callback.clear();
		}

  private:
    void	callCallback(const Params& params, int level)
		{
		    if (_callback)
			try
			{
			    _callback(params, level);

			    toServer(params);

			    dynamic_reconfigure::Config	msg;
			    toMessage(msg, params);
			    _parameter_update_pub.publish(msg);
			}
			catch (const std::exception& e)
			{
			    ROS_WARN_STREAM("ReconfServer::callCallback(): "
					    << e.what());
			}
			catch (...)
			{
			    ROS_WARN_STREAM("ReconfServer::callCallback(): "
					    << "unknown exception.");
			}
		    else
			ROS_DEBUG_STREAM("ReconfServer::callCallback(): "
					 << "callback is not set.");
		}

    void	clamp(Params& params) const
		{
		    auto	param = params.begin();
		    auto	min   = _min.cbegin();
		    auto	max   = _max.cbegin();
		    for (auto p : _param_descriptions)
			p->clamp(*param++, *min++, *max++);
		}

    uint32_t	calcLevel(const Params& params) const
		{
		    uint32_t	level = 0;
		    auto	param1 = params.begin();
		    auto	param2 = _params.begin();
		    for (auto p : _param_descriptions)
			level = p->calcLevel(level, *param1++, *param2++);

		    return level;
		}

    void	fromServer(Params& params) const
		{
		    try
		    {
			static bool	setup = false;

			auto	param = params.begin();
			for (auto p : _param_descriptions)
			    p->fromServer(_nh, *param++);
		    
			for (auto g : _groups)
			    if (!setup && g->id == 0)
			    {
				setup = true;
			      //g->setInitialState(*this);
			    }
		    }
		    catch (const std::exception& err)
		    {
			ROS_DEBUG_STREAM("ReconfServer::fromServer(): "
					 << err.what());
		    }
		    catch (...)
		    {
			ROS_DEBUG_STREAM("ReconfServer::fromServer(): "
					 << "Unknown error.");
		    }
		}

    void	toServer(const Params& params) const
		{
		    try
		    {
			auto	param = params.begin();
			for (auto p : _param_descriptions)
			    p->toServer(_nh, *param++);
		    }
		    catch (const std::exception& err)
		    {
			ROS_DEBUG_STREAM("ReconfServer::toServer(): "
					 << err.what());
		    }
		    catch (...)
		    {
			ROS_DEBUG_STREAM("ReconfServer::toServer(): "
					 << "Unknown error.");
		    }
		}

    bool	fromMessage(dynamic_reconfigure::Config& msg,
			    Params& params) const
		{
		    try
		    {
			auto	param = params.begin();
			for (auto p : _param_descriptions)
			    p->fromMessage(msg, *param++);

			for (auto g : _groups)
			    if (g->id == 0)
				g->fromMessage(msg);
		    }
		    catch (const std::exception& err)
		    {
			ROS_DEBUG_STREAM("ReconfServer::fromMessage(): "
					 << err.what());
		    }
		    catch (...)
		    {
			ROS_DEBUG_STREAM("ReconfServer::fromMessage(): "
					 << "Unknown error.");
		    }
		}
    
    void	toMessage(dynamic_reconfigure::Config& msg,
			  const Params& params) const
		{
		    try
		    {
			ConfigTools::clear(msg);

			auto	param = params.begin();
			for (auto p : _param_descriptions)
			    p->toMessage(msg, *param++);

			for (auto g : _groups)
			    if (g->id == 0)
				g->toMessage(msg);
		    }
		    catch (const std::exception& err)
		    {
			ROS_DEBUG_STREAM("ReconfServer::toMessage(): "
					 << err.what());
		    }
		    catch (...)
		    {
			ROS_DEBUG_STREAM("ReconfServer::toMessage(): "
					 << "Unknown error.");
		    }
		}

    bool	setConfigCallback(
		    dynamic_reconfigure::Reconfigure::Request&  req,
		    dynamic_reconfigure::Reconfigure::Response& rsp)
		{
		    boost::recursive_mutex::scoped_lock lock(_mutex);

		    ROS_DEBUG_STREAM("setConfigCallback() called.");

		    auto	params = _params;
		    fromMessage(req.config, params);
		    clamp(params);
		    
		    callCallback(params, calcLevel(params));

		    toMessage(rsp.config, params);

		    _params = params;
		    
		    return true;
		}

  private:
    ros::NodeHandle		_nh;
    const ros::ServiceServer	_set_parameters_srv;
    const ros::Publisher	_parameter_descriptions_pub;
    const ros::Publisher	_parameter_update_pub;

    ParamDescriptions		_param_descriptions;
    Groups			_groups;

    Params			_min;
    Params			_max;
    Params			_dflt;
    Params			_params;

    CallbackType		_callback;

    boost::recursive_mutex	_mutex;
};

}
#endif

