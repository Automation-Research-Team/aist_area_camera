/*!
 *  \file	ReconfServer.cpp
 */
#include "TU/ReconfServer.h"

namespace TU
{
/************************************************************************
*  class ReconfServer							*
************************************************************************/
ReconfServer::ReconfServer(const ros::NodeHandle& nh)
    :_nh(nh),
     _set_parameters_srv(
	 _nh.advertiseService("set_parameters",
			      &ReconfServer::reconfCallback, this)),
     _parameter_descriptions_pub(
	 _nh.advertise<ConfigDescription>("parameter_descriptions", 1, true)),
     _parameter_update_pub(_nh.advertise<Config>("parameter_updates", 1, true))
{
}
    
void
ReconfServer::setCallback(const CallbackType& callback)
{
    std::lock_guard<std::mutex>	lock(_mutex);

    _groups.emplace_back("Default", "", 0, 0, true, _params);
		    
    ConfigDescription	config_desc;
    toMessage(config_desc);
    for (const auto& group : _groups)
	config_desc.groups.emplace_back(group);

    _parameter_descriptions_pub.publish(config_desc);

    _callback = callback;
}

void
ReconfServer::clearCallback()
{
    std::lock_guard<std::mutex>	lock(_mutex);

    _callback.clear();
}

std::string
ReconfServer::canonicalName(const std::string& name)
{
    auto	canonical_name = name;
    std::for_each(canonical_name.begin(), canonical_name.end(),
		  [](auto&& c){ if (!isalnum(c)) c = '_'; });
    return canonical_name;
}
    
void
ReconfServer::fromServer() const
{
    try
    {
	static bool	setup = false;

	for (const auto& param : _params)
	    param->fromServer(_nh);
		    
	for (const auto& group : _groups)
	    if (!setup && group.id == 0)
		setup = true;
    }
    catch (const std::exception& err)
    {
	ROS_DEBUG_STREAM("ReconfServer::fromServer(): " << err.what());
    }
    catch (...)
    {
	ROS_DEBUG_STREAM("ReconfServer::fromServer(): Unknown error.");
    }
}

void
ReconfServer::toServer() const
{
    try
    {
	for (const auto& param : _params)
	    param->toServer(_nh);
    }
    catch (const std::exception& err)
    {
	ROS_DEBUG_STREAM("ReconfServer::toServer(): " << err.what());
    }
    catch (...)
    {
	ROS_DEBUG_STREAM("ReconfServer::toServer(): Unknown error.");
    }
}

bool
ReconfServer::fromMessage(const Config& config) const
{
    try
    {
	for (const auto& param : _params)
	    param->fromMessage(config);

	for (const auto& group : _groups)
	    if (group.id == 0)
		group.fromMessage(config);
    }
    catch (const std::exception& err)
    {
	ROS_DEBUG_STREAM("ReconfServer::fromMessage(): " << err.what());
    }
    catch (...)
    {
	ROS_DEBUG_STREAM("ReconfServer::fromMessage(): Unknown error.");
    }
}
    
void
ReconfServer::toMessage(Config& config) const
{
    try
    {
	ConfigTools::clear(config);

	for (const auto& param : _params)
	    param->toMessage(config);

	for (const auto& group : _groups)
	    if (group.id == 0)
		group.toMessage(config);
    }
    catch (const std::exception& err)
    {
	ROS_DEBUG_STREAM("ReconfServer::toMessage(): " << err.what());
    }
    catch (...)
    {
	ROS_DEBUG_STREAM("ReconfServer::toMessage(): Unknown error.");
    }
}

void
ReconfServer::toMessage(ConfigDescription& config_desc) const
{
    try
    {
	ConfigTools::clear(config_desc.min);
	ConfigTools::clear(config_desc.max);
	ConfigTools::clear(config_desc.dflt);

	for (const auto& param : _params)
	    param->toMessage(config_desc);

	for (const auto& group : _groups)
	    if (group.id == 0)
	    {
		group.toMessage(config_desc.min);
		group.toMessage(config_desc.max);
		group.toMessage(config_desc.dflt);
	    }
    }
    catch (const std::exception& err)
    {
	ROS_DEBUG_STREAM("ReconfServer::toMessage(): " << err.what());
    }
    catch (...)
    {
	ROS_DEBUG_STREAM("ReconfServer::toMessage(): Unknown error.");
    }
}

bool
ReconfServer::reconfCallback(dynamic_reconfigure::Reconfigure::Request&  req,
			     dynamic_reconfigure::Reconfigure::Response& rsp)
{
    std::lock_guard<std::mutex>	lock(_mutex);

    if (!_callback)
    {
	ROS_DEBUG_STREAM("ReconfServer::reconfCallback(): "
			 << "callback is not set.");
	return;
    }
		    
    const auto	params = _params;	// Keep original params.
    fromMessage(req.config);		// req.config ==> _params

    try
    {
	_callback(_params, params);

	toServer();
			
	Config	config;
	toMessage(config);			// config <== _params
	_parameter_update_pub.publish(config);	// publish config
    }
    catch (const std::exception& err)
    {
	ROS_WARN_STREAM("ReconfServer::reconfCallback(): " << err.what());
    }
    catch (...)
    {
	ROS_WARN_STREAM("ReconfServer::reconfCallback(): unknown exception.");
    }

    toMessage(rsp.config);		// rsp.config <== _params
		    
    return true;
}

std::ostream&
operator <<(std::ostream& out, const ReconfServer::Param& param)
{
    out << std::boolalpha
	<< '[' << param.name << "]: level=0x"
	<< std::hex << param.level << std::dec << ",val=";

    if (param.type_info() == typeid(bool))
	out << param.value<bool>();
    else if (param.type_info() == typeid(int))
	out << param.value<int>();
    else if (param.type_info() == typeid(double))
	out << param.value<double>();
    else if (param.type_info() == typeid(std::string))
	out << param.value<std::string>();
    else
	out << "UNKNOWN";
    return out;
}
    
std::ostream&
operator <<(std::ostream& out, const ReconfServer::Params& params)
{
    for (const auto& param : params)
	out << *param << std::endl;
    return out;
}
    
}	// namespace TU

