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
    boost::recursive_mutex::scoped_lock lock(_mutex);

    _groups.emplace_back("Default", "", 0, 0, true, _params);
		    
    ConfigDescription	description_message;
    toMessage(description_message);
    for (const auto& group : _groups)
	description_message.groups.emplace_back(group);

    _parameter_descriptions_pub.publish(description_message);

    _callback = callback;
}

void
ReconfServer::clearCallback()
{
    boost::recursive_mutex::scoped_lock lock(_mutex);

    _callback.clear();
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
ReconfServer::fromMessage(const Config& msg) const
{
    try
    {
	for (const auto& param : _params)
	    param->fromMessage(msg);

	for (const auto& group : _groups)
	    if (group.id == 0)
		group.fromMessage(msg);
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
ReconfServer::toMessage(Config& msg) const
{
    try
    {
	ConfigTools::clear(msg);

	for (const auto& param : _params)
	    param->toMessage(msg);

	for (const auto& group : _groups)
	    if (group.id == 0)
		group.toMessage(msg);
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
ReconfServer::toMessage(ConfigDescription& desc_msg) const
{
    try
    {
	ConfigTools::clear(desc_msg.min);
	ConfigTools::clear(desc_msg.max);
	ConfigTools::clear(desc_msg.dflt);

	for (const auto& param : _params)
	    param->toMessage(desc_msg);

	for (const auto& group : _groups)
	    if (group.id == 0)
	    {
		group.toMessage(desc_msg.min);
		group.toMessage(desc_msg.max);
		group.toMessage(desc_msg.dflt);
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
    boost::recursive_mutex::scoped_lock lock(_mutex);

    const auto	params = _params;	// Keep copy of original params.
    fromMessage(req.config);		// req.config ==> _params

    if (!_callback)
    {
	ROS_DEBUG_STREAM("ReconfServer::reconfCallback(): "
			 << "callback is not set.");
	return;
    }
		    
    try
    {
	_callback(_params, params);

	toServer();
			
	Config	msg;
	toMessage(msg);				// msg <== _params
	_parameter_update_pub.publish(msg);
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
operator <<(std::ostream& out, const ReconfServer::AbstractParam& param)
{
    out << std::boolalpha
	<< '[' << param.name << "]: level=0x"
	<< std::hex << param.level << std::dec << ",val=";
    if (param.type == type_name<bool>())
	out << boost::any_cast<bool>(param.value());
    else if (param.type == type_name<int>())
	out << boost::any_cast<int>(param.value());
    else if (param.type == type_name<double>())
	out << boost::any_cast<double>(param.value());
    else if (param.type == type_name<std::string>())
	out << boost::any_cast<std::string>(param.value());
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

