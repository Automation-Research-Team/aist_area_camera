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
    addGroup(DEFAULT_GROUP, "default", false, true);
}
    
int32_t
ReconfServer::addGroup(int32_t parent, const std::string& name,
		       bool collapse, bool state)
{
    _groups.emplace_back(canonicalName(name), (collapse ? "collapse" : ""),
			 parent, _groups.size(), state);
    return _groups.size() - 1;
}

void
ReconfServer::setCallback(const CallbackType& callback)
{
    std::lock_guard<std::mutex>	lock(_mutex);

    ConfigDescription	config_desc;
    toMessage(config_desc);  // Set and append to min/max/dflt values/groups.
    _parameter_descriptions_pub.publish(config_desc);

    toServer();

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
		  [](auto&& c){ if (c != '/' && !isalnum(c)) c = '_'; });
    return canonical_name;
}
    
void
ReconfServer::fromServer() const
{
    try
    {
	for (const auto& param : _params)
	    param->fromServer(_nh);
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
	config_desc.groups.clear();
	
	for (const auto& param : _params)
	    param->toMessage(config_desc);  // Set min/max/dflt values.

	for (const auto& group : _groups)
	{
	    group.toMessage(config_desc);  // Append to min/max/dflt groups.
	    config_desc.groups.emplace_back(group);
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
	toMessage(rsp.config);				// config <== _params
	_parameter_update_pub.publish(rsp.config);	// publish config
    }
    catch (const std::exception& err)
    {
	ROS_WARN_STREAM("ReconfServer::reconfCallback(): " << err.what());
    }
    catch (...)
    {
	ROS_WARN_STREAM("ReconfServer::reconfCallback(): unknown exception.");
    }
		    
    return true;
}

std::string
ReconfServer::dirName(int32_t id) const
{
    if (id == 0)
	return std::string();
    else
    {
	const auto	group = std::find_if(_groups.begin(), _groups.end(),
					     [id](const auto& group)
					     { return group.id == id; });
	if (group == _groups.end())
	    throw std::runtime_error(
			"ReconfServer::findGroup(): no groups with id=" +
			std::to_string(id) + ".");

	return dirName(group->parent) + group->name + '/';
    }
}
    
/************************************************************************
*  I/O functions							*
************************************************************************/
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

