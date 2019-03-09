/*!
 *  \file	ReconfServer.h
 */
#ifndef TU_ROS_RECONFSERVER_H
#define TU_ROS_RECONFSERVER_H

#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/config_tools.h>
#include <boost/function.hpp>
#include <boost/any.hpp>
#include <mutex>
#include <cctype>

namespace TU
{
/************************************************************************
*  statcic functions							*
************************************************************************/
template <class T>
static std::string	type_name()			;
template <>
inline std::string	type_name<bool>()		{ return "bool"; }
template <>
inline std::string	type_name<int>()		{ return "int"; }
template <>
inline std::string	type_name<double>()		{ return "double"; }
template <>
inline std::string	type_name<std::string>()	{ return "string"; }

/************************************************************************
*  class ReconfServer							*
************************************************************************/
class ReconfServer
{
  public:
    using Config		= dynamic_reconfigure::Config;
    using ConfigDescription	= dynamic_reconfigure::ConfigDescription;
    using ConfigTools		= dynamic_reconfigure::ConfigTools;
    using Any			= boost::any;
    
    class Param : public dynamic_reconfigure::ParamDescription
    {
      public:
			Param(const std::string& name_,
			      const std::string& type_,
			      uint32_t		 level_,
			      const std::string& description_,
			      const std::string& edit_method_)
			{
			    name	= name_;
			    type	= type_;
			    level	= level_;
			    description	= description_;
			    edit_method	= edit_method_;
			}

	virtual Param*	clone()					  const	= 0;
	template <class T>
	T		value()	const
			{
			    return boost::any_cast<T>(val());
			}
	const std::type_info&
			type_info() const
			{
			    return val().type();
			}
	virtual void	setValue(const Any& val)			= 0;
	virtual bool	operator ==(const Param& param)		  const	= 0;
	bool		operator !=(const Param& param) const
			{
			    return !operator ==(param);
			}
	virtual void	fromServer(const ros::NodeHandle& nh)		= 0;
	virtual void	toServer(const ros::NodeHandle& nh)	  const	= 0;
	virtual bool	fromMessage(const Config& config)		= 0;
	virtual void	toMessage(Config& config)		  const	= 0;
	virtual void	toMessage(ConfigDescription& config_desc) const	= 0;

      protected:
	virtual Any	val()					  const	= 0;
    };
    
    template <class T>
    class ConcreteParam final : public Param
    {
      public:
			ConcreteParam(const std::string& name_,
				      const std::string& type_,
				      uint32_t		 level_,
				      const std::string& description_,
				      const std::string& edit_method_,
				      const T& min,  const T& max,
				      const T& dflt, const T& val)
			    :Param(name_, type_, level_,
				   description_, edit_method_),
			    _min(min), _max(max), _dflt(dflt), _val(val)
			{
			    ROS_INFO_STREAM("ConcreteParam[" << name <<
					    "]: type=" << type
					    << std::hex << ",level=0x" << level
					    << std::dec << ",val="<< _val);
			}

	virtual Param*	clone() const
			{
			    return new ConcreteParam(*this);
			}
	
	virtual void	setValue(const Any& val)
			{
			    _val = boost::any_cast<T>(val);
			}

	virtual bool	operator ==(const Param& param) const
			{
			    return _val == param.value<T>();
			}

	virtual void	fromServer(const ros::NodeHandle& nh)
			{
			    nh.getParam(name, _val);
			    clamp();
			}

	virtual void	toServer(const ros::NodeHandle& nh) const
			{
			    nh.setParam(name, _val);
			}

	virtual bool	fromMessage(const Config& config)
			{
			    if (!ConfigTools::getParameter(config, name, _val))
				return false;

			    clamp();
			    return true;
			}

	virtual void	toMessage(Config& config) const
			{
			    ConfigTools::appendParameter(config, name, _val);
			}

	virtual void	toMessage(ConfigDescription& config_desc) const
			{
			    ConfigTools::appendParameter(config_desc.min,
							 name, _min);
			    ConfigTools::appendParameter(config_desc.max,
							 name, _max);
			    ConfigTools::appendParameter(config_desc.dflt,
							 name, _dflt);
			}

      protected:
	virtual Any	val() const
			{
			    return _val;
			}
	
      private:
	void		clamp()
			{
			    if (_val < _min)
				_val = _min;
			    if (_val > _max)
				_val = _max;
			}
	
      private:
	const T		_min;
	const T		_max;
	const T		_dflt;
	T		_val;
    };

    class Params : public std::vector<std::unique_ptr<Param> >
    {
      public:
		Params()					= default;
		Params(const Params& params)
		{
		    for (const auto& param : params)
			emplace_back(param->clone());
		}
	Params&	operator =(const Params& params)
		{
		    if (&params != this)
		    {
			clear();
			for (const auto& param : params)
			    emplace_back(param->clone());
		    }
		    return *this;
		}
		Params(Params&& params)				= default;
	Params&	operator =(Params&& params)			= default;
    };
    
    using CallbackType	= boost::function<void(const Params& new_params,
					       const Params& old_params)>;

  private:
    class Group final : public dynamic_reconfigure::Group
    {
      public:
		Group(const std::string& name_,
		      const std::string& type_,
		      int32_t		 parent_,
		      int32_t		 id_,
		      bool		 state_)
		    :state(state_)
		{
		    name   = name_;
		    type   = type_;
		    parent = parent_;
		    id	   = id_;

		    ROS_INFO_STREAM("GroupDescription["
				    << name << "]: type="
				    << type << ",id=" << id);
		}
	
	bool	fromMessage(const Config& config)
		{
		    return ConfigTools::getGroupState(config, name, *this);
		}

	void	toMessage(Config& config) const
		{
		    ConfigTools::appendGroup(config, name, id, parent, *this);
		}

	void	toMessage(ConfigDescription& config_desc) const
		{
		    toMessage(config_desc.min);
		    toMessage(config_desc.max);
		    toMessage(config_desc.dflt);
		}
	
      public:
	bool	state;
    };

    using Groups	= std::vector<Group>;

  public:
		ReconfServer(const ros::NodeHandle& nh)			;

    int32_t	addGroup(int32_t parent,
			 const std::string& name, bool state)		;

    template <class T>
    void	addParam(int32_t parent, uint32_t level,
			 const std::string& name,
			 const std::string& description,
			 const std::string& edit_method,
			 const T& min, const T& max, const T& dflt)
		{
		    _params.emplace_back(new ConcreteParam<T>(
					     dirName(parent) +
					     canonicalName(name),
					     type_name<T>(),
					     level, description, edit_method,
					     min, max, dflt, dflt));

		    const auto	group = std::find_if(
					    _groups.begin(), _groups.end(),
					    [parent](const auto& group)
					    { return group.id == parent; });
		    group->parameters.emplace_back(*_params.back());
		}

    void	setCallback(const CallbackType& callback)		;
    void	clearCallback()						;

  private:
    static std::string
		canonicalName(const std::string& name)			;
    void	fromServer()					const	;
    void	toServer()					const	;
    bool	fromMessage(const Config& config)		const	;
    void	toMessage(Config& config)			const	;
    void	toMessage(ConfigDescription& config_desc)	const	;
    bool	reconfCallback(
		    dynamic_reconfigure::Reconfigure::Request&  req,
		    dynamic_reconfigure::Reconfigure::Response& rsp)	;
    std::string	dirName(int32_t id)				const	;

  public:
    constexpr static uint32_t	DEFAULT_GROUP = 0;
    
  private:
    ros::NodeHandle		_nh;
    const ros::ServiceServer	_set_parameters_srv;
    const ros::Publisher	_parameter_descriptions_pub;
    const ros::Publisher	_parameter_update_pub;

    Params			_params;
    Groups			_groups;

    CallbackType		_callback;

    mutable std::mutex		_mutex;
};

template <> inline void
ReconfServer::ConcreteParam<std::string>::clamp()			{}
template <> inline void
ReconfServer::ConcreteParam<bool>::clamp()				{}

/************************************************************************
*  I/O functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const ReconfServer::Param& param)	;

std::ostream&
operator <<(std::ostream& out, const ReconfServer::Params& params)	;

}
#endif
