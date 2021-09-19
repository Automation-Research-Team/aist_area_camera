/*!
 *  \file	main.cpp
 */
#include <aist_area_camera/CameraArrayNode.h>
#include "TU/IIDCCameraArray.h"

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "iidc_camera");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle						nh("~");
        aist_area_camera::CameraArrayNode<TU::IIDCCameraArray>	node(nh);
        node.run();
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
