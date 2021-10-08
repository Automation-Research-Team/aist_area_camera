/*!
 *  \file	v4l2_camera.cpp
 */
#include <aist_area_camera/CameraArrayNode.h>
#include "TU/V4L2CameraArray.h"

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "v4l2_camera");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle						nh("~");
        aist_area_camera::CameraArrayNode<TU::V4L2CameraArray>	node(nh);
        node.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << "*** " << err.what() << std::endl;
        return 1;
    }

    return 0;
}
