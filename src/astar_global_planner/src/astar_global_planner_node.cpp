#include "astar_global_planner.h"

// 节点主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");

    if(testhfile(1))
        ROS_INFO_STREAM("testhfile success!");

    AStartFindPath planner;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
