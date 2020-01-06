#include "astar_global_planner.h"

// 节点主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");

    if(testhfile(1))
        ROS_INFO_STREAM("testhfile success!");

    AStartFindPath planner;

    ros::param::get("~x_0",planner.startpoint_x);
    ros::param::get("~y_0",planner.startpoint_y);

    ros::param::get("~x_1",planner.endpoint_x);
    ros::param::get("~y_1",planner.endpoint_y);

    planner.sign_cacul = true;
    ros::Rate r(1.0); // 设置回调执行频率（每秒1次）
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
