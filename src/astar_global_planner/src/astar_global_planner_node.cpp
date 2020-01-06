#include "astar_global_planner.h"
#include "tree.hh"

struct pattern
{
    int robot1;
    int robot2;
};

struct leaf
{
    int value;
    pattern* pattern_mode;
};

// 节点主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");

    if(testhfile(1))
        ROS_INFO_STREAM("testhfile success!");

    tree<leaf> tr2;
    tree<leaf>::iterator top2, one2;

    pattern p_tmp;
    p_tmp.robot1 = 11;
    p_tmp.robot2 = 22;

    leaf first;
    first.value=32;
    first.pattern_mode = &p_tmp;
    top2=tr2.begin();
    one2=tr2.insert(top2,first);

    cout << one2->pattern_mode->robot1 << endl;

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
