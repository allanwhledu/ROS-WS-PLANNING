#include "astar_global_planner.h"
#include "tree.hh"

//struct pattern
//{
//    int robot1;
//    int robot2;
//};

struct leaf
{
    int value;
    std::vector<int> *prior_mode;
};

// 节点主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");

    if(testhfile(1))
        ROS_INFO_STREAM("testhfile success!");

    tree<leaf> tr2;

    tree<leaf>::iterator top2, one2;
    tree<leaf>::iterator init_leaf;
    std::vector<tree<leaf>::iterator> testid;
    for(int i=0;i<10;i++){
        testid.push_back(init_leaf);
    }

//    for(int i=0;i<10;i++){
//        string name="leaf";
//        name.append(to_string(i));
//    }

    std::vector<int> p_tmp;
    p_tmp.push_back(11);
    p_tmp.push_back(22);

    leaf first;
    first.value=32;
    first.prior_mode = &p_tmp;
    leaf second;
    second.value=36;
    second.prior_mode =&p_tmp;
    top2=tr2.begin();
    testid.at(0)=tr2.insert(top2,first);
    testid.at(1)=tr2.insert(testid.at(0),second);

    ROS_INFO_STREAM(testid.at(0)->value);
    ROS_INFO_STREAM("and");
    ROS_INFO_STREAM(testid.at(1)->value);

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
