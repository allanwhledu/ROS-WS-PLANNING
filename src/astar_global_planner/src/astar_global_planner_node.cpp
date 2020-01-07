#include "astar_global_planner.h"
#include "tree.hh"

//struct pattern
//{
//    int robot1;
//    int robot2;
//};

struct leaf
{
    nav_msgs::Path plan;
    std::vector<int> prior_mode;
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
    std::vector<tree<leaf>::iterator> init_leafv;
//    for(int i=0;i<10;i++){
//        init_leafv.push_back(init_leaf);
//    }

//    for(int i=0;i<10;i++){
//        string name="leaf";
//        name.append(to_string(i));
//    }

    std::vector<int> p_tmp;
    p_tmp.push_back(11);
    p_tmp.push_back(22);

    leaf first;
    first.prior_mode = p_tmp;
    leaf second;
    second.prior_mode =p_tmp;
    top2=tr2.begin();

    init_leafv.push_back(init_leaf);
    init_leafv.at(0)=tr2.insert(top2,first);
    init_leafv.push_back(init_leaf);
    init_leafv.at(1)=tr2.insert(init_leafv.at(0),second);

    AStartFindPath planner;

    bool firstloop = true;
    ros::Rate r(1.0);
    while (ros::ok())
    {
        if(!planner.plan.poses.empty())
        {

        }

        if(firstloop == true)
        {
            ros::param::get("~x_0",planner.startpoint_x);
            ros::param::get("~y_0",planner.startpoint_y);
            ros::param::get("~x_1",planner.endpoint_x);
            ros::param::get("~y_1",planner.endpoint_y);
            firstloop = false;
        }


        planner.sign_cacul = true;

        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        if(!planner.plan.poses.empty())
        {
            ROS_INFO_STREAM("Got plan_segment.");
            planner.startpoint_x = planner.plan.poses.back().pose.position.x;
            planner.startpoint_y = planner.plan.poses.back().pose.position.y;

            ROS_INFO_STREAM("Clear the tmp_plan.");
            planner.clear_tmpplan();

            planner.endpoint_x++;
            planner.endpoint_y++;
        }

        r.sleep();
    }
}
