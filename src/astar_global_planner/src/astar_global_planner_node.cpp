#include "astar_global_planner.h"
#include "tree.hh"

//struct pattern
//{
//    int robot1;
//    int robot2;
//};

nav_msgs::OccupancyGrid::ConstPtr mapmsg;
bool iscentermapget = false;

void center_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapmsg = msg;
    iscentermapget = true;
    ROS_INFO_STREAM("get map.");
}


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

    ros::NodeHandle n;

    ros::Subscriber map_sub;
    ros::Subscriber map_sub2;
    ros::Publisher nav_plan;

    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map",1,&center_map_Callback);
    nav_plan = n.advertise<nav_msgs::Path>("astar_path", 1);


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

    AStartFindPath init_planner;

    ros::param::get("~x_0",init_planner.startpoint_x);
    ros::param::get("~y_0",init_planner.startpoint_y);
    ros::param::get("~x_1",init_planner.endpoint_x);
    ros::param::get("~y_1",init_planner.endpoint_y);


    AStartFindPath planner=init_planner;

    ROS_INFO_STREAM("planner's endpoint_x:");
    ROS_INFO_STREAM(planner.endpoint_x);

//    TESTCOPY test1;
//    test1.obj1 = 2;
//
//    TESTCOPY test2;
//    test2 = test1;
//    test2.obj1 = 3;
//
//    ROS_INFO_STREAM("test1.obj1:"<< test1.obj1);

    ros::Rate r(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        if(iscentermapget)
        {
            planner.de_map_Callback(mapmsg);
        }


        ROS_INFO_STREAM("planner.sign_cacul =");
        ROS_INFO_STREAM(planner.sign_cacul);
        if(planner.sign_cacul)
            planner.set_Target();


        if(!planner.plan.poses.empty())
        {
            nav_plan.publish(planner.plan);

            ROS_INFO_STREAM("Got plan_segment.");
            planner.startpoint_x = planner.plan.poses.back().pose.position.x;
            planner.startpoint_y = planner.plan.poses.back().pose.position.y;

            ROS_INFO_STREAM("Clear the tmp_plan.");
            planner.clear_tmpplan();

            planner.endpoint_x++;
            planner.endpoint_y++;
        }
        ROS_INFO_STREAM("at loopend planner.sign_cacul =");
        ROS_INFO_STREAM(planner.sign_cacul);
        r.sleep();
    }
}
