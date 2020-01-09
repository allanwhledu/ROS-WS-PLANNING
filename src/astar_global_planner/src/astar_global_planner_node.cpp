#include "astar_global_planner.h"
#include "tree.hh"

//struct pattern
//{
//    int robot1;
//    int robot2;
//};

nav_msgs::OccupancyGrid::ConstPtr mapmsg;
bool isCenterMapGet = false;
int m_height;
int m_width;
int m_resolution;

void center_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapmsg = msg;
    m_height=msg->info.height;
    m_width=msg->info.width;
    m_resolution=msg->info.resolution;
    isCenterMapGet = true;
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

    // Callback and Publish.
    ros::Subscriber map_sub;
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
    init_planner.isRootLoop = true;

    ros::param::get("~x_0",init_planner.startpoint_x);
    ros::param::get("~y_0",init_planner.startpoint_y);
    ros::param::get("~x_1",init_planner.endpoint_x);
    ros::param::get("~y_1",init_planner.endpoint_y);

    while (ros::ok)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("outloop spin passed.");
        if(isCenterMapGet)
        {
            ROS_INFO_STREAM("center_map got.");
            break;
        }
    }

    ros::Rate r(1.0);
    int loop_count = 1;
    while (ros::ok() && loop_count<3)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        if(loop_count == 1)
        {

            init_planner.de_map_Callback(mapmsg);
            ROS_INFO_STREAM("init_planner.sign_cacul =");
            ROS_INFO_STREAM(init_planner.sign_cacul);
            if(init_planner.sign_cacul)
                init_planner.setTarget();
            if(!init_planner.plan.poses.empty())
            {
                nav_plan.publish(init_planner.plan);
                ROS_INFO_STREAM("Got init_plan_segment.");
            }
            ROS_INFO_STREAM("next loop -----------------------");
        }

        if(loop_count = 2)
        {
            // new planner, wait to add
            AStartFindPath planner=init_planner;
            planner.isRootLoop = false;
//            deepCopyMnode(planner.m_node, m_height, m_width, init_planner.m_node);
            planner.startpoint_x = init_planner.plan.poses.back().pose.position.x;
            planner.startpoint_y = init_planner.plan.poses.back().pose.position.y;
            planner.endpoint_x++;
            planner.endpoint_y++;

            ROS_INFO_STREAM("planner's endpoint_x:");
            ROS_INFO_STREAM(planner.endpoint_x);
            ROS_INFO_STREAM("check m_node[][].flag:");
            ROS_INFO_STREAM(planner.m_node[8][3].flag);

            planner.de_map_Callback(mapmsg);
            ROS_INFO_STREAM("planner.sign_cacul ="<<planner.sign_cacul);
            if(planner.sign_cacul)
                planner.setTarget();
            if(!planner.plan.poses.empty())
            {
                nav_plan.publish(planner.plan);
                ROS_INFO_STREAM("Got plan_segment.");
            }
            ROS_INFO_STREAM("next loop -----------------------");

            ROS_INFO_STREAM("check mnode:");
            init_planner.m_node[4][4].flag = 1;
            ROS_INFO_STREAM("init_planner.m_node[4][4].flag ="<<init_planner.m_node[4][4].flag);
            planner.m_node[4][4].flag = 2;
            ROS_INFO_STREAM("init_planner.m_node[4][4].flag ="<<init_planner.m_node[4][4].flag);
            ROS_INFO_STREAM("planner.m_node[4][4].flag ="<<planner.m_node[4][4].flag);
        }




//        if(!planner.plan.poses.empty())
//        {
//            nav_plan.publish(planner.plan);
//
//            ROS_INFO_STREAM("Got plan_segment.");
//            planner.startpoint_x = planner.plan.poses.back().pose.position.x;
//            planner.startpoint_y = planner.plan.poses.back().pose.position.y;
//
//            ROS_INFO_STREAM("Clear the tmp_plan.");
//            planner.clear_tmpplan();
//
//            planner.endpoint_x++;
//            planner.endpoint_y++;
//        }
//        ROS_INFO_STREAM("at loopend planner.sign_cacul =");
//        ROS_INFO_STREAM(planner.sign_cacul);

        loop_count++;
        r.sleep();
    }

    return 0;
}
