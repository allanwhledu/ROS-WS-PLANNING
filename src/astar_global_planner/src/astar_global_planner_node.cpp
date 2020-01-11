#include "astar_global_planner.h"
#include "tree.hh"

nav_msgs::OccupancyGrid::ConstPtr mapmsg;
bool isCenterMapGet = false;
int m_height;
int m_width;
int m_resolution;

std::vector<int> vlast_endpoint_x;
std::vector<int> vlast_endpoint_y;

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

    // path planning part.
    AStartFindPath init_planner;
    init_planner.isRootLoop = true;

    ros::param::get("~x_0",init_planner.startpoint_x);
    ros::param::get("~y_0",init_planner.startpoint_y);
    ros::param::get("~x_1",init_planner.endpoint_x);
    ros::param::get("~y_1",init_planner.endpoint_y);




    // wait for mapmsg.
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


    AStartFindPath planner1;
    AStartFindPath planner2;

    ros::Rate r(1.0);
    int loop_count = 1;
    while (ros::ok() && loop_count<5)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        if(loop_count == 1)
        {
            ROS_INFO_STREAM("loop 1");
            init_planner.de_map_Callback(mapmsg);

            init_planner.setTarget();

            if(!init_planner.plan.poses.empty())
            {
                nav_plan.publish(init_planner.plan);
                ROS_INFO_STREAM("Got init_plan_segment.");
            }

            ROS_INFO_STREAM("0, 0: "<<init_planner.m_node[0][0].flag);

            vlast_endpoint_x.push_back(init_planner.plan.poses.back().pose.position.x);
            vlast_endpoint_y.push_back(init_planner.plan.poses.back().pose.position.y);

            ROS_INFO_STREAM("next loop -----------------------");
        }


        if(loop_count ==2 )
//        if(0)
        {

            ROS_INFO_STREAM("loop: "<<loop_count);
            // new planner, wait to add
            ROS_INFO_STREAM("m_width:"<<m_width);


            planner1.isRootLoop = false;
            planner1.startpoint_x = init_planner.startpoint_x;
            planner1.startpoint_y = init_planner.startpoint_y;
            planner1.endpoint_x = init_planner.endpoint_x;
            planner1.endpoint_y = init_planner.endpoint_y;
            planner1.last_endpoint_x = vlast_endpoint_x.back();
            planner1.last_endpoint_y = vlast_endpoint_y.back();

            ROS_INFO_STREAM("m_width:"<<m_width);

            deepCopyMnode(planner1.m_node, m_height, m_width, init_planner.m_node, mapmsg, planner1.openlist, init_planner.openlist, planner1.closelist, init_planner.closelist);


            ROS_INFO_STREAM("planner's endpoint_x:");
            ROS_INFO_STREAM(planner1.endpoint_x);
            ROS_INFO_STREAM("check m_node[][].flag:");
            ROS_INFO_STREAM(planner1.m_node[8][5].flag);
            ROS_INFO_STREAM(planner1.m_node[8][5].flag);


            planner1.setTarget();

            if(!planner1.plan.poses.empty())
            {
                nav_plan.publish(planner1.plan);
                vlast_endpoint_x.push_back(planner1.plan.poses.back().pose.position.x);
                vlast_endpoint_y.push_back(planner1.plan.poses.back().pose.position.y);
                ROS_INFO_STREAM("Got plan_segment.");
            }
            ROS_INFO_STREAM("next loop -----------------------");

//            ROS_INFO_STREAM("check mnode:");
////            init_planner.m_node[4][4].flag = 1;
//            ROS_INFO_STREAM("init_planner.m_node[4][4].flag ="<<init_planner.m_node[4][4].flag);
////            planner1.m_node[4][4].flag = 2;
//            ROS_INFO_STREAM("init_planner.m_node[4][4].flag ="<<init_planner.m_node[4][4].flag);
//            ROS_INFO_STREAM("planner.m_node[4][4].flag ="<<planner1.m_node[4][4].flag);
        }

        if(loop_count ==3 )
//        if(0)
        {
            ROS_INFO_STREAM("loop: "<<loop_count);
            // new planner, wait to add
            ROS_INFO_STREAM("m_width:"<<m_width);

            planner2.isRootLoop = false;
            planner2.startpoint_x = planner1.startpoint_x;
            planner2.startpoint_y = planner1.startpoint_y;
            planner2.endpoint_x = planner1.endpoint_x;
            planner2.endpoint_y = planner1.endpoint_y;
            planner2.last_endpoint_x = vlast_endpoint_x.back();
            planner2.last_endpoint_y = vlast_endpoint_y.back();

            ROS_INFO_STREAM("m_width:"<<m_width);

            deepCopyMnode(planner2.m_node, m_height, m_width, planner1.m_node, mapmsg, planner2.openlist, planner1.openlist, planner2.closelist, planner1.closelist);


            ROS_INFO_STREAM("planner's endpoint_x:");
            ROS_INFO_STREAM(planner2.endpoint_x);
            ROS_INFO_STREAM("check m_node[][].flag:");
            ROS_INFO_STREAM(planner2.m_node[8][5].flag);
            ROS_INFO_STREAM(planner2.m_node[8][5].flag);


            planner2.setTarget();

            if(!planner2.plan.poses.empty())
            {
                nav_plan.publish(planner2.plan);
                vlast_endpoint_x.push_back(planner2.plan.poses.back().pose.position.x);
                vlast_endpoint_y.push_back(planner2.plan.poses.back().pose.position.y);
                ROS_INFO_STREAM("Got plan_segment.");
            }
            ROS_INFO_STREAM("next loop -----------------------");
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

        for(int i=0;i<5;i++)
        {
            r.sleep();
        }
    }

    return 0;
}
