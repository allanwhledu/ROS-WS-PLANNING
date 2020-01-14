#include "multi_robot_astar_planner.h"

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

//    tree<int> tr_test;
//    tree<int>::iterator top = tr_test.begin();
//    tree<int>::iterator int_it;
//    int test_int = 0;
//    int_it = tr_test.append_child(top,test_int);
//    ROS_INFO_STREAM("IT_TREE:"<< (top==tr_test.parent(int_it)));


    multi_robot_astar_planner test;

    std::vector<AStartFindPath*> vec_planner;

    ros::Rate r(1.0);
    int loop_count = 1;
    bool arrived = false;
        while (ros::ok() && arrived == false)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        // test tree.
        test.tr.begin_leaf()->planners.at(0).de_map_Callback(mapmsg);

//        // path planning part.
//        auto init_planner = new AStartFindPath;
//
//
//        if(loop_count == 1)
//        {
//            init_planner->isRootLoop = true;
//            ros::param::get("~x_1",init_planner->endpoint_x);
//            ros::param::get("~y_1",init_planner->endpoint_y);
//            ros::param::get("~x_0",init_planner->startpoint_x);
//            ros::param::get("~y_0",init_planner->startpoint_y);
//        } else
//        {
//            init_planner->isRootLoop = false;
//            ros::param::get("~x_1",init_planner->endpoint_x);
//            ros::param::get("~y_1",init_planner->endpoint_y);
//            init_planner->startpoint_x = vlast_endpoint_x.back();
//            init_planner->startpoint_y = vlast_endpoint_y.back();
//        }
//
//        init_planner->de_map_Callback(mapmsg);
//
//        init_planner->setTarget();
//
//        if(!init_planner->plan.poses.empty())
//        {
//            nav_plan.publish(init_planner->plan);
//            ROS_INFO_STREAM("Got init_plan_segment.");
//        }
//
//        // save the last end of path.
//        vlast_endpoint_x.push_back(init_planner->plan.poses.front().pose.position.x);
//        vlast_endpoint_y.push_back(init_planner->plan.poses.front().pose.position.y);
//
//        // state feed back.
//        arrived = init_planner->arrived;
//
//        // save the last planner class.
//        vec_planner.push_back(init_planner);


        loop_count++;

        for(int i=0;i<3;i++)
        {
            r.sleep();
        }


        ROS_INFO_STREAM("next loop -----------------------");
    }

    return 0;
}
