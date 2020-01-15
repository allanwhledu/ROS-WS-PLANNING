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

vector<int> startpoint_x(2);
vector<int> startpoint_y(2);
vector<int> endpoint_x(2);
vector<int> endpoint_y(2);

tree<planner_group>::iterator grow_tree(tree<planner_group>::iterator last_leaf, nav_msgs::Path& null_path)
{
    tree<planner_group>::iterator newpg;
    newpg = last_leaf->grow_new_leaf();
    ROS_INFO_STREAM("newpg got.");

    newpg->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
    newpg->set_planner_group();
    if(newpg->planners.empty())
        ROS_INFO_STREAM("planners init failed.");

    ROS_INFO_STREAM("first leaf!");
    newpg->planners.at(0)->isRootLoop = true;
    ROS_INFO_STREAM("we can access the tr.planner.");

    newpg->planners.at(0)->de_map_Callback(mapmsg);

    newpg->planners.at(0)->setTarget();

    if(!newpg->planners.at(0)->plan.poses.empty())
    {
        null_path = newpg->planners.at(0)->plan;
        newpg->path = newpg->planners.at(0)->plan;
        ROS_INFO_STREAM("Got init_plan_segment.");
    }

    return newpg;
}


// 节点主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");

//    ros::param::get("~x_0",startpoint_x);
//    ros::param::get("~y_0",startpoint_y);
//    ros::param::get("~x_1",endpoint_x);
//    ros::param::get("~y_1",endpoint_y);
    startpoint_x[0] = 1;
    startpoint_y[0] = 1;
    endpoint_x[0] = 7;
    endpoint_y[0] = 1;

    startpoint_x[1] = 7;
    startpoint_y[1] = 1;
    endpoint_x[1] = 1;
    endpoint_y[1] = 1;

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
    ROS_INFO_STREAM("solid multi_robot_planner.");

    std::vector<AStartFindPath*> vec_planner;

    ros::Rate r(1.0);
    int loop_count = 1;
    bool arrived = false;
        while (ros::ok() && arrived == false && loop_count<2)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        //test tree.
        tree<planner_group>::iterator init_planner = test.tr->child(test.top, 0);
        init_planner->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
        init_planner->set_planner_group();
        if(init_planner->planners.empty())
            ROS_INFO_STREAM("planners init failed.");

        ROS_INFO_STREAM("first robot!");
        ROS_INFO_STREAM("we can access the tr.planner.");
        init_planner->planners.at(0)->de_map_Callback(mapmsg);

        init_planner->planners.at(0)->setTarget();

        if(!init_planner->planners.at(0)->plan.poses.empty())
        {
            nav_plan.publish(init_planner->planners.at(0)->plan);
            init_planner->path = init_planner->planners.at(0)->plan;
            init_planner->print_tpath();
            ROS_INFO_STREAM("Got init_plan_segment.");
        }
//        if(init_planner->planners.at(0)->arrived)
//            return 0;

        // test vertical leaf.
        ROS_INFO_STREAM("debug1");
        ROS_INFO_STREAM("debug2");
        init_planner->set_planner_group();
        ROS_INFO_STREAM("debug3");
        if(init_planner->planners.empty())
            ROS_INFO_STREAM("planners init failed.");

        ROS_INFO_STREAM("second leaf!");
        ROS_INFO_STREAM("we can access the tr.planner.");

        init_planner->planners.at(1)->de_map_Callback(mapmsg);
        ROS_INFO_STREAM("debug4");

        init_planner->planners.at(1)->setTarget();
        ROS_INFO_STREAM("debug5");

        if(!init_planner->planners.at(1)->plan.poses.empty())
        {
            nav_plan.publish(init_planner->planners.at(1)->plan);
            init_planner->path = init_planner->planners.at(1)->plan;
            init_planner->print_tpath();
            ROS_INFO_STREAM("Got init_plan_segment.");
        }
//        if(init_planner->planners.at(1)->arrived)
//            return 0;




        // test new leaf.
//        nav_msgs::Path nullpath;
//        tree<planner_group>::iterator planner = grow_tree(init_planner, nullpath);
//        nav_plan.publish(nullpath);
//        if(planner->planners.at(0)->arrived)
//            return 0;
//
//        nav_msgs::Path nullpath2;
//        tree<planner_group>::iterator planner2 = grow_tree(planner, nullpath2);
//        nav_plan.publish(nullpath2);
//        if(planner2->planners.at(0)->arrived)
//            return 0;


        loop_count++;

        for(int i=0;i<3;i++)
        {
            r.sleep();
        }


        ROS_INFO_STREAM("next loop -----------------------");
    }

    return 0;
}
