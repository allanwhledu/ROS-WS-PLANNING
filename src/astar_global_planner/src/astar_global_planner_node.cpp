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



    std::vector<AStartFindPath*> vec_planner;

    ros::Rate r(1.0);
    int loop_count = 1;
    bool arrived = false;
        while (ros::ok() && arrived == false)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        // path planning part.
        auto init_planner = new AStartFindPath;


        if(loop_count == 1)
        {
            init_planner->isRootLoop = true;
            ros::param::get("~x_1",init_planner->endpoint_x);
            ros::param::get("~y_1",init_planner->endpoint_y);
            ros::param::get("~x_0",init_planner->startpoint_x);
            ros::param::get("~y_0",init_planner->startpoint_y);
        } else
        {
            ros::param::get("~x_1",init_planner->endpoint_x);
            ros::param::get("~y_1",init_planner->endpoint_y);
            init_planner->last_endpoint_x = vlast_endpoint_x.back();
            init_planner->last_endpoint_y = vlast_endpoint_y.back();
        }


        ROS_INFO_STREAM("loop 1");
        init_planner->de_map_Callback(mapmsg);

        init_planner->setTarget();

        if(!init_planner->plan.poses.empty())
        {
            nav_plan.publish(init_planner->plan);
            ROS_INFO_STREAM("Got init_plan_segment.");
        }

        ROS_INFO_STREAM("0, 0: "<<init_planner->m_node[0][0].flag);

        vlast_endpoint_x.push_back(init_planner->plan.poses.back().pose.position.x);
        vlast_endpoint_y.push_back(init_planner->plan.poses.back().pose.position.y);

        arrived = init_planner->arrived;

        vec_planner.push_back(init_planner);

        ROS_INFO_STREAM("next loop -----------------------");


//        if(loop_count >1 )
//        {
//
////            AStartFindPath* planner1;
//            auto planner = new AStartFindPath;
//            ROS_INFO_STREAM("loop: "<<loop_count);
//            planner->loop_count = loop_count;
//            planner->isRootLoop = false;
//            planner->startpoint_x = vec_planner.back()->startpoint_x;
//            planner->startpoint_y = vec_planner.back()->startpoint_y;
//            planner->endpoint_x = vec_planner.back()->endpoint_x;
//            planner->endpoint_y = vec_planner.back()->endpoint_y;
//            planner->last_endpoint_x = vlast_endpoint_x.back();
//            planner->last_endpoint_y = vlast_endpoint_y.back();
//
//            ROS_INFO_STREAM("m_width:"<<m_width);
//
//            deepCopyMnode(planner->m_node, m_height, m_width, vec_planner.back()->m_node, mapmsg, planner->openlist, vec_planner.back()->openlist, planner->closelist, vec_planner.back()->closelist);
//
//
//            planner->setTarget();
//
//            if(!planner->plan.poses.empty())
//            {
//                nav_plan.publish(planner->plan);
//                vlast_endpoint_x.push_back(planner->plan.poses.back().pose.position.x);
//                vlast_endpoint_y.push_back(planner->plan.poses.back().pose.position.y);
//                ROS_INFO_STREAM("Got plan_segment.");
//            }
//
//            arrived = planner->arrived;
//
//            vec_planner.push_back(planner);
//
//            ROS_INFO_STREAM("next loop -----------------------");
//        }

        loop_count++;

        for(int i=0;i<3;i++)
        {
            r.sleep();
        }
    }

    return 0;
}
