#include "multi_robot_astar_planner.h"

nav_msgs::OccupancyGrid::ConstPtr mapmsg;
bool isCenterMapGet = false;
int m_height;
int m_width;
int m_resolution;

int robots_start_end_points[][4] = {{1, 1, 7, 1},
                                    {7, 1, 1, 1},
};
int num_robots = 2;
int layer_depth = 3;

std::vector<int> vlast_endpoint_x;
std::vector<int> vlast_endpoint_y;

void center_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    mapmsg = msg;
    m_height = msg->info.height;
    m_width = msg->info.width;
    m_resolution = msg->info.resolution;
    isCenterMapGet = true;
    ROS_INFO_STREAM("get map.");
}

string intToString(int v) {
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%u", v);

    string str = buf;
    return str;
}

struct leaf {
    nav_msgs::Path plan;
    std::vector<int> prior_mode;
};

vector<int> startpoint_x(num_robots);
vector<int> startpoint_y(num_robots);
vector<int> endpoint_x(num_robots);
vector<int> endpoint_y(num_robots);

tree<planner_group>::iterator grow_tree(tree<planner_group>::iterator last_leaf, vector <nav_msgs::Path> &null_path) {
    tree<planner_group>::iterator newpg;
    newpg = last_leaf->grow_new_leaf();
    ROS_INFO_STREAM("newpg got.");

    newpg->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
    newpg->set_planner_group(num_robots);
    if (newpg->planners.empty())
        ROS_INFO_STREAM("planners init failed.");


    for (int i = 0; i < num_robots; ++i) {
        ROS_INFO_STREAM("# " << num_robots << " leaf!");
//        newpg->planners.at(i)->isRootLoop = true;
        ROS_INFO_STREAM("we can access the tr.planner.");

        newpg->planners.at(i)->de_map_Callback(mapmsg);

        newpg->planners.at(i)->setTarget();

        if (!newpg->planners.at(i)->plan.poses.empty()) {
            nav_msgs::Path new_path = newpg->planners.at(i)->plan;
            null_path.push_back(new_path);
            (*newpg).add_feedback_from_path(new_path, i); //TODO: 是在这里吗？　还是应该在ｍａｉｎ函数里

            newpg->pathes.push_back(newpg->planners.at(i)->plan);
            newpg->print_tpath();
            ROS_INFO_STREAM("Got init_plan_segment.");
        }
    }

    return newpg;
}

// 节点主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_planner");
    vector <vector<int>> permt;
    int robots_idx_lst[] = {0, 1};

    multi_robot_astar_planner test;
    test.perm(robots_idx_lst, sizeof(robots_idx_lst) / sizeof(robots_idx_lst[0]), permt);

//    ros::param::get("~x_0",startpoint_x);
//    ros::param::get("~y_0",startpoint_y);
//    ros::param::get("~x_1",endpoint_x);
//    ros::param::get("~y_1",endpoint_y);

    for (int k = 0; k < num_robots; ++k) {
        startpoint_x[k] = robots_start_end_points[k][0];
        startpoint_y[k] = robots_start_end_points[k][1];
        endpoint_x[k] = robots_start_end_points[k][2];
        endpoint_y[k] = robots_start_end_points[k][3];
    }

    ros::NodeHandle n;

    // Callback and Publish.
    ros::Subscriber map_sub;
    vector <ros::Publisher> nav_plans;
    for (int l = 0; l < num_robots; ++l) {
        ros::Publisher nav_plan;
        nav_plan = n.advertise<nav_msgs::Path>("astar_path_robot_" + intToString(l), 1);
        nav_plans.push_back(nav_plan);
    }

    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &center_map_Callback);


    // wait for mapmsg.
    while (ros::ok) {
        ros::spinOnce();
        ROS_INFO_STREAM("outloop spin passed.");
        if (isCenterMapGet) {
            ROS_INFO_STREAM("center_map got.");
            break;
        }
    }

    ROS_INFO_STREAM("solid multi_robot_planner.");

    std::vector < AStartFindPath * > vec_planner;

    ros::Rate r(1.0);
    int loop_count = 1;
    bool arrived = false;
    while (ros::ok() && arrived == false && loop_count < 2) { //TODO
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        //test tree.
        tree<planner_group>::iterator init_planner = test.tr->child(test.top, 0);
        init_planner->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
        init_planner->set_planner_group(num_robots);
        for (int j = 0; j < permt.size(); ++j) {
            for (int idx = 0; idx < num_robots; ++idx) {
                init_planner->planners.at(permt[j][idx])->de_map_Callback(mapmsg);
                init_planner->planners.at(permt[j][idx])->setTarget();
                if (init_planner->planners.empty())
                    ROS_INFO_STREAM("planners init failed.");
                if (!init_planner->planners.at(permt[j][idx])->plan.poses.empty()) {
                    nav_plans[idx].publish(init_planner->planners.at(permt[j][idx])->plan); //TODO check 下标
                    init_planner->pathes.push_back(init_planner->planners.at(permt[j][idx])->plan);
                    init_planner->print_tpath();
                    ROS_INFO_STREAM("Got init_plan_segment.");
                }
            }

            vector <tree<planner_group>::iterator> open_planner_group;
            tree<planner_group>::iterator last_planner_group;
            open_planner_group.push_back(init_planner);
            for (int idx = 0; idx < layer_depth; ++idx) {
                //TODO: sort open_planner_group by feedback
                //???
                last_planner_group = open_planner_group[0]; //last_planner_group is already sorted
                // test new leaf.
                vector <nav_msgs::Path> nullpaths;
                for (int i = 0; i < permt.size(); ++i) {
                    open_planner_group.push_back(grow_tree(last_planner_group, nullpaths));
                    test.tr->append_child(init_planner, last_planner_group);
                    for (int k = 0; k < num_robots; ++k) {
                        nav_plans[k].publish(nullpaths[-1]); //TODO: 应该输出对当前机器人最优的路径　//TODO check 下标
                    }
                    ROS_WARN_STREAM(
                            "tree size: " << test.tr->size() << ", tree depth: " << test.tr->depth(last_planner_group));
                    ROS_WARN_STREAM(
                            "current node: inner " << i << "middle " << idx << "outer: " << j);
                }
            }
            for (int idx = 0; idx < num_robots; ++idx) {
                if (last_planner_group->planners.at(permt[j][idx])->arrived) {
                    ROS_WARN_STREAM("ARRIVED AND EXIT");
                    return 0;
                }
            }

            nav_msgs::Path fullpath; //TODO: init?
//            (* last_planner_group).publish_path(fullpath, last_planner_group, nav_plans);
//            /*
            fullpath.header.frame_id = "odom";
            int dep = 0;
            while (last_planner_group->parent_loc != last_planner_group->top_loc) {
                reverse(last_planner_group->pathes.at(0).poses.begin(), last_planner_group->pathes.at(0).poses.end());
                fullpath.poses.insert(fullpath.poses.end(), last_planner_group->pathes.at(0).poses.begin(),
                                      last_planner_group->pathes.at(0).poses.end());
                last_planner_group = last_planner_group->parent_loc;
                dep++;
            }
            reverse(last_planner_group->pathes.at(0).poses.begin(), last_planner_group->pathes.at(0).poses.end());
            fullpath.poses.insert(fullpath.poses.end(), last_planner_group->pathes.at(0).poses.begin(),
                                  last_planner_group->pathes.at(0).poses.end());
//                for(int i = 1; i < last_planner_group->pathes.size(); i++)
//                {
//                    fullpath.poses.insert(fullpath.poses.end(),last_planner_group->pathes.at(0).poses.begin(),last_planner_group->pathes.at(0).poses.end());
//                }

            ROS_INFO_STREAM("dep: " << dep);
            ROS_INFO_STREAM("now we will pub full path...");

            for (auto &pose : fullpath.poses) {
                ROS_INFO_STREAM("points in fullpath: " << pose.pose.position.x << " " << pose.pose.position.y);
            }
            nav_plans[0].publish(fullpath);

//             **/
        }
//
//        loop_count++;
//
//        for (int i = 0; i < 3; i++) {
//            r.sleep();
//        }

        ROS_INFO_STREAM("next loop -----------------------");
    }

    return 0;
}
