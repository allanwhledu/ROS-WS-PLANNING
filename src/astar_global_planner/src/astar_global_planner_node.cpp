#include "multi_robot_astar_planner.h"

nav_msgs::OccupancyGrid::ConstPtr mapmsg;
bool isCenterMapGet = false;
int m_height, m_width, m_resolution;

int robots_start_end_points[][4] = {
        {1, 1, 7, 1},
//        {7, 1, 1, 3},
        {7, 4, 1, 4},
};
int num_robots = 2;
int layer_depth = 2;

std::vector<int> vlast_endpoint_x, vlast_endpoint_y;

void center_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    mapmsg = msg;
    m_height = msg->info.height;
    m_width = msg->info.width;
    m_resolution = msg->info.resolution;
    isCenterMapGet = true;
    ROS_INFO_STREAM("get map.");
}

struct leaf {
    nav_msgs::Path plan;
    std::vector<int> prior_mode;
};

vector<int> startpoint_x(num_robots), startpoint_y(num_robots), endpoint_x(num_robots), endpoint_y(num_robots);

tree<planner_group>::iterator // 深拷贝
grow_tree(tree<planner_group>::iterator last_leaf, vector <nav_msgs::Path> &null_path, vector<int> &permti) {
    tree<planner_group>::iterator newpg;
    newpg = last_leaf->grow_new_leaf();
//    ROS_INFO_STREAM("newpg got in grow tree.");

    newpg->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
    //TODO: set_planner_group把startpoint给改了，改成currentpoint的位置
    newpg->set_planner_group(num_robots); //TODO 这地方卡住了
    if (newpg->planners.empty())
        ROS_INFO_STREAM("planners init failed.");

    for (int i = 0; i < num_robots; ++i) {
        ROS_INFO_STREAM("# grow tree " << num_robots << " leaf!");
//        ROS_INFO_STREAM("we can access the tr.planner.");
        newpg->planners.at(permti[i])->de_map_Callback(mapmsg);

        newpg->planners.at(permti[i])->setTarget();

        if (!newpg->planners.at(permti[i])->plan.poses.empty()) {
            nav_msgs::Path new_path = newpg->planners.at(permti[i])->plan;
            null_path.push_back(new_path);
            (*newpg).add_feedback_from_path(new_path, permti[i]); //TODO: 是在这里吗？　还是应该在main函数里

            newpg->pathes.push_back(newpg->planners.at(permti[i])->plan);
            newpg->print_tpath();

//            ROS_INFO_STREAM("Got init_plan_segment in grow_tree.");
        }
    }
    return newpg;
}

void print_open_planner_group_vec(vector <tree<planner_group>::iterator> &open_planner_group_vec) {
    for (int i = 0; i < open_planner_group_vec.size(); ++i) {

        ROS_WARN_STREAM("feedback: " << open_planner_group_vec.at(i)->feedback);
    }
}

void sort_open_planner_group_vec(vector <tree<planner_group>::iterator> &open_planner_group_vec) {
    sort(open_planner_group_vec.begin(), open_planner_group_vec.end(), feedback_smaller_than);

//    open_planner_group_vec.sort(open_planner_group_vec.begin().feedback_smaller_than);
    print_open_planner_group_vec(open_planner_group_vec);
}


// 节点主函数
int main(int argc, char **argv) {
    ARR_LEN(robots_start_end_points, num_robots);

    ros::init(argc, argv, "astar_planner");
    vector <vector<int>> permt;
    int robots_idx_lst[num_robots];
    for (int m = 0; m < num_robots; ++m) {
        robots_idx_lst[m] = m;
    }

    multi_robot_astar_planner test;
    vector <tree<planner_group>::iterator> init_pg_locs = test.init_set_multi_robot_astar_planner(
            num_robots);
    bool init_already = false;
    perm(robots_idx_lst, sizeof(robots_idx_lst) / sizeof(robots_idx_lst[0]), permt);

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
//        ROS_INFO_STREAM("outloop spin passed.");
        if (isCenterMapGet) {
//            ROS_INFO_STREAM("center_map got.");
            break;
        }
    }

    ROS_INFO_STREAM("solid multi_robot_planner.");

    std::vector < AStartFindPath * > vec_planner;

    ros::Rate r(1.0);
    int loop_count = 0;
    bool arrived = false;
    while ((ros::ok() && arrived == false) && loop_count < 1) { //TODO
        ros::spinOnce();
        ROS_INFO_STREAM("spin passed.");

        // only init once
        if (!init_already) {
            for (int j = 0; j < permt.size(); ++j) {
                tree<planner_group>::iterator init_planner_group = test.tr->child(test.top, j);
                init_planner_group->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
                init_planner_group->set_planner_group(num_robots);

                init_pg_locs.push_back(init_planner_group);
                for (int idx = 0; idx < num_robots; ++idx) {
                    init_planner_group->planners.at(permt[j][idx])->de_map_Callback(mapmsg);
                    init_planner_group->planners.at(permt[j][idx])->setTarget();
//                if (init_planner_group->planners.empty())
//                    ROS_INFO_STREAM("planners init failed.");
                    if (!init_planner_group->planners.at(permt[j][idx])->plan.poses.empty()) {
                        nav_plans[idx].publish(init_planner_group->planners.at(permt[j][idx])->plan); //TODO check 下标
                        init_planner_group->add_feedback_from_path(init_planner_group->planners.at(permt[j][idx])->plan,
                                                                   permt[j][idx]);
                        init_planner_group->pathes.push_back(init_planner_group->planners.at(permt[j][idx])->plan);
                        init_planner_group->print_tpath();
                        ROS_INFO_STREAM("Got init_plan_segment in main.");
                    }
                }
            }
            init_already = true;
        }
        ROS_INFO_STREAM("INIT DONE");


        for (int j = 0; j < permt.size(); ++j) {// 对每一个init_planner_group做一次树的扩展
            vector <tree<planner_group>::iterator> open_planner_group_vec;
            for (int k = 0; k < permt.size(); ++k) {
                open_planner_group_vec.push_back(init_pg_locs.at(k));
            }
            tree<planner_group>::iterator last_planner_group = init_pg_locs.at(j);
            ROS_WARN_STREAM("find the right init_planner_group.");

            //开始纵深扩展，树已经

            //             top
            //  init1(op0)     init2(op1)

            //接下来准备：
            //             top
            //  init1(op0)     init2(op1)
            //  op2   op3      op4   op5
            // .....(op6,7,8,9)..........
            // ..........................
            // ..........................

            for (int idx = 0; idx < layer_depth; ++idx) {
                //TODO: sort open_planner_group_vec by feedback
                sort_open_planner_group_vec(open_planner_group_vec);
                last_planner_group = open_planner_group_vec.at(0); //last_planner_group is already sorted

                // 从feedback最小的一个pg开始扩展
                vector <nav_msgs::Path> nullpaths;
                for (int i = 0; i < permt.size(); ++i) {
                    open_planner_group_vec.push_back(grow_tree(last_planner_group, nullpaths, permt[i]));
//                    ROS_WARN_STREAM("grow_tree complete.");
                    //每长出一个sub pg，就放入open_planner_group_vec的末尾，并插入到tree
                    test.tr->append_child(last_planner_group, open_planner_group_vec.back());//TODO: ?
                    //新长出来sub pg遍历其planner进行publish
                    bool all_arrived = true;
                    for (int k = 0; k < num_robots; ++k) {
                        nav_plans[k].publish(nullpaths[-1]); //TODO: 应该输出对当前机器人最优的路径　//TODO check 下标
                        AStartFindPath *plan = open_planner_group_vec.back()->planners.at(permt[i][k]);
                        bool end = (plan->poses.back().pose.position.x == endpoint_x &&
                                    plan->poses.back().pose.position.y == endpoint_y);
                        all_arrived &= end;
                    }
                    if (all_arrived) {
                        ROS_WARN_STREAM("ALL ARRIVED AND EXIT");
                        vector <nav_msgs::Path> fullpaths(num_robots); //TODO: init?
                        (*last_planner_group).publish_path(fullpaths, open_planner_group_vec.back(), nav_plans);
                        return 0;
                    }
                }
                ROS_WARN_STREAM(
                        "tree size: " << test.tr->size() << ", tree depth: " << test.tr->depth(last_planner_group));
                ROS_WARN_STREAM(
                        "current node: middle " << idx << ", outer: " << j);
                ROS_WARN_STREAM("还是没有能逃过");

                //取feedback最小的一个pg，试探是否有planner已经到达终点，如果全部到达则发布path信息
                /*
                sort_open_planner_group_vec(open_planner_group_vec);
                last_planner_group = open_planner_group_vec.at(0); //last_planner_group is already sorted
                std::cout << "** smallest feedback: " << last_planner_group->feedback << std::endl;
                bool all_arrived = true;
                bool single_arrived = false;
                for (int idx = 0; idx < num_robots; ++idx) {
                    AStartFindPath *aStartFindPath = last_planner_group->planners.at(permt[j][idx]);
                    all_arrived &= (aStartFindPath->x == aStartFindPath->endpoint_x) &&
                                   (aStartFindPath->y == aStartFindPath->endpoint_y);
                    single_arrived |= (aStartFindPath->x == aStartFindPath->endpoint_x) &&
                                      (aStartFindPath->y == aStartFindPath->endpoint_y);
                }
                if (all_arrived) {
                    ROS_WARN_STREAM("ALL ARRIVED AND EXIT");
                    vector <nav_msgs::Path> fullpaths(num_robots); //TODO: init?
                    (*last_planner_group).publish_path(fullpaths, last_planner_group, nav_plans);
                    return 0;
                } else if (single_arrived) {
                    ROS_WARN_STREAM("SINGLE ARRIVED AND EXIT");
                }*/
            }
            //至此，指定层数的扩展已经进行完毕
        }

        loop_count++;
//
//        for (int i = 0; i < 3; i++) {
//            r.sleep();
//        }

        ROS_INFO_STREAM("next loop -----------------------");
    }

    return 0;
}
