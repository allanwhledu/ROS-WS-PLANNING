#include "planning_tree.h"
#include <locale.h>

// this is the main cpp.

nav_msgs::OccupancyGrid::ConstPtr mapmsg;
bool is_map_get = false;
int m_height, m_width, m_resolution;

int robots_start_end_points[][4] = {
        {1, 1, 9, 9},
        {9, 9, 1, 1},
        {9, 5, 3, 3},
        {3, 3, 9, 5},
        {8, 5, 1, 9},
//        {1, 9, 8, 5},
};
int num_robots = 5;

void center_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    mapmsg = msg;
    m_height = msg->info.height;
    m_width = msg->info.width;
    m_resolution = msg->info.resolution;
    is_map_get = true;
    ROS_INFO_STREAM("已获得地图");
}

vector<int> startpoint_x(num_robots), startpoint_y(num_robots), endpoint_x(num_robots), endpoint_y(num_robots);

tree<planning_leaf>::iterator // 深拷贝
grow_tree(tree<planning_leaf>::iterator last_leaf, vector <nav_msgs::Path> &null_path, vector<int> &permti) {
    tree<planning_leaf>::iterator newpg;
    newpg = last_leaf->grow_new_leaf();
    last_leaf->feedback += 10000;

    newpg->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
    //TODO: set_planner_group把startpoint给改了，改成currentpoint的位置
    newpg->set_planning_leaf(num_robots, permti);
    if (newpg->planners.empty())
        ROS_INFO_STREAM("no...规划器组未得到正确填充!");

    for (int i = 0; i < num_robots; ++i) {
        ROS_INFO_STREAM("执行本规划器组中，对应第" << i+1 << "个机器人的规划器!");
        newpg->planners.at(permti[i])->de_map_Callback(mapmsg);
        newpg->planners.at(permti[i])->setTarget();
    }

    newpg->print_tpath();

    // 让pathes还是按照 机器人ID（0，1...） 顺序
    for (int i = 0; i < num_robots; ++i) {
        if (!newpg->planners.at(i)->plan.poses.empty()) {
            nav_msgs::Path new_path = newpg->planners.at(i)->plan;
            null_path.push_back(new_path);
            (*newpg).add_feedback_from_path(newpg->planners.at(i), i);
        }
    }
    for (int i = 0; i < num_robots; ++i) {
        newpg->pathes.push_back(newpg->get_planner_by_robot_ID(i)->plan);
    }
    return newpg;
}

void print_open_planner_group_vec(vector <tree<planning_leaf>::iterator> &open_planner_group_vec) {
    std::cout << "feedback:";
    for (int i = 0; i < open_planner_group_vec.size(); ++i) {
        std::cout << " " << open_planner_group_vec.at(i)->feedback;
    }
    std::cout << endl;
}

void sort_open_planner_group_vec(vector <tree<planning_leaf>::iterator> &open_planner_group_vec) {
    sort(open_planner_group_vec.begin(), open_planner_group_vec.end(), feedback_smaller_than);
    print_open_planner_group_vec(open_planner_group_vec);
}


// 节点主函数
int main(int argc, char **argv) {
    // 设置中文输出的支持
    setlocale( LC_ALL, "" );

    // 初始化节点
    ros::init(argc, argv, "astar_planner");

    // 生成机器人序号的全排列
    vector <vector<int>> permt;
    int robots_idx_lst[num_robots];
    for (int m = 0; m < num_robots; ++m) {
        robots_idx_lst[m] = m;
    }
    perm(robots_idx_lst, sizeof(robots_idx_lst) / sizeof(robots_idx_lst[0]), permt);

    // 实例化一个搜索树,并添加第一层树叶;将第一层所有树叶的迭代器存入init_pl_locs
    planning_tree test;
    vector<tree<planning_leaf>::iterator> init_pl_locs = test.init_set_multi_robot_astar_planner(permt.size());
    bool init_already = false;

    // 将顶部定义的机器人起始点存入到vector形式
    for (int k = 0; k < num_robots; ++k) {
        startpoint_x[k] = robots_start_end_points[k][0];
        startpoint_y[k] = robots_start_end_points[k][1];
        endpoint_x[k] = robots_start_end_points[k][2];
        endpoint_y[k] = robots_start_end_points[k][3];
    }

    ros::NodeHandle n;

    // Callback and Publish.
    // 一个sub,很多个pub,并且pub由数组生成
    ros::Subscriber map_sub;
    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &center_map_Callback);
    vector<ros::Publisher> nav_plans;
    for (int l = 0; l < num_robots; ++l) {
        ros::Publisher nav_plan;
        nav_plan = n.advertise<nav_msgs::Path>("astar_path_robot_" + intToString(l), 10);
        nav_plans.push_back(nav_plan);
    }

    // 等待地图信息的发布
    while (ros::ok) {
        ros::spinOnce();
        if (is_map_get) {
            break;
        }
    }

    ROS_INFO_STREAM("开始实例化规划器组...");

    std::vector <AStartFindPath*> vec_planner; // 其实这里说明指针的星号放在变量类型后面更好

    ros::Rate r(1.0);
    int loop_count = 0;
    bool arrived = false;
    while ((ros::ok() && !arrived) && loop_count < 1) { //TODO
        ros::spinOnce();
        ROS_INFO_STREAM("回调函数已调用1次.");

        // only init once
        if (!init_already) {
            for (int i = 0; i < permt.size(); ++i) {
                // 每一层有permt.size()个leaf,每一个leaf中则有num_robots个规划器
                ROS_WARN_STREAM("------------");
                ROS_WARN_STREAM("设置root层的第" << i + 1 << "个规划器树叶");
                tree<planning_leaf>::iterator root_planning_leaf = test.tr->child(test.top, i);
                root_planning_leaf->get_start_and_goal(startpoint_x, startpoint_y, endpoint_x, endpoint_y);
                //每个规划器树叶中有permt.size()个规划器,这里的i选择了一种优先级序列
                root_planning_leaf->set_planning_leaf(num_robots, permt[i]);

                for (int idx = 0; idx < num_robots; ++idx) {
                    root_planning_leaf->planners.at(permt[i][idx])->de_map_Callback(mapmsg);
                    root_planning_leaf->planners.at(permt[i][idx])->setTarget();
                    if (root_planning_leaf->planners.empty())
                        ROS_INFO_STREAM("规划器并没有添加到当前规划器组中!");
                    if (!root_planning_leaf->planners.at(permt[i][idx])->plan.poses.empty()) {
                        root_planning_leaf->add_feedback_from_path(
                                root_planning_leaf->planners.at(permt[i][idx]),
                                permt[i][idx]);
                    }
                }

                // 将本个leaf所规划出来的路径打印出来，并且存入pathes中
                ROS_INFO_STREAM("本规划器组规划已规划完成，tpath为：");
                root_planning_leaf->print_tpath(); // 一个leaf应该对应规划出来一段路径
                for (int idx = 0; idx < num_robots; ++idx) {
                    root_planning_leaf->pathes.push_back(root_planning_leaf->get_planner_by_robot_ID(idx)->plan);
                }
            }
            init_already = true;
        }
        ROS_INFO_STREAM("root层的规划器组初始化成功.");

        // todo 目前重新看代码看到这里
        vector <tree<planning_leaf>::iterator> open_planner_group_vec;

        for (int k = 0; k < permt.size(); ++k) {
            open_planner_group_vec.push_back(init_pl_locs.at(k));
        }

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
        ROS_WARN_STREAM("搜索树的root层已经搭建完成，接下来开始生长...");
        tree<planning_leaf>::iterator last_planner_group;
        int max_iter_num = 120;
        for (int idx = 0; idx < max_iter_num; ++idx) {
            ROS_WARN_STREAM("正在搜索最优规划器组，本次为第"<< idx+1<<"次尝试.");
            //TODO: sort open_planner_group_vec by feedback
            sort_open_planner_group_vec(open_planner_group_vec);
            last_planner_group = open_planner_group_vec.front();

            ROS_WARN_STREAM("最优规划器组已找到，其feedback值为： "<<last_planner_group->feedback);
            // 从feedback最小的一个pg开始扩展
            ROS_INFO_STREAM("扩展最优规划器中...");
            vector <nav_msgs::Path> nullpaths;
            for (int i = 0; i < permt.size(); ++i) {

                open_planner_group_vec.push_back(grow_tree(last_planner_group, nullpaths, permt[i])); //这里的i控制了优先级序列的生成
                ROS_INFO_STREAM("按照优先级的类型需要生成"<<permt.size()<<"个规划器组，已生成第"<<i+1<<"个规划器组");
                //每长出一个sub pg，就放入open_planner_group_vec的末尾，并插入到tree

                test.tr->append_child(last_planner_group, open_planner_group_vec.back());

                //新长出来sub pg遍历其planner进行publish
                bool all_arrived = true;
                for (int k = 0; k < num_robots; ++k) {

//                    nav_plans[k].publish(nullpaths[-1]); //TODO: 应该输出对当前机器人最优的路径　//TODO check 下标

                    all_arrived &= open_planner_group_vec.back()->planners.at(permt[i][k])->arrived;
                }

                if (all_arrived) {
                    ROS_WARN_STREAM("所有机器人已到达目标位置.");
                    vector <nav_msgs::Path> fullpaths(num_robots);

                    for (int i = 0; i < 100; i++) {
                        (*open_planner_group_vec.back()).publish_path(fullpaths, open_planner_group_vec.back(),
                                                                      nav_plans);
                        r.sleep();
                        r.sleep();
                        r.sleep();
                        r.sleep();
                    }
                    exit(0);
                }
            }
            ROS_WARN_STREAM(
                    "tree depth: " << idx /*test.tr->depth(last_planner_group)*/);

        }
        //至此，指定层数的扩展已经进行完毕

        loop_count++;
        ROS_INFO_STREAM("next loop -----------------------");
    }
    exit(0);
}
