//
// Created by 李云帆 on 2020-01-14.
//



#ifndef ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
#define ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H

#include "astar_global_planner.h"
#include "utils.h"
#include <string>

// this class is for construct group-searching.

using namespace std;

#define ARR_LEN(array, length){ length =  sizeof(array) / sizeof(array[0]); }

struct Tpoint {
    int robot_id;
    int x = 0;
    int y = 0;
    int t = 0;
};

class planning_leaf {
public:
    vector<int> startpoint_x;
    vector<int> startpoint_y;
    vector<int> endpoint_x;
    vector<int> endpoint_y;

    vector<AStartFindPath*> planners;

    int feedback = 9999;
    bool noPath = false;
    bool published = false;
    vector <Tpoint> tpath;

    vector <nav_msgs::Path> pathes;


    tree<planning_leaf> *tree_ptr = nullptr;
    tree<planning_leaf>::iterator top_loc;
    tree<planning_leaf>::iterator parent_loc;
    tree<planning_leaf>::iterator self_loc;
    tree<planning_leaf>::iterator child_loc;

    void add_feedback_from_path(AStartFindPath *planner, int idx) {
        if (feedback == 9999) { feedback = 0; }
        if (planner->noPath == true) {
            feedback += 10000;
        } else
            feedback += DistManhattan(endpoint_x.at(idx), endpoint_y.at(idx),
                                      planner->plan.poses.back().pose.position.x,
                                      planner->plan.poses.back().pose.position.y);
    }

    void get_start_and_goal(vector<int> sx, vector<int> sy, vector<int> ex, vector<int> ey) {
        for (int i = 0; i < sx.size(); i++) {
            startpoint_x.push_back(sx[i]);
            startpoint_y.push_back(sy[i]);
            endpoint_x.push_back(ex[i]);
            endpoint_y.push_back(ey[i]);
        }

        ROS_INFO_STREAM("pl pointer in s and g: " << this);
    }


    void set_planning_leaf(int num_robots, vector<int> &permti) {
        for (int i = 0; i < num_robots; ++i) {
            AStartFindPath *planner;
            planner = new AStartFindPath;

            if (parent_loc != top_loc) {
                planner->robot_id = i;
                planner->startpoint_x = (*parent_loc).pathes.at(i).poses.back().pose.position.x;
                planner->startpoint_y = (*parent_loc).pathes.at(i).poses.back().pose.position.y;
                ROS_INFO_STREAM("获得当前位置作为本次规划的起点:" << planner->startpoint_x
                                                   << " " << planner->startpoint_y);

                Tpoint tpoint;
                tpoint.robot_id = i;
                tpoint.x = (*parent_loc).pathes.at(i).poses.back().pose.position.x;
                tpoint.y = (*parent_loc).pathes.at(i).poses.back().pose.position.y;
                tpoint.t = 0;
                this->tpath.push_back(tpoint);

            } else {
                planner->robot_id = i;
                planner->startpoint_x = startpoint_x[i];
                planner->startpoint_y = startpoint_y[i];
            }
            planner->endpoint_x = endpoint_x[i];
            planner->endpoint_y = endpoint_y[i];
            ROS_INFO_STREAM("got destination." << planner->endpoint_x << planner->endpoint_y);

            // 将本planner所在的leaf的指针存入planner中;
            // 将本planner的指针存入所在leaf中;
            planner->leaf_ptr = this;
            this->planners.push_back(planner);
        }

    }

    AStartFindPath *get_planner_by_robot_ID(int rid) {
        for (int i = 0; i < planners.size(); ++i) {
            if (planners.at(i)->robot_id == rid)
                return planners.at(i);
        }
    }


    void publish_path(vector <nav_msgs::Path> &fullpaths, tree<planning_leaf>::iterator &last_planner_group,
                      vector <ros::Publisher> &nav_plans) {
        for (int i = 0; i < (*last_planner_group).planners.size(); ++i) {
            ROS_WARN_STREAM("planners size: " << (*last_planner_group).planners.size());
            nav_msgs::Path fullpath = fullpaths.at(i);
            fullpath.header.frame_id = "odom";
            int dep = 0;
            tree<planning_leaf>::iterator pointer_group = last_planner_group;
            while (pointer_group->parent_loc != pointer_group->top_loc) {
                reverse(pointer_group->pathes.at(i).poses.begin(), pointer_group->pathes.at(i).poses.end());
                fullpath.poses.insert(fullpath.poses.end(), pointer_group->pathes.at(i).poses.begin(),
                                      pointer_group->pathes.at(i).poses.end());
                reverse(pointer_group->pathes.at(i).poses.begin(), pointer_group->pathes.at(i).poses.end());
                pointer_group = pointer_group->parent_loc;
                dep++;
            }
            reverse(pointer_group->pathes.at(i).poses.begin(), pointer_group->pathes.at(i).poses.end());
            fullpath.poses.insert(fullpath.poses.end(), pointer_group->pathes.at(i).poses.begin(),
                                  pointer_group->pathes.at(i).poses.end());
            reverse(pointer_group->pathes.at(i).poses.begin(), pointer_group->pathes.at(i).poses.end());
            //                for(int i = 1; i < last_planner_group->pathes.size(); i++)
            //                {
            //                    fullpath.poses.insert(fullpath.poses.end(),last_planner_group->pathes.at(i).poses.begin(),last_planner_group->pathes.at(i).poses.end());
            //                }

            ROS_INFO_STREAM("dep: " << dep);
            ROS_INFO_STREAM("now we will pub full path...");
            if (1)
                for (auto j = 0; j<fullpath.poses.size(); j++)
                {
                    ROS_INFO_STREAM("points in fullpath: " << fullpath.poses.at(j).pose.position.x<< " " << fullpath.poses.at(j).pose.position.y<<" "<<j);
                }

            nav_plans[i].publish(fullpath);
        }
        published = true;
    }

    tree<planning_leaf>::iterator grow_new_leaf() {
        //init a planer_group
        planning_leaf pg;

        child_loc = tree_ptr->append_child(self_loc, pg);

        (*child_loc).tree_ptr = tree_ptr;
        (*child_loc).top_loc = top_loc;
        (*child_loc).parent_loc = self_loc;
        (*child_loc).self_loc = child_loc;

        return child_loc;
    }


    void print_tpath() {
        std::cout << "tpath: ";
        for (auto &Tpoint : tpath)
            std::cout << "[id-" << Tpoint.robot_id << " p:" << Tpoint.x <<" "<< Tpoint.y << " t:" << Tpoint.t << "] ,";
        std::cout << endl;
    }


};

class planning_tree {
public:
    tree<planning_leaf> *tr;
    tree<planning_leaf>::iterator top;

    planning_tree() {
        tr = new tree<planning_leaf>;
        ROS_INFO_STREAM("solidding the tree.");
        top = tr->begin();
        ROS_INFO_STREAM("top is the begin.");
    }

    vector <tree<planning_leaf>::iterator> init_set_multi_robot_astar_planner(int permt_size) {
        vector <tree<planning_leaf>::iterator> init_pl_locs;
        for (int i = 0; i < permt_size; ++i) {
            // 这里面tr是树,top是树的顶端,本次添加的pl是第一层树叶,通过循环permt_size次添加多个树叶
            planning_leaf pl;
            ROS_INFO_STREAM("treeeeee: " << tr);
            tree<planning_leaf>::iterator pl_locs = tr->append_child(top, pl);

            // so the appen_child is a deep copy!
            // and will not print out the solid process's msg.

            (*pl_locs).tree_ptr = tr;
            (*pl_locs).top_loc = top;
            (*pl_locs).self_loc = pl_locs;
            (*pl_locs).parent_loc = tr->parent(pl_locs);
            init_pl_locs.push_back(pl_locs);
        }
        return init_pl_locs;
    }

};


inline void sort_vector_ascend(vector <tree<planning_leaf>::iterator> open_planner_group_vec) {

}

inline bool feedback_smaller_than(tree<planning_leaf>::iterator &pg, tree<planning_leaf>::iterator &pg_other) {
    return pg->feedback < pg_other->feedback;
}

#endif //ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
