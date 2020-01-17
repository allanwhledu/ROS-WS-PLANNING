//
// Created by 李云帆 on 2020-01-14.
//



#ifndef ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
#define ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H

#include "astar_global_planner.h"
#include "utils.h"
#include <string>

using namespace std;

#define ARR_LEN(array, length){ length =  sizeof(array) / sizeof(array[0]); }

struct Tpoint {
    int x = 0;
    int y = 0;
    int t = 0;
};

class planner_group {
public:
    vector<int> startpoint_x;
    vector<int> startpoint_y;
    vector<int> endpoint_x;
    vector<int> endpoint_y;

    vector<AStartFindPath *> planners;

    int feedback = 9999;
    vector <Tpoint> tpath;

    vector <nav_msgs::Path> pathes;


    tree<planner_group> *tree_ptr = nullptr;
    tree<planner_group>::iterator top_loc;
    tree<planner_group>::iterator parent_loc;
    tree<planner_group>::iterator self_loc;
    tree<planner_group>::iterator child_loc;

    void add_feedback_from_path(nav_msgs::Path path, int idx) {
        if (feedback == 9999) { feedback = 0; }
        feedback += DistManhattan(endpoint_x.at(idx), endpoint_y.at(idx),
                                  path.poses.back().pose.position.x,
                                  path.poses.back().pose.position.y);
    }

    int get_feedback(int idx) {
        return this->planners.at(idx)->feedback;
    }

    void get_start_and_goal(vector<int> sx, vector<int> sy, vector<int> ex, vector<int> ey) {
        for (int i = 0; i < sx.size(); i++) {
            startpoint_x.push_back(sx[i]);
            startpoint_y.push_back(sy[i]);
            endpoint_x.push_back(ex[i]);
            endpoint_y.push_back(ey[i]);
        }

        ROS_INFO_STREAM("pg pointer in s and g: " << this);
    }


    void set_planner_group(int num_robots) {
        for (int i = 0; i < num_robots; ++i) {
            AStartFindPath *init_planner;
            init_planner = new AStartFindPath;
//            ROS_INFO_STREAM("init planner completed.");

            if (parent_loc != top_loc) {
//                ROS_INFO_STREAM("getting last endpoint...");
                init_planner->startpoint_x = (*parent_loc).pathes.at(i).poses.back().pose.position.x;
                init_planner->startpoint_y = (*parent_loc).pathes.at(i).poses.back().pose.position.y;
                init_planner->endpoint_x = endpoint_x[i];
                init_planner->endpoint_y = endpoint_y[i];
                ROS_INFO_STREAM("got last endpoint.");
            } else {
                init_planner->startpoint_x = startpoint_x[i];
                init_planner->startpoint_y = startpoint_y[i];
                init_planner->endpoint_x = endpoint_x[i];
                init_planner->endpoint_y = endpoint_y[i];
            }

            init_planner->group_ptr = this;

            this->planners.push_back(init_planner);
        }

    }

    void publish_path(vector <nav_msgs::Path> &fullpaths, tree<planner_group>::iterator &last_planner_group,
                      vector <ros::Publisher> &nav_plans) {
        for (int i = 0; i < (*last_planner_group).planners.size(); ++i) {
            nav_msgs::Path fullpath = fullpaths.at(i);
            fullpath.header.frame_id = "odom";
            int dep = 0;
            while (last_planner_group->parent_loc != last_planner_group->top_loc) {
                reverse(last_planner_group->pathes.at(i).poses.begin(), last_planner_group->pathes.at(i).poses.end());
                fullpath.poses.insert(fullpath.poses.end(), last_planner_group->pathes.at(i).poses.begin(),
                                      last_planner_group->pathes.at(i).poses.end());
                last_planner_group = last_planner_group->parent_loc;
                dep++;
            }
            reverse(last_planner_group->pathes.at(i).poses.begin(), last_planner_group->pathes.at(i).poses.end());
            fullpath.poses.insert(fullpath.poses.end(), last_planner_group->pathes.at(i).poses.begin(),
                                  last_planner_group->pathes.at(i).poses.end());
            //                for(int i = 1; i < last_planner_group->pathes.size(); i++)
            //                {
            //                    fullpath.poses.insert(fullpath.poses.end(),last_planner_group->pathes.at(i).poses.begin(),last_planner_group->pathes.at(i).poses.end());
            //                }

            ROS_INFO_STREAM("dep: " << dep);
            ROS_INFO_STREAM("now we will pub full path...");

            for (auto &pose : fullpath.poses) {
                ROS_INFO_STREAM("points in fullpath: " << pose.pose.position.x << " " << pose.pose.position.y);
            }
            nav_plans[i].publish(fullpath);
        }
    }

    tree<planner_group>::iterator grow_new_leaf() {
        //init a planer_group
        planner_group pg;

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
            std::cout << Tpoint.x << Tpoint.y << " " << Tpoint.t << " ,";
        std::cout << endl;
    }


};

class multi_robot_astar_planner {
public:
    tree<planner_group> *tr;
    tree<planner_group>::iterator top;

    multi_robot_astar_planner() {
        tr = new tree<planner_group>;
        ROS_INFO_STREAM("solidding the tree.");
        top = tr->begin();
        ROS_INFO_STREAM("top is the begin.");
    }

    vector<tree<planner_group>::iterator> init_set_multi_robot_astar_planner(int num_robots) {
        vector<tree<planner_group>::iterator> init_pg_locs;
        for (int i = 0; i < num_robots; ++i) {
            planner_group pg;
            ROS_INFO_STREAM("treeeeee: " << tr);
            tree<planner_group>::iterator pg_loc = tr->append_child(top, pg);
            // so the appen_child is a deep copy!
            // and will not print out the solid process's msg.

            (*pg_loc).tree_ptr = tr;
            (*pg_loc).top_loc = top;
            (*pg_loc).self_loc = pg_loc;
            (*pg_loc).parent_loc = tr->parent(pg_loc);
            init_pg_locs.push_back(pg_loc);
        }
        return init_pg_locs;
    }

};


inline void sort_vector_ascend(vector <tree<planner_group>::iterator> open_planner_group_vec) {

}

inline bool feedback_smaller_than(tree<planner_group>::iterator &pg, tree<planner_group>::iterator &pg_other) {
    return pg->feedback < pg_other->feedback;
}

#endif //ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
