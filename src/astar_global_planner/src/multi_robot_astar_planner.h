//
// Created by 李云帆 on 2020-01-14.
//



#ifndef ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
#define ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H

#include "astar_global_planner.h"
using namespace std;

#define ARR_LEN(array, length){ length =  sizeof(array) / sizeof(array[0]); }

struct Tpoint
{
    int x;
    int y;
    int t;
};

class planner_group {
public:
    vector<int> startpoint_x;
    vector<int> startpoint_y;
    vector<int> endpoint_x;
    vector<int> endpoint_y;

    vector<AStartFindPath*> planners;

    int feedback;
    vector<Tpoint> tpath;
    nav_msgs::Path path;

    tree<planner_group>* tree_ptr= nullptr;
    tree<planner_group>::iterator top_loc;
    tree<planner_group>::iterator parent_loc;
    tree<planner_group>::iterator self_loc;
    tree<planner_group>::iterator child_loc;

    int get_feedback(int idx){
        return this->planners.at(idx)->feedback;
    }

    void get_start_and_goal(vector<int> sx, vector<int> sy, vector<int> ex, vector<int> ey){
        for(int i = 0; i < sx.size() ; i++)
        {
            startpoint_x.push_back(sx[i]);
            startpoint_y.push_back(sy[i]);
            endpoint_x.push_back(ex[i]);
            endpoint_y.push_back(ey[i]);
        }

        ROS_INFO_STREAM("pg pointer in s and g: "<<this);
    }



    void set_planner_group(){
       for (int i = 0; i < 2; ++i) {
           AStartFindPath *init_planner;
           init_planner = new AStartFindPath;
           ROS_INFO_STREAM("init planner completed.");

           if (parent_loc!=top_loc){
               ROS_INFO_STREAM("getting last endpoint...");
               init_planner->startpoint_x = (*parent_loc).path.poses.back().pose.position.x;
               init_planner->startpoint_y = (*parent_loc).path.poses.back().pose.position.y;
               init_planner->endpoint_x = endpoint_x[i];
               init_planner->endpoint_y = endpoint_y[i];
               ROS_INFO_STREAM("got lastendpoint.");
           } else
           {
               init_planner->startpoint_x = startpoint_x[i];
               init_planner->startpoint_y = startpoint_y[i];
               init_planner->endpoint_x = endpoint_x[i];
               init_planner->endpoint_y = endpoint_y[i];
           }

           init_planner->group_ptr = this;

           this->planners.push_back(init_planner);
       }
   }

    tree<planner_group>::iterator grow_new_leaf(){
        //init a planer_group
        planner_group pg;

        child_loc = tree_ptr->append_child(self_loc, pg);

        (*child_loc).tree_ptr = tree_ptr;
        (*child_loc).top_loc = top_loc;
        (*child_loc).parent_loc = self_loc;
        (*child_loc).self_loc = child_loc;

        return child_loc;
    }


    void print_tpath()
    {
        std::cout<<"tpath: ";
        for(auto & Tpoint : tpath)
            std::cout<<Tpoint.x<<Tpoint.y<<" "<<Tpoint.t<<" ,";
        std::cout<<endl;
    }


};

class multi_robot_astar_planner {
public:
    tree<planner_group>* tr;
    tree<planner_group>::iterator top;

    multi_robot_astar_planner(){
        tr = new tree<planner_group>;
        ROS_INFO_STREAM("solidding the tree.");
        top = tr->begin();
        ROS_INFO_STREAM("top is the begin.");

        planner_group pg;

        ROS_INFO_STREAM("treeeeee: "<< tr);

        tree<planner_group>::iterator pg_loc = tr->append_child(top, pg);
        // so the appen_child is a deep copy!
        // and will not print out the solid process's msg.

        (*pg_loc).tree_ptr = tr;
        (*pg_loc).top_loc = top;
        (*pg_loc).self_loc = pg_loc;
        (*pg_loc).parent_loc = tr->parent(pg_loc);
    }

};

//void perm(int arr[],vector<vector<int>> ret) {
//    int len;
//    ARR_LEN(arr, len);
//    if (len < 2) return;
//    int i, j, temp;
//    do {
//        vector<int> tmp;
//        for (int k = 0; k < len; ++k) {
//            tmp.push_back(arr[k]);
//        }
//        ret.push_back(tmp);
//        i = j = len - 1;
//        while (i > 0 && arr[i] < arr[i - 1]) --i;
//        temp = i;
//        if (i == 0) break;
//        while (temp + 1 < len && arr[temp + 1] > arr[i - 1]) ++temp;
//        swap(arr[i - 1], arr[temp]);  //交换两个值
//        reverse(arr + i, arr + len);  //逆序
//    } while (true);
//}



#endif //ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
