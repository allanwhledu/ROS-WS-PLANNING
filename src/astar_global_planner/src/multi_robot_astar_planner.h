//
// Created by 李云帆 on 2020-01-14.
//

#ifndef ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
#define ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H

#include "astar_global_planner.h"
#include "tree.hh"
#define ARR_LEN(array, length){ length =  sizeof(array) / sizeof(array[0]); }

class planner_group {
public:
    vector<AStartFindPath> planners;
    int feedback;
    nav_msgs::Path path;
    tree<planner_group>::iterator parent_loc;

    int get_feedback(int idx){
        return this->planners.at(idx).feedback;
    }

   void set_planner_group(tree<planner_group> tr){
       for (int i = 0; i < 2; ++i) {
           AStartFindPath init_planner;
           ROS_INFO_STREAM("init tree completed.");

           if (parent_loc!=tr.begin()){
               ROS_INFO_STREAM("init tree completed1.");

               init_planner.startpoint_x = (*parent_loc).path.poses.front().pose.position.x;
               init_planner.startpoint_y = (*parent_loc).path.poses.front().pose.position.y;
           }
           ROS_INFO_STREAM("init tree completed2.");

           this->planners.push_back(init_planner);
       }
   }
};

class multi_robot_astar_planner {
public:
    tree<planner_group> tr;
    tree<planner_group>::iterator top;

    multi_robot_astar_planner(){
        top = tr.begin();
        planner_group pg;
        ROS_INFO_STREAM("cut in");
        tree<planner_group>::iterator pg_loc = tr.append_child(top, pg);
        ROS_INFO_STREAM("after cut in");
        pg.parent_loc = tr.parent(pg_loc);
        pg.set_planner_group(tr);
        ROS_INFO_STREAM("init tree completed.");
    }


    void multi_robot_astar_planning(){
        //init a planer_group
        planner_group pg;
    }

};

void perm(int arr[],vector<vector<int>> ret) {
    int len;
    ARR_LEN(arr, len);
    if (len < 2) return;
    int i, j, temp;
    do {
        vector<int> tmp;
        for (int k = 0; k < len; ++k) {
            tmp.push_back(arr[k]);
        }
        ret.push_back(tmp);
        i = j = len - 1;
        while (i > 0 && arr[i] < arr[i - 1]) --i;
        temp = i;
        if (i == 0) break;
        while (temp + 1 < len && arr[temp + 1] > arr[i - 1]) ++temp;
        swap(arr[i - 1], arr[temp]);  //交换两个值
        reverse(arr + i, arr + len);  //逆序
    } while (true);
}



#endif //ROS_WS_PLANNING_MULTI_ROBOT_ASTAR_PLANNER_H
