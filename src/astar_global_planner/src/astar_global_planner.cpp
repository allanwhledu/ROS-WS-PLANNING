#include "multi_robot_astar_planner.h"

bool testhfile(int x) {
    return x == 1;
}

/*
extern bool Comp(list<ListNode>::iterator &first, list<ListNode>::iterator &second) {
    if (!first->PtrToNode || !second->PtrToNode) {
        ROS_WARN_STREAM("There is a NULL pointer");
        return false;
    }
    if (first->PtrToNode->value_f >= second->PtrToNode->value_f) //TODO 这里是什么原因，会使得这里没有flag，flag是空的？
    {
        return (first->PtrToNode->value_h < second->PtrToNode->value_h);
    } else {
        return true;
    }
}*/

extern bool Comp(ListNode &first, ListNode &second) {
    if (!first.PtrToNode || !second.PtrToNode) {
        ROS_WARN_STREAM("There is a NULL pointer");
        return false;
    }
    if (first.PtrToNode->value_f >= second.PtrToNode->value_f) //TODO 这里是什么原因，会使得这里没有flag，flag是空的？
    {
        return (first.PtrToNode->value_h < second.PtrToNode->value_h);
    } else {
        return true;
    }
}

// 结构体和类的初始化
Node::Node() {
    flag = 0;
    value_h = 0;
    value_g = 0;
    value_f = 0;
    parent = NULL;
}

// init planner class.
AStartFindPath::AStartFindPath() {
    startpoint_x = 0;
    startpoint_y = 0;
    endpoint_y = 0;
    endpoint_x = 0;
    x = 0; // current position
    y = 0;

    m_node = new Node *[m_height];
    for (int i = 0; i < m_height; i++) {
        m_node[i] = new Node[m_width];
    }

    openlist = new std::list<ListNode>;
    closelist = new std::list<ListNode>;

    isRootLoop = false;

    arrived = false;
    feedback = 0;
    ROS_WARN_STREAM("this is a new planner");
}

// 获得目的坐标参数的子函数
int AStartFindPath::GetPos(int &x, int &y) {
    x = startpoint_x;
    y = startpoint_y;
    ROS_INFO_STREAM("start point in map " << startpoint_x << " " << startpoint_y << "\n");
    return 0;
}

// 计算与检测有关的子函数
unsigned int AStartFindPath::DistanceManhattan(int d_x, int d_y, int x, int y) {
    return (std::abs(d_x - x) + std::abs(d_y - y)) * DISTANCE;
}

void AStartFindPath::IsChangeParent(std::list <ListNode> *open, int center_x, int center_y) {
    int i;
    for (i = 0; i < 4; i++) {
        int new_x = center_x + direction[i][0];
        int new_y = center_y + direction[i][1];
        if (new_x >= 0 && new_y >= 0 && new_y < m_height && new_x < m_width && IsInOpenList(new_x, new_y)) {
            if (m_node[new_y][new_x].value_g > m_node[center_y][center_x].value_g + 10) {
                ROS_INFO_STREAM("changing parents.");
                m_node[new_y][new_x].parent = &m_node[center_y][center_x];
                m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g + 10;
            }
        }
    }

}

bool AStartFindPath::IsAvailable(int x, int y, int time) {
    if (IsInOpenList(x, y))
        return false;
    if (IsInCloseList(x, y))
        return false;
    if (m_node[y][x].flag == WALL || m_node[y][x].flag == STARTPOINT)
        return false;

    for (auto it = group_ptr->tpath.begin(); it != group_ptr->tpath.end(); it++) {
        if (x == it->x && y == it->y && time == it->t) {
            ROS_WARN_STREAM("conflict in " << x << "," << y << "," << time << ".");
            return false;
        }
    }
    return true;
}

bool AStartFindPath::IsInOpenList(int x, int y) {
    return (m_node[y][x].flag == INOPEN);
}

bool AStartFindPath::IsInCloseList(int x, int y) {
    return (m_node[y][x].flag == INCLOSE);
}

// 与维护列表有关的子函数
void AStartFindPath::AddNode2Open(std::list <ListNode> *openlist, Node *node) {
    if (node->flag != STARTPOINT) {
        node->flag = INOPEN;
    }

    ListNode opennode;
    opennode.PtrToNode = node;

    openlist->push_front(opennode);
    sort()
    openlist->sort(Comp);
}

void output_close_list(std::list <ListNode> *close) {
    string close_list_str = "Starting output close list: ";
    auto it = ++close->begin();
    while (it != close->end()) {
        if (it->PtrToNode) {
            close_list_str +=
                    "x" + intToString(it->PtrToNode->location_x) + "y" + intToString(it->PtrToNode->location_y) +
                    "f" + intToString(it->PtrToNode->value_f) + "h" + intToString(it->PtrToNode->value_h) +
                    "flag" + intToString(it->PtrToNode->flag) + ", ";
        } else {
            close_list_str += "NULL PIONTER, ";
        }
        ++it;
    }
    ROS_WARN_STREAM(close_list_str);
}

bool has_nullptr_in_close(std::list <ListNode> *close) {
    bool sort_flag = true;
    auto it = ++close->begin();
    while (it != close->end()) {
        if (!it->PtrToNode) {
            ROS_WARN_STREAM("NULL PIONTER");
            sort_flag = false;
        }
        ++it;
    }
    return sort_flag;
}

void AStartFindPath::AddNode2Close(std::list <ListNode> *close, std::list <ListNode> *open) {
    if (!open->size() || !open->front().PtrToNode) {
        ROS_INFO_STREAM("no data in openlist!");
        return;
    }

    if (open->front().PtrToNode->flag != STARTPOINT)
        open->front().PtrToNode->flag = INCLOSE;


    ListNode closenode;
    closenode.PtrToNode = open->front().PtrToNode;

    if (closenode.PtrToNode->flag != STARTPOINT)
        close->push_back(closenode);
    open->pop_front();
    if (!close->empty()) {
        if (has_nullptr_in_close(close)) {
            close->sort(Comp); //TODO: bug
            ROS_INFO_STREAM("end sort");
        }
        output_close_list(close);
    }
}

// 不断将未探索点加入探索列表，最终得到路径
bool AStartFindPath::Check_and_Put_to_Openlist(std::list <ListNode> *open, std::list <ListNode> *close) {
    // 遍历openlist，并输出其中的所有点坐标
    int center_x = open->front().PtrToNode->location_x;
    int center_y = open->front().PtrToNode->location_y;

    ROS_INFO_STREAM("now points in open: " << open->size());
    for (list<ListNode>::iterator it = open->begin(); it != open->end(); ++it)
        if (it->PtrToNode->parent != NULL)
            ROS_INFO_STREAM(
                    it->PtrToNode->location_x << " " << it->PtrToNode->location_y << " flag" << it->PtrToNode->flag
                                              << " h" << it->PtrToNode->value_h << " g" << it->PtrToNode->value_g
                                              << " f" << it->PtrToNode->value_f << " parent-> "
                                              << it->PtrToNode->parent->location_x << " "
                                              << it->PtrToNode->parent->location_y);

    // 利用传入的center xy信息开始check
    AddNode2Close(close, open);

    ROS_INFO_STREAM(
            "checking point " << m_node[center_y][center_x].location_x << "," << m_node[center_y][center_x].location_y
                              << "," << ", g:" << m_node[center_y][center_x].value_g << ", f:"
                              << m_node[center_y][center_x].value_f << ", flag:" << m_node[center_y][center_x].flag);
    for (int i = 0; i < 4; i++) {
        int new_x = center_x + direction[i][0];
        int new_y = center_y + direction[i][1];

        if (new_x >= 0 && new_y >= 0 && new_y < m_height && new_x < m_width &&
            IsAvailable(new_x, new_y, m_node[center_y][center_x].value_g + 10)) {

            if (m_node[new_y][new_x].flag == DESTINATION) {
                m_node[new_y][new_x].parent = &m_node[center_y][center_x];
                m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g + 10;
                AddNode2Open(open, &m_node[new_y][new_x]);
                ROS_INFO_STREAM("destination already got in openlist.");
                arrived = true;
                return true;
            }

            m_node[new_y][new_x].parent = &m_node[center_y][center_x];
            m_node[new_y][new_x].value_h = DistanceManhattan(endpoint_x, endpoint_y, new_x, new_y);//曼哈顿距离
            m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g + 10;
            m_node[new_y][new_x].value_f = m_node[new_y][new_x].value_g + m_node[new_y][new_x].value_h;

            // 加入到 openlist中

//            ROS_INFO_STREAM(
//                    "add node " << m_node[new_y][new_x].location_x << "," << m_node[new_y][new_x].location_y << ","
//                                << ", h:" << m_node[new_y][new_x].value_h << "," << ", g:"
//                                << m_node[new_y][new_x].value_g << ", f:" << m_node[new_y][new_x].value_f);
            AddNode2Open(open, &m_node[new_y][new_x]);
        }
//        else
//            ROS_INFO_STREAM(
//                    "will not add node " << m_node[new_y][new_x].location_x << "," << m_node[new_y][new_x].location_y
//                                         << "," << ", g:" << m_node[new_y][new_x].value_g << ", f:"
//                                         << m_node[new_y][new_x].value_f);
    }

    // 重排openlist
    IsChangeParent(open, center_x, center_y);

    return false;
}

void AStartFindPath::FindDestinnation(std::list <ListNode> *open, std::list <ListNode> *close) {
    int i = 0;
    printf("开始计算路径！\n");

    // check 1 step from startpoint.
    ROS_INFO_STREAM("start points:" << startpoint_x << " " << startpoint_y);

    // circulate check...
    while (!Check_and_Put_to_Openlist(open, close)) {
        int length = 8;

        if (open == NULL || ++i > length) {
//            ROS_INFO_STREAM("completed segment path.");
//            ROS_INFO_STREAM(i);
            break;
        }
    }
    printf("计算路径成功！！\n");


    // count path step.

    list <ListNode> *closeAndopen;
    closeAndopen = new std::list<ListNode>;

    closeAndopen->assign(openlist->begin(), openlist->end());
    for (list<ListNode>::iterator it = closelist->begin(); it != closelist->end(); ++it)
        closeAndopen->push_back(*it);
    closeAndopen->sort(Comp);

//    ROS_INFO_STREAM("now points in closeAndopen: " << closeAndopen->size());
//    for (list<ListNode>::iterator it = closeAndopen->begin(); it != closeAndopen->end(); ++it)
//        if (it->PtrToNode->parent != NULL)
//            ROS_INFO_STREAM(
//                    it->PtrToNode->location_x << " " << it->PtrToNode->location_y << " flag" << it->PtrToNode->flag
//                                              << " h" << it->PtrToNode->value_h << " g" << it->PtrToNode->value_g
//                                              << " f" << it->PtrToNode->value_f << " parent-> "
//                                              << it->PtrToNode->parent->location_x << " "
//                                              << it->PtrToNode->parent->location_y);

    Node *forstepcount;
    forstepcount = &m_node[closeAndopen->front().PtrToNode->location_y][closeAndopen->front().PtrToNode->location_x];

    plan.header.frame_id = plan.header.frame_id = "odom";
    while (forstepcount->flag != STARTPOINT) {
        ROS_INFO_STREAM("point:" << " " << forstepcount->location_x << " " << forstepcount->location_y << " flag: "
                                 << forstepcount->flag);

        if (forstepcount->parent != NULL)
            ROS_INFO_STREAM(
                    "parents:" << " " << forstepcount->parent->location_x << " " << forstepcount->parent->location_y);

        geometry_msgs::PoseStamped point;
        point.pose.position.x = forstepcount->location_x;
        point.pose.position.y = forstepcount->location_y;

        ROS_INFO_STREAM("debug imformation1.");
        if (forstepcount->parent == NULL) {
            ROS_INFO_STREAM("debug imformation2.");
            break;
        } else {
            ROS_INFO_STREAM("debug imformation3.");
            plan.poses.push_back(point);
            forstepcount = forstepcount->parent;
        }


        if (forstepcount->flag == STARTPOINT)
            ROS_INFO_STREAM("reaching start.");
    }
    // here is the startpoint.
    ROS_INFO_STREAM("point:" << " " << forstepcount->location_x << " " << forstepcount->location_y << " flag: "
                             << forstepcount->flag);

    geometry_msgs::PoseStamped point;
    point.pose.position.x = forstepcount->location_x;
    point.pose.position.y = forstepcount->location_y;
    plan.poses.push_back(point);

    reverse(plan.poses.begin(), plan.poses.end());

    int path_length = 4;
    int path_length0 = plan.poses.size();
    if (path_length0 > path_length) {
        plan.poses.erase(plan.poses.end() - (path_length0 - path_length), plan.poses.end());
    }

    // return tpath.
    Tpoint tpoint;
    for (int i = 0; i < plan.poses.size(); i++) {
        tpoint.x = plan.poses.at(i).pose.position.x;
        tpoint.y = plan.poses.at(i).pose.position.y;
        tpoint.t = i * DISTANCE;
        group_ptr->tpath.push_back(tpoint);
    }


//
//    if(isRootLoop == false)
//    {
//        i--;
//        ROS_INFO_STREAM("How many steps in path: "<<i);
//        plan.poses.resize(i);
//        plan.header.frame_id="odom";
//
//        Node* tempnode;
//        tempnode= &m_node[open->front().PtrToNode->location_y][open->front().PtrToNode->location_x];
//
//
//        while(!(tempnode->location_x==last_endpoint_x && tempnode->location_y==last_endpoint_y))
//        {
//            ROS_INFO_STREAM("debug imformation3.");
//            plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution;
//            plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
//            ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);
//            tempnode=tempnode->parent;
//        }
//
//        plan.poses[--i].pose.position.x=last_endpoint_x*m_resolution;
//        plan.poses[i].pose.position.y=last_endpoint_y*m_resolution;
//        ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);
//
//        ROS_INFO_STREAM("soon return.");
//        return;
//
//    }
//
//
//    ROS_INFO_STREAM("How many steps in path: "<<i);
//    plan.poses.resize(i);
//    plan.header.frame_id="odom";
//
//
//    // construct path from map.
//    Node* tempnode;
//    tempnode= &m_node[open->front().PtrToNode->location_y][open->front().PtrToNode->location_x];
//
//
//
//    while(tempnode->parent->flag!=STARTPOINT)
//    {
//        ROS_INFO_STREAM("debug imformation1.");
//        plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution;
//        plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
//        ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);
//
//        tempnode=tempnode->parent;
//    }
//    ROS_INFO_STREAM("debug imformation.");
//    plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution;
//    plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
//    ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);
//
////    tempnode=tempnode->parent;
//    ROS_INFO_STREAM("this is the first loop.");
//    plan.poses[--i].pose.position.x=startpoint_x*m_resolution;
//    plan.poses[i].pose.position.y=startpoint_y*m_resolution;
//    ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<m_node[startpoint_y][startpoint_x].flag<<" "<<m_node[startpoint_y][startpoint_x].value_f);

    ROS_INFO_STREAM("path constructed.");
}

// get map to planner class.
void AStartFindPath::de_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    // global var.
    m_height = msg->info.height;
    m_width = msg->info.width;
    m_resolution = msg->info.resolution;
    ROS_INFO_STREAM("get map with height and width as " << m_height << " " << m_width << " " << m_resolution);

    m_node = new Node *[m_height];
    for (int i = 0; i < m_height; i++) {
        m_node[i] = new Node[m_width];
        for (int j = 0; j < m_width; j++) {
            m_node[i][j].location_x = j;
            m_node[i][j].location_y = i;
            m_node[i][j].parent = NULL;
            m_node[i][j].gray_val = msg->data[(i) * m_width + (j)];
            if (msg->data[(i) * m_width + (j)] == 100) m_node[i][j].flag = WALL;
            else m_node[i][j].flag = VIABLE;
        }
    }

    ROS_INFO_STREAM("now test the map");
    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            int gray = msg->data[(i) * m_width + (j)];
//            ROS_INFO_STREAM(gray<<" ");
        }
//        ROS_INFO_STREAM("\n");
    }
    ROS_INFO_STREAM("now test the nodes");
    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            int gray = m_node[i][j].gray_val;
//            ROS_INFO_STREAM("node "<<m_node[i][j].location_x<<" "<<m_node[i][j].location_y<<" and gray value "<<gray<<" ");
        }
//        ROS_INFO_STREAM("\n");
    }
}

void AStartFindPath::setTarget() {
    int delay;
    ros::param::get("~delay", delay);
    ros::Duration(delay).sleep();

    // set target.
    des_x = endpoint_x;
    des_y = endpoint_y;
    ROS_INFO_STREAM(
            "goal and it's flag " << m_node[des_y][des_x].location_x << " " << m_node[des_y][des_x].location_y
                                  << " "
                                  << m_node[des_y][des_x].flag);
    // check target.
    if (m_node[des_y][des_x].flag == WALL) {
        printf("destination is on the wall!\n");
        return;
    }

    // set start.
    GetPos(x, y);
    ROS_INFO_STREAM("start and it's flag " << m_node[y][x].location_x << " " << m_node[y][x].location_y << " "
                                           << m_node[y][x].flag);


    // load map in rootloop.

    if (openlist->size()) {
        openlist->clear();
    }
    closelist->front().PtrToNode = NULL;


    // trans start and end information to map.
    // 1. change startpoint's flag and put it into openlist.
    m_node[y][x].flag = STARTPOINT;

    ListNode newopen;
    newopen.PtrToNode = &m_node[y][x];

    newopen.PtrToNode->value_g = 0;
    newopen.PtrToNode->value_h = DistManhattan(x, y, des_x, des_y);
    newopen.PtrToNode->value_f = newopen.PtrToNode->value_g + newopen.PtrToNode->value_h;

    openlist->push_back(newopen);
    startpoint_x = x;
    startpoint_y = y;


    // 2. change endpoint's flag.
    m_node[des_y][des_x].flag = DESTINATION;
    endpoint_x = des_x;
    endpoint_y = des_y;


    // run algorithm.
    ROS_INFO_STREAM("getting path...");
    FindDestinnation(openlist, closelist);
    ROS_INFO_STREAM("has get path.");
}

void AStartFindPath::clear_tmpplan() {
    plan = Null_plan;
}

void
deepCopyMnode(Node *msg1[], int m_height, int m_width, Node *msg2[], const nav_msgs::OccupancyGrid::ConstPtr &msg,
              std::list <ListNode> *open1, std::list <ListNode> *open2, std::list <ListNode> *close1,
              std::list <ListNode> *close2) {
    for (int i = 0; i < m_height; i++) {

        for (int j = 0; j < m_width; j++) {
            msg1[i][j].location_x = j;
            msg1[i][j].location_y = i;
            if (msg2[i][j].parent != NULL) {
                unsigned int x = msg2[i][j].parent->location_x;
                unsigned int y = msg2[i][j].parent->location_y;
                msg1[i][j].parent = &msg1[y][x];
                ROS_INFO_STREAM("get parents.");
            } else
                msg1[i][j].parent = NULL;

            msg1[i][j].gray_val = msg->data[(i) * m_width + (j)];
            msg1[i][j].flag = msg2[i][j].flag;

            msg1[i][j].value_g = msg2[i][j].value_g;
            msg1[i][j].value_h = msg2[i][j].value_h;
            msg1[i][j].value_f = msg2[i][j].value_f;
        }
    }

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
//            if(msg2[i][j].parent!=NULL)
//            {
//
//                int x = msg2[i][j].parent->location_y;
//                int y = msg2[i][j].parent->location_x;
//                msg1[i][j].flag = msg2[i][j].flag;
//                msg1[i][j].value_g = msg2[i][j].value_g;
//                msg1[i][j].value_h = msg2[i][j].value_h;
//                msg1[i][j].value_f = msg2[i][j].value_f;
//                msg1[i][j].parent = &msg1[y][x];
//                ROS_INFO_STREAM("for m_node.yx"<<y<<" "<<x);
//                ROS_INFO_STREAM("for parents msg2[y][x]"<<msg2[y][x].location_y<<" "<<msg1[y][x].location_x);
//                ROS_INFO_STREAM("for parents msg1[y][x]"<<msg1[y][x].location_y<<" "<<msg1[y][x].location_x);
//            }

            if (msg2[i][j].flag == 4 || msg2[i][j].flag == 5) {
                int x = msg2[i][j].location_x;
                int y = msg2[i][j].location_y;
                msg1[i][j].flag = msg2[i][j].flag;
                ROS_INFO_STREAM("for m_node.yx" << y << " " << x);
                ROS_INFO_STREAM("forflag msg1[y][x]" << msg1[y][x].location_y << " " << msg1[y][x].location_x);
            }
        }
    }

    ROS_INFO_STREAM("translate openlist...");
    for (std::list<ListNode>::iterator it = open2->begin(); it != open2->end(); ++it) {
        ListNode new_open;
        new_open.PtrToNode = &msg1[it->PtrToNode->location_y][it->PtrToNode->location_x];
        open1->push_back(new_open);
        ROS_INFO_STREAM(
                "get a opennode. " << new_open.PtrToNode->location_x << " " << new_open.PtrToNode->location_y << " "
                                   << new_open.PtrToNode->flag);
    }

    for (std::list<ListNode>::iterator it = open2->begin(); it != open2->end(); ++it)
        ROS_INFO_STREAM(
                "open1->next->PtrToNode:" << it->PtrToNode->location_x << " " << it->PtrToNode->location_y << " "
                                          << it->PtrToNode->flag);


    ROS_INFO_STREAM("translate closelist...");
    for (std::list<ListNode>::iterator it = close2->begin(); it != close2->end(); ++it) {
        ListNode new_close;
        new_close.PtrToNode = &msg1[it->PtrToNode->location_y][it->PtrToNode->location_x];
        close1->push_back(new_close);
        ROS_INFO_STREAM(
                "get a closenode. " << new_close.PtrToNode->location_x << " " << new_close.PtrToNode->location_y
                                    << " "
                                    << new_close.PtrToNode->flag);
    }


    ROS_INFO_STREAM("deepCopyNode completed.");
}



