#include "astar_global_planner.h"

bool testhfile(int x){
    if(x==1)
        return true;
    else
        return false;
}

extern bool CompOpen(ListNode first, ListNode second)
{
    if(first.PtrToNode->value_f >= second.PtrToNode->value_f) //由大到小排序 //如果想要由小到大，改为大于即可
    {
        return false;
    } else
    {
        return true;
    }
}

extern bool CompClose(ListNode first, ListNode second)
{
    if(first.PtrToNode->value_f >= second.PtrToNode->value_f) //由大到小排序 //如果想要由小到大，改为大于即可
    {
        return false;
    } else
    {
        return true;
    }
}

// 结构体和类的初始化
Node::Node()
{
    flag=0;
    value_h= 0;
    value_g= 0;
    value_f =  0;
    parent= NULL;
}
AStartFindPath::AStartFindPath()
{
    startpoint_x = 0;startpoint_y = 0;
    endpoint_y = 0; endpoint_x = 0;
    x=0;y=0;

    m_node=new Node*[m_height];
    for(int i=0;i<m_height ;i++)
    {
        m_node[i]=new Node[m_width];
    }

    openlist = new std::list<ListNode>;
    closelist= new std::list<ListNode>;

    isRootLoop = false;

    arrived = false;

}

// 获得目的坐标参数的子函数
int AStartFindPath::GetPos(int& x,int& y)
{
    x=startpoint_x;
    y=startpoint_y;
    ROS_INFO_STREAM("start point in map "<<startpoint_x<<" "<<startpoint_y<<"\n");
    return 0;
}

// 计算与检测有关的子函数
unsigned int AStartFindPath::DistanceManhattan(int d_x, int d_y, int x, int y)
{
    int temp = (std::abs(d_x-x)+std::abs(d_y-y))*DISTANCE;
    return temp;
}
void AStartFindPath::IsChangeParent(std::list<ListNode>* open, int center_x, int center_y){
    int i;
    for(i=0; i<4 ; i++)
    {
        int new_x=center_x + direction[i][0];
        int new_y=center_y+ direction[i][1];
        if(new_x>=0 && new_y>=0 && new_y<m_height && new_x<m_width && IsInOpenList(new_x, new_y))
        {
            if(m_node[new_y][new_x].value_g > m_node[center_y][center_x].value_g+10)
            {
                ROS_INFO_STREAM("changing parents.");
                m_node[new_y][new_x].parent = &m_node[center_y][center_x];
                m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g+10;
            }
        }
    }

}
bool AStartFindPath::IsAvailable(int x, int y)
{
    bool flag=true;

    if(IsInOpenList(x, y))
        flag = false;
    if(IsInCloseList(x, y))
        flag = false;
    if(m_node[y][x].flag == WALL || m_node[y][x].flag == STARTPOINT)
        flag = false;

    return flag;
}
bool AStartFindPath::IsInOpenList(int x,int y)
{
    if(m_node[y][x].flag == INOPEN)
        return true;
    else
        return false;
}
bool AStartFindPath::IsInCloseList(int x,int y)
{
    if(m_node[y][x].flag == INCLOSE)
        return true;
    else
        return false;
}

// 与维护列表有关的子函数
void  AStartFindPath::AddNode2Open(std::list<ListNode>* openlist, Node* node)
{
    if(node->flag!=STARTPOINT)
    {
        node->flag= INOPEN;
    }

    ListNode opennode;
    opennode.PtrToNode = node;

    openlist->push_front(opennode);
    openlist->sort(CompOpen);
}

void AStartFindPath::AddNode2Close(std::list<ListNode>* close, std::list<ListNode>* open)
{
    if(open==NULL)
    {
        ROS_INFO_STREAM("no data in openlist!");
        return;
    }
    if(open->front().PtrToNode->flag != STARTPOINT)
        open->front().PtrToNode->flag =INCLOSE;

    ListNode closenode;
    closenode.PtrToNode = open->front().PtrToNode;
    close->push_back(closenode);
    open->pop_front();
}

// 不断将未探索点加入探索列表，最终得到路径
bool AStartFindPath::Check_and_Put_to_Openlist(std::list<ListNode>* open, std::list<ListNode>* close)
{
    // 遍历openlist，并输出其中的所有点坐标
    int center_x = open->front().PtrToNode->location_x;
    int center_y = open->front().PtrToNode->location_y;

    ROS_INFO_STREAM("now points in open: "<<open->size());
    for (list<ListNode>::iterator it = open->begin(); it != open->end(); ++it)
        if(it->PtrToNode->parent != NULL)
            ROS_INFO_STREAM(it->PtrToNode->location_x << " " << it->PtrToNode->location_y<<" flag"<<it->PtrToNode->flag<<" h"<<it->PtrToNode->value_h<<" g"<<it->PtrToNode->value_g<<" f"<<it->PtrToNode->value_f<<" parent-> "<<it->PtrToNode->parent->location_y<<" "<<it->PtrToNode->parent->location_x);

    // 利用传入的center xy信息开始check
    AddNode2Close(close,open);
    ROS_INFO_STREAM("checking point "<<m_node[center_y][center_x].location_x<<","<<m_node[center_y][center_x].location_y<<","<<", g:"<<m_node[center_y][center_x].value_g<<", f:"<<m_node[center_y][center_x].value_f<<", flag:"<<m_node[center_y][center_x].flag);
    int i;
    for(i=0; i<4 ; i++)
    {
        int new_x=center_x + direction[i][0];
        int new_y=center_y+ direction[i][1];

        if(new_x>=0 && new_y>=0 && new_y<m_height && new_x<m_width && IsAvailable(new_x, new_y))
        {

            if(	m_node[new_y][new_x].flag==DESTINATION)
            {
                m_node[new_y][new_x].parent = &m_node[center_y][center_x];
                m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g+10;
                AddNode2Open(open, &m_node[new_y][new_x]);
                ROS_INFO_STREAM("destination already got in openlist.");
                arrived = true;
                return true;
            }

            m_node[new_y][new_x].parent = &m_node[center_y][center_x];
            m_node[new_y][new_x].value_h = DistanceManhattan(endpoint_x, endpoint_y, new_x,new_y);//曼哈顿距离
            m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g+10;
            m_node[new_y][new_x].value_f = m_node[new_y][new_x].value_g+m_node[new_y][new_x].value_h;

            // 加入到 openlist中

            ROS_INFO_STREAM("add node "<<m_node[new_y][new_x].location_x<<","<<m_node[new_y][new_x].location_y<<","<<", h:"<<m_node[new_y][new_x].value_h<<","<<", g:"<<m_node[new_y][new_x].value_g<<", f:"<<m_node[new_y][new_x].value_f);
            AddNode2Open(open, &m_node[new_y][new_x]);
        }
        else
            ROS_INFO_STREAM("will not add node "<<m_node[new_y][new_x].location_x<<","<<m_node[new_y][new_x].location_y<<","<<", g:"<<m_node[new_y][new_x].value_g<<", f:"<<m_node[new_y][new_x].value_f);
    }

    // 重排openlist
    IsChangeParent(open, center_x,  center_y);

    return false;
}
void AStartFindPath::FindDestinnation(std::list<ListNode>* open, std::list<ListNode>* close)
{
    int i=0;
    printf("开始计算路径！\n");

    // check 1 step from startpoint.
    ROS_INFO_STREAM("start points:"<<startpoint_x<<" "<<startpoint_y);

    // circulate check...
    while(!Check_and_Put_to_Openlist(open, close))
    {
        i++;
        int length = 5;

        if(open==NULL||i>5)
        {
            ROS_INFO_STREAM("completed segment path.");
            ROS_INFO_STREAM(i);
            break;
        }
    }
    printf("计算路径成功！！\n");


    // count path step.
    Node* forstepcount;
    forstepcount= &m_node[open->front().PtrToNode->location_y][open->front().PtrToNode->location_x];

    plan.header.frame_id = plan.header.frame_id="odom";
    while(forstepcount->flag!=STARTPOINT)
    {
        ROS_INFO_STREAM("point:"<<" "<<forstepcount->location_x<<" "<<forstepcount->location_y<<" flag: "<<forstepcount->flag);

        if(forstepcount->parent!=NULL)
            ROS_INFO_STREAM("parents:"<<" "<<forstepcount->parent->location_x<<" "<<forstepcount->parent->location_y);

        geometry_msgs::PoseStamped point;
        point.pose.position.x = forstepcount->location_x;
        point.pose.position.y = forstepcount->location_y;

        plan.poses.push_back(point);

        forstepcount=forstepcount->parent;
        ROS_INFO_STREAM("debug point1");

//        if(forstepcount->location_x == last_endpoint_x && forstepcount->location_y == last_endpoint_y && isRootLoop!=1)
//            break;

        if(forstepcount->flag==STARTPOINT)
            ROS_INFO_STREAM("reaching start.");
    }
    // here is the startpoint.
    ROS_INFO_STREAM("point:"<<" "<<forstepcount->location_x<<" "<<forstepcount->location_y<<" flag: "<<forstepcount->flag);
    geometry_msgs::PoseStamped point;
    point.pose.position.x = forstepcount->location_x;
    point.pose.position.y = forstepcount->location_y;
    plan.poses.push_back(point);


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

void AStartFindPath::de_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // global var.
    m_height=msg->info.height;
    m_width=msg->info.width;
    m_resolution=msg->info.resolution;
    ROS_INFO_STREAM("get map with height and width as "<<m_height<<" "<<m_width<<" "<<m_resolution);

    m_node=new Node*[m_height];
    for(int i=0;i<m_height ;i++)
    {
        m_node[i]=new Node[m_width];
        for(int j=0;j<m_width;j++)
        {
            m_node[i][j].location_x = j;
            m_node[i][j].location_y =i;
            m_node[i][j].parent = NULL;
            m_node[i][j].gray_val=msg->data[(i)*m_width+(j)];
            if(msg->data[(i)*m_width+(j)]==100) m_node[i][j].flag = WALL;
            else m_node[i][j].flag = VIABLE;
        }
    }

    ROS_INFO_STREAM("now test the map");
    for(int i=0;i<m_height ;i++)
    {
        for(int j=0;j<m_width;j++)
        {
            int gray = msg->data[(i)*m_width+(j)];
//            ROS_INFO_STREAM(gray<<" ");
        }
//        ROS_INFO_STREAM("\n");
    }
    ROS_INFO_STREAM("now test the nodes");
    for(int i=0;i<m_height;i++)
    {
        for(int j=0;j<m_width;j++)
        {
            int gray = m_node[i][j].gray_val;
//            ROS_INFO_STREAM("node "<<m_node[i][j].location_x<<" "<<m_node[i][j].location_y<<" and gray value "<<gray<<" ");
        }
//        ROS_INFO_STREAM("\n");
    }
}

void AStartFindPath::setTarget()
{
    int delay;
    ros::param::get("~delay",delay);
    ros::Duration(delay).sleep();

    // set target.
    des_x=endpoint_x;
    des_y=endpoint_y;
    ROS_INFO_STREAM("goal and it's flag "<<m_node[des_y][des_x].location_x<<" "<<m_node[des_y][des_x].location_y<<" "<<m_node[des_y][des_x].flag);
    // check target.
    if(m_node[des_y][des_x].flag==WALL)
    {
        printf("destination is on the wall!\n");
        return;
    }

    // set start.
    GetPos(x,y);
    ROS_INFO_STREAM("start and it's flag "<<m_node[y][x].location_x<<" "<<m_node[y][x].location_y<<" "<<m_node[y][x].flag);


    // load map in rootloop.
    if(isRootLoop)
    {
        printf("重载节点！！\n");

        closelist->front().PtrToNode = NULL;
        for(int i=0;i<m_height ;i++)
            for(int j=0;j<m_width;j++)
                if(m_node[i][j].flag!=WALL) m_node[i][j].flag=VIABLE;

        printf("重载节点完成！！\n");
    }

    // trans start and end information to map.
    // 1. change startpoint's flag and put it into openlist.

    m_node[y][x].flag = STARTPOINT;
    if(isRootLoop)
    {
        ListNode newopen;
        newopen.PtrToNode = &m_node[y][x];
        openlist->push_back(newopen);
        startpoint_x=x;
        startpoint_y=y;
    }


    // 2. change endpoint's flag.
    if(isRootLoop)
    {
        m_node[des_y][des_x].flag = DESTINATION;
        endpoint_x= des_x;
        endpoint_y=des_y;
    }

    // run algorithm.
    ROS_INFO_STREAM("getting path...");
    FindDestinnation(openlist,closelist);
    ROS_INFO_STREAM("has get path.");
}

void AStartFindPath::clear_tmpplan()
{
    plan = Null_plan;
}

void deepCopyMnode(Node* msg1[], int m_height, int m_width, Node* msg2[], const nav_msgs::OccupancyGrid::ConstPtr& msg, std::list<ListNode>* open1, std::list<ListNode>* open2, std::list<ListNode>* close1, std::list<ListNode>* close2)
{
    for(int i=0;i<m_height ;i++)
    {

        for(int j=0;j<m_width;j++)
        {
            msg1[i][j].location_x = j;
            msg1[i][j].location_y =i;
            if(msg2[i][j].parent != NULL)
            {
                unsigned int x = msg2[i][j].parent->location_y;
                unsigned int y = msg2[i][j].parent->location_x;
                msg1[i][j].parent = &msg1[y][x];
                ROS_INFO_STREAM("get parents.");
            } else
                msg1[i][j].parent = NULL;

            msg1[i][j].gray_val=msg->data[(i)*m_width+(j)];
            msg1[i][j].flag = msg2[i][j].flag;

            msg1[i][j].value_g = msg2[i][j].value_g;
            msg1[i][j].value_h = msg2[i][j].value_h;
            msg1[i][j].value_f = msg2[i][j].value_f;
        }
    }

    for(int i=0;i<m_height ;i++)
    {
        for(int j=0;j<m_width;j++)
        {
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

            if(msg2[i][j].flag==4 || msg2[i][j].flag==5)
            {
                int x = msg2[i][j].location_y;
                int y = msg2[i][j].location_x;
                msg1[i][j].flag = msg2[i][j].flag;
                ROS_INFO_STREAM("for m_node.yx"<<y<<" "<<x);
                ROS_INFO_STREAM("forflag msg1[y][x]"<<msg1[y][x].location_y<<" "<<msg1[y][x].location_x);
            }
        }
    }

    ROS_INFO_STREAM("translate openlist...");
    for (std::list<ListNode>::iterator it = open2->begin(); it != open2->end(); ++it)
    {
        ListNode new_open;
        new_open.PtrToNode = &msg1[it->PtrToNode->location_y][it->PtrToNode->location_x];
        open1->push_back(new_open);
        ROS_INFO_STREAM("get a opennode. "<<new_open.PtrToNode->location_x<<" "<<new_open.PtrToNode->location_y<<" "<<new_open.PtrToNode->flag);
    }

    for (std::list<ListNode>::iterator it = open2->begin(); it != open2->end(); ++it)
        ROS_INFO_STREAM("open1->next->PtrToNode:"<<it->PtrToNode->location_x<<" "<<it->PtrToNode->location_y<<" "<<it->PtrToNode->flag);



    ROS_INFO_STREAM("translate closelist...");
    for (std::list<ListNode>::iterator it = close2->begin(); it != close2->end(); ++it)
    {
        ListNode new_close;
        new_close.PtrToNode = &msg1[it->PtrToNode->location_y][it->PtrToNode->location_x];
        close1->push_back(new_close);
        ROS_INFO_STREAM("get a closenode. "<<new_close.PtrToNode->location_x<<" "<<new_close.PtrToNode->location_y<<" "<<new_close.PtrToNode->flag);
    }


    ROS_INFO_STREAM("deepCopyNode completed.");
}



