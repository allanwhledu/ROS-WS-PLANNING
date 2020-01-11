#include "astar_global_planner.h"

bool testhfile(int x){
    if(x==1)
        return true;
    else
        return false;
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

    openlist = new OpenList;
    closelist= new CloseList;

    isRootLoop = false;

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
//    unsigned int temp=((d_x - x)>=0?(d_x - x):-(d_x - x) + (d_y-y)>=0?(d_y-y):-(d_y-y))*DISTANCE;
    int temp = (std::abs(d_x-x)+std::abs(d_y-y))*DISTANCE;
    return temp;
}
void AStartFindPath::IsChangeParent(OpenList* open,int center_x, int center_y){
    int i;
    for(i=0; i<4 ; i++)
    {
        int new_x=center_x + direction[i][0];
        int new_y=center_y+ direction[i][1];
        if(new_x>=0 && new_y>=0 && new_y<m_height && new_x<m_width && IsInOpenList(new_x, new_y))
        {
            if(m_node[new_y][new_x].value_g > m_node[center_y][center_x].value_g+10)
            {
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
    if(m_node[y][x].flag == WALL)
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
void AddNode2Open(OpenList* openlist, Node* node)
{
    if(openlist ==NULL)
    {
        cout<<"no data in openlist!"<<endl;
        return;
    }

    if(node->flag!=STARTPOINT)
    {
        node->flag= INOPEN;
    }

    OpenList* temp =  new OpenList;
    temp->next=NULL;
    temp->PtrToNode = node;
    while(openlist->next != NULL)
    {
        if(node->value_f < openlist->next->PtrToNode->value_f)
        {
            temp->next = openlist->next;
            openlist->next = temp;
            break;
        }
        else
            openlist= openlist->next;
    }
    openlist->next = temp;
}
void AddNode2Close(CloseList* close, OpenList* &open)
{
    if(open==NULL)
    {
        ROS_INFO_STREAM("no data in openlist!");
        return;
    }
    if(open->PtrToNode->flag != STARTPOINT)
        open->PtrToNode->flag =INCLOSE;

    if(close->PtrToNode == NULL)
    {
        close->PtrToNode = open->PtrToNode;
        open=open->next;
        return;
    }
    while(close->next!= NULL)
        close= close->next;

    CloseList* temp= new CloseList;
    temp->PtrToNode = open->PtrToNode;
    temp->next=NULL;
    close->next= temp;  //close接上open

    open=open->next;  //open丢掉第一个
}

// 不断将未探索点加入探索列表，最终得到路径
bool AStartFindPath::Check_and_Put_to_Openlist(OpenList* open,int center_x, int center_y)
{
    // 遍历openlist，并输出其中的所有点坐标
    OpenList* tempopen = open;
    ROS_INFO_STREAM("now points in open:");
    while(tempopen->next!=NULL)
    {
        ROS_INFO_STREAM(tempopen->PtrToNode->location_x << " " << tempopen->PtrToNode->location_y<<" flag"<<tempopen->PtrToNode->flag<<" f"<<tempopen->PtrToNode->value_f);
        tempopen = tempopen->next;
    }
    ROS_INFO_STREAM(tempopen->PtrToNode->location_x << " " << tempopen->PtrToNode->location_y<<" flag"<<tempopen->PtrToNode->flag<<" f"<<tempopen->PtrToNode->value_f);


    // 利用传入的center xy信息开始check
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
                return true;
            }

            m_node[new_y][new_x].parent = &m_node[center_y][center_x];

            m_node[new_y][new_x].value_h = DistanceManhattan(endpoint_x, endpoint_y, new_x,new_y);//曼哈顿距离
            m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g+10;
            m_node[new_y][new_x].value_f = m_node[new_y][new_x].value_g+m_node[new_y][new_x].value_h;

            // 加入到 openlist中
            AddNode2Open(open, &m_node[new_y][new_x]);
            ROS_INFO_STREAM("add node "<<m_node[new_y][new_x].location_x<<","<<m_node[new_y][new_x].location_y<<","<<", g:"<<m_node[new_y][new_x].value_g<<", f:"<<m_node[new_y][new_x].value_f);
        }
        else
            ROS_INFO_STREAM("will not add node "<<m_node[new_y][new_x].location_x<<","<<m_node[new_y][new_x].location_y<<","<<", g:"<<m_node[new_y][new_x].value_g<<", f:"<<m_node[new_y][new_x].value_f);
    }

    // 重排openlist
    IsChangeParent(open, center_x,  center_y);

    return false;
}
void AStartFindPath::FindDestinnation(OpenList* open,CloseList* close)
{
    int i=0;
    printf("开始计算路径！\n");

    // check 1 step from startpoint.
    ROS_INFO_STREAM("start points:"<<startpoint_x<<" "<<startpoint_y);
//    Check_and_Put_to_Openlist(open,startpoint_x,startpoint_y);
//
//    // take 1st openlist node to closelist.
//    AddNode2Close(close,open);

    // circulate check...
    while(!Check_and_Put_to_Openlist(open, open->PtrToNode->location_x, open->PtrToNode->location_y))
    {
        i++;
        AddNode2Close(close,open);
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
    forstepcount= &m_node[open->PtrToNode->location_y][open->PtrToNode->location_x];

    i=1;
    while(forstepcount->parent->flag!=STARTPOINT)
    {
        ROS_INFO_STREAM("debugdebug.");
        ROS_INFO_STREAM("point:"<<" "<<forstepcount->location_x<<" "<<forstepcount->location_y);

        ROS_INFO_STREAM("forstepcount->parent->flag:"<<forstepcount->parent->flag);

        forstepcount=forstepcount->parent;
        ROS_INFO_STREAM("debug point1");
        i++;

        if(forstepcount->location_x == last_endpoint_x && forstepcount->location_y == last_endpoint_y && isRootLoop!=1)
//            i--;
            break;
    }
    i++;

    if(isRootLoop == false)
    {
        i--;
        ROS_INFO_STREAM("How many steps in path: "<<i);
        plan.poses.resize(i);
        plan.header.frame_id="odom";

        Node* tempnode;
        tempnode= &m_node[open->PtrToNode->location_y][open->PtrToNode->location_x];


        while(!(tempnode->location_x==last_endpoint_x && tempnode->location_y==last_endpoint_y))
        {
            ROS_INFO_STREAM("debug imformation3.");
            plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution;
            plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
            ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);
            tempnode=tempnode->parent;
        }

        plan.poses[--i].pose.position.x=last_endpoint_x*m_resolution;
        plan.poses[i].pose.position.y=last_endpoint_y*m_resolution;
        ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);

        ROS_INFO_STREAM("soon return.");
        return;

    }


    ROS_INFO_STREAM("How many steps in path: "<<i);
    plan.poses.resize(i);
    plan.header.frame_id="odom";


    // construct path from map.
    Node* tempnode;
    tempnode= &m_node[open->PtrToNode->location_y][open->PtrToNode->location_x];



    while(tempnode->parent->flag!=STARTPOINT)
    {
        ROS_INFO_STREAM("debug imformation1.");
        plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution;
        plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
        ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);

        tempnode=tempnode->parent;
    }
    ROS_INFO_STREAM("debug imformation.");
    plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution;
    plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
    ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<tempnode->flag<<" "<<tempnode->value_f);

//    tempnode=tempnode->parent;
    ROS_INFO_STREAM("this is the first loop.");
    plan.poses[--i].pose.position.x=startpoint_x*m_resolution;
    plan.poses[i].pose.position.y=startpoint_y*m_resolution;
    ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<m_node[startpoint_y][startpoint_x].flag<<" "<<m_node[startpoint_y][startpoint_x].value_f);



    ROS_INFO_STREAM("path constructed.");
}

void AStartFindPath::de_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // global var.
    m_height=msg->info.height;
    m_width=msg->info.width;
    m_resolution=msg->info.resolution;
    ROS_INFO_STREAM("get map with height and width as "<<m_height<<" "<<m_width<<" "<<m_resolution);

    m_node=new Node*[m_height]; //分配一个动态内存给m_node，它是由m_height个指向Node类的指针构成的数组
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

        closelist->PtrToNode=NULL;
        for(int i=0;i<m_height ;i++)
            for(int j=0;j<m_width;j++)
                if(m_node[i][j].flag!=WALL) m_node[i][j].flag=VIABLE;

        printf("重载节点完成！！\n");
    } else
    {
//        if(openlist!=NULL){delete  openlist; openlist=NULL;}
//        if(closelist!=NULL){delete closelist; closelist=NULL;}
//        openlist = new OpenList;
//        closelist= new CloseList;
//        closelist->PtrToNode=NULL;
    }

    // trans start and end information to map.
    // 1. change startpoint's flag and put it into openlist.

    m_node[y][x].flag = STARTPOINT;
    if(isRootLoop)
    {
        openlist->next=NULL;
        openlist->PtrToNode= &m_node[y][x];
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

void deepCopyMnode(Node* msg1[],int m_height, int m_width, Node* msg2[], const nav_msgs::OccupancyGrid::ConstPtr& msg, OpenList* open1, OpenList* open2, CloseList* close1, CloseList* close2)
{
    for(int i=0;i<m_height ;i++)
    {

        for(int j=0;j<m_width;j++)
        {
            msg1[i][j].location_x = j;
            msg1[i][j].location_y =i;
            msg1[i][j].parent = NULL;
            msg1[i][j].gray_val=msg->data[(i)*m_width+(j)];
            if(msg->data[(i)*m_width+(j)]==100) msg1[i][j].flag = WALL;
            else msg1[i][j].flag = VIABLE;
        }
    }

    for(int i=0;i<m_height ;i++)
    {
        for(int j=0;j<m_width;j++)
        {
            if(msg2[i][j].parent!=NULL)
            {
                int x = msg2[i][j].parent->location_y;
                int y = msg2[i][j].parent->location_x;
                msg1[i][j].flag = msg2[i][j].flag;
                msg1[i][j].value_g = msg2[i][j].value_g;
                msg1[i][j].value_h = msg2[i][j].value_h;
                msg1[i][j].value_f = msg2[i][j].value_f;
                msg1[i][j].parent = &msg1[y][x];
                ROS_INFO_STREAM("for m_node.yx"<<y<<" "<<x);
                ROS_INFO_STREAM("for parents msg2[y][x]"<<msg2[y][x].location_y<<" "<<msg1[y][x].location_x);
                ROS_INFO_STREAM("for parents msg1[y][x]"<<msg1[y][x].location_y<<" "<<msg1[y][x].location_x);
            }

            if(msg2[i][j].flag==4)
            {
                int x = msg2[i][j].location_y;
                int y = msg2[i][j].location_x;
                msg1[i][j].flag = msg2[i][j].flag;
                ROS_INFO_STREAM("for m_node.yx"<<y<<" "<<x);
                ROS_INFO_STREAM("forflag msg1[y][x]"<<msg1[y][x].location_y<<" "<<msg1[y][x].location_x);
            }
        }
    }


//    if(open1!=NULL){delete  open1; open1=NULL;}
//    if(close1!=NULL){delete close1; close1=NULL;}
//    open1 = new OpenList;
//    close1= new CloseList;
//    ROS_INFO_STREAM("debug1");


    ROS_INFO_STREAM("check mnode1 same location:"<<msg1[2][7].location_x<<msg1[2][7].location_y);
    ROS_INFO_STREAM("check mnode2 same location:"<<msg2[2][7].location_x<<msg2[2][7].location_y);


    open1->PtrToNode = &msg1[open2->PtrToNode->location_y][open2->PtrToNode->location_x];
    ROS_INFO_STREAM("check open1 location:"<<open1->PtrToNode->location_x);
    ROS_INFO_STREAM("check open2 location:"<<open2->PtrToNode->location_x);



    auto open2_tmp = new OpenList;
    open2_tmp = open2;
    auto open1_tmp = open1;

//    while(open2_tmp->next!=NULL)
//    {
//        ROS_INFO_STREAM("open2->next->PtrToNode->location_x:"<<open2_tmp->next->PtrToNode->location_x);
//        ROS_INFO_STREAM("open2->next->PtrToNode->location_y:"<<open2_tmp->next->PtrToNode->location_y);
//
//        open2_tmp = open2_tmp->next;
//    }

    while (open2->next!=NULL)
    {
        auto new_open = new OpenList;
        open2 = open2->next;
        if(open2->next==NULL)
            break;

        new_open->PtrToNode = &msg1[open2->PtrToNode->location_y][open2->PtrToNode->location_x];
        ROS_INFO_STREAM("get a opennode. "<<new_open->PtrToNode->location_x<<" "<<new_open->PtrToNode->location_y<<" "<<new_open->PtrToNode->flag);

        open1->next = new_open;
        open1 = open1->next;
    }

    // filter openlist.
//    open1 = open1_tmp;

    auto fakeopen1 = new OpenList;
    auto fakeopen2 = new OpenList;
    fakeopen1 = open1_tmp;
    fakeopen2->next = open1_tmp;

    fakeopen1 = fakeopen1->next;
    while(fakeopen1->next!=NULL)
    {
        if(fakeopen1->PtrToNode->flag==4 || fakeopen1->PtrToNode->flag==3)
        {
            fakeopen2->next->next = fakeopen1->next;
            ROS_INFO_STREAM("filt openlist: "<<fakeopen1->PtrToNode->location_x<<" "<<fakeopen1->PtrToNode->location_y<<" "<<fakeopen1->PtrToNode->flag);
            fakeopen1 = fakeopen1->next;
        } else
        {
            fakeopen2 = fakeopen2->next;
            fakeopen1 = fakeopen1->next;
        }


//        if(open1_tmp->PtrToNode->flag==4)
//        {
//            ROS_INFO_STREAM("filt openlist: "<<open1_tmp->PtrToNode->location_x<<" "<<open1_tmp->PtrToNode->location_y<<" "<<open1_tmp->PtrToNode->flag);
//            open1_tmp = open1_tmp->next;
//            fakeopen1 = open1_tmp;
//            continue;
//        } else
//        {
//            open1_tmp = open1_tmp->next;
//            fakeopen1 = fakeopen1->next;
//            continue;
//        }
    }
    open1 = open1_tmp;


    open1_tmp = open1;
    while(open1_tmp->next!=NULL)
    {
        ROS_INFO_STREAM("open1->next->PtrToNode:"<<open2_tmp->PtrToNode->location_x<<" "<<open1_tmp->PtrToNode->location_y<<" "<<open1_tmp->PtrToNode->flag);

        open1_tmp = open1_tmp->next;
    }

//    open2 = open2_tmp;

    close1->PtrToNode = &msg1[close2->PtrToNode->location_y][close2->PtrToNode->location_x];
    ROS_INFO_STREAM("check close1 location:"<<close1->PtrToNode->location_x);
    ROS_INFO_STREAM("check close2 location:"<<close2->PtrToNode->location_x);

    auto close2_tmp = new CloseList;
    close2_tmp = close2;
    auto close1_tmp = close1;

    while (close2->next!=NULL)
    {
        ROS_INFO_STREAM("get a closenode.");
        auto new_close = new CloseList;
        close2 = close2->next;
        if(close2->next==NULL)
            break;

        new_close->PtrToNode = &msg1[close2->PtrToNode->location_y][close2->PtrToNode->location_x];

        close1->next = new_close;
        close1 = close1->next;
    }

    close1 = close1_tmp;
    close2 = close2_tmp;

//    while(open2_tmp->next!=NULL)
//    {
//        ROS_INFO_STREAM("open2->next->PtrToNode->location_x:"<<open2_tmp->next->PtrToNode->location_x);
//        ROS_INFO_STREAM("open1->next->PtrToNode->location_x:"<<open1_tmp->next->PtrToNode->location_x);
//
//        open2_tmp = open2_tmp->next;
//        open1_tmp = open1_tmp->next;
//    }

//    while(close2_tmp->next!=NULL)
//    {
//        ROS_INFO_STREAM("close2->next->PtrToNode->location_x:"<<close2_tmp->next->PtrToNode->location_x);
//        ROS_INFO_STREAM("close1->next->PtrToNode->location_x:"<<close1_tmp->next->PtrToNode->location_x);
//
//        close2_tmp = close2_tmp->next;
//        close1_tmp = close1_tmp->next;
//    }


    ROS_INFO_STREAM("deepCopyNode completed.");
}



