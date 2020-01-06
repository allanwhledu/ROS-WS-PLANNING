#include "astar_global_planner.h"

bool testhfile(int x){
    if(x==1)
        return true;
    else
        return false;
}

// 计算与检测有关的子函数
unsigned int AStartFindPath::DistanceManhattan(int d_x, int d_y, int x, int y)
{
    unsigned int temp=((d_x - x)>0?(d_x - x):-(d_x - x) + (d_y-y)>0?(d_y-y):-(d_y-y))*DISTANCE;
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
bool AStartFindPath::IsAvailable(int x, int y, int time)
{
    bool flag=true;
    if(IsInOpenList(x, y))
        flag = false;
    if(IsInCloseList(x, y))
        flag = false;

    vector<geometry_msgs::Point>::iterator it;
    for(it = map.data.begin();it!=map.data.end();++it){
        if ((x==it->x-1 && y==it->y-1 && time==it->z) || (x==it->x-1 && y==it->y-1 && time==it->z-1)){
            ROS_WARN_STREAM("conflict in "<<x<<","<<y<<","<<time<<".");
            flag = false;
        }
    }

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
    if(m_node[y][x].flag == INCLOSE|| m_node[y][x].flag==STARTPOINT)
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
    temp->opennode = node;
    while(openlist->next != NULL)
    {
        if(node->value_f < openlist->next->opennode->value_f)
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
    if(open->opennode->flag != STARTPOINT)
        open->opennode->flag =INCLOSE;

    if(close->closenode == NULL)
    {
        close->closenode = open->opennode;
        open=open->next;
        return;
    }
    while(close->next!= NULL)
        close= close->next;

    CloseList* temp= new CloseList;
    temp->closenode = open->opennode;
    temp->next=NULL;
    close->next= temp;  //close接上open

    open=open->next;  //open丢掉第一个
}


// 不断将未探索点加入探索列表，最终得到路径
bool AStartFindPath::Check_and_Put_to_Openlist(OpenList* open,int center_x, int center_y)
{
    ROS_INFO_STREAM("now points in open:");
    while(open->next!=NULL)
    {
        ROS_INFO_STREAM(open->opennode->location_x<<" "<<open->opennode->location_y);
        open = open->next;
    }
    ROS_INFO_STREAM(open->opennode->location_x<<" "<<open->opennode->location_y);

    ROS_INFO_STREAM("checking point "<<m_node[center_y][center_x].location_x<<","<<m_node[center_y][center_x].location_y<<","<<m_node[center_y][center_x].value_g/10);
    int i;
    for(i=0; i<4 ; i++)
    {
        int new_x=center_x + direction[i][0];
        int new_y=center_y+ direction[i][1];
        int time = m_node[center_y][center_x].value_g/10+1;

        if(new_x>=0 && new_y>=0 && new_y<m_height && new_x<m_width && IsAvailable(new_x, new_y, time))
        {

            if(	m_node[new_y][new_x].flag==DESTINATION)
            {
                m_node[new_y][new_x].parent = &m_node[center_y][center_x];
                m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g+10;
                return true;
            }
            m_node[new_y][new_x].flag =INOPEN;
            m_node[new_y][new_x].parent = &m_node[center_y][center_x];
            m_node[new_y][new_x].value_h = DistanceManhattan(endpoint_x, endpoint_y, new_x,new_y);//曼哈顿距离

            m_node[new_y][new_x].value_g = m_node[center_y][center_x].value_g+10;

            m_node[new_y][new_x].value_f = m_node[new_y][new_x].value_g+m_node[new_y][new_x].value_h;

            AddNode2Open(open, &m_node[new_y][new_x]);// 加入到 openlist中
            ROS_INFO_STREAM("add node "<<m_node[new_y][new_x].location_x<<","<<m_node[new_y][new_x].location_y<<","<<m_node[new_y][new_x].value_g/10);
        }
        else
            ROS_INFO_STREAM("will not add node "<<m_node[new_y][new_x].location_x<<","<<m_node[new_y][new_x].location_y<<","<<m_node[new_y][new_x].value_g/10);
    }
    IsChangeParent(open, center_x,  center_y);

    return false;
}
void AStartFindPath::FindDestinnation(OpenList* open,CloseList* close)
{
    int i=0;
    printf("开始计算路径！\n");
    Check_and_Put_to_Openlist(open,startpoint_x,startpoint_y);
    AddNode2Close(close,open);
    while(!Check_and_Put_to_Openlist(open, open->opennode->location_x-1, open->opennode->location_y-1))
    {
        i++;
        AddNode2Close(close,open);
        if(open==NULL||i>5)
        {
//            printf("找不到出口！\n");
            ROS_INFO_STREAM("completed segment path.");
            ROS_INFO_STREAM(i);
            break;
        }
    }
    printf("计算路径成功！！\n");

    nav_msgs::Path plan;

    Node* forstepcount;
    forstepcount= &m_node[open->opennode->location_y-1][open->opennode->location_x-1];
    i=0;
    while(forstepcount->parent->flag!=STARTPOINT)
    {
        forstepcount=forstepcount->parent;
        i++;
    }
    i++;

    ROS_INFO_STREAM("How many steps in path: "<<i+1);
    plan.poses.resize(i+1);
    plan.header.frame_id="odom";

    Node* tempnode;
    tempnode= &m_node[open->opennode->location_y-1][open->opennode->location_x-1];
    plan.poses[i].pose.position.x=tempnode->location_x*m_resolution; //为了使网格到栅格所以减了半个格子
    plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
    plan.poses[i].pose.position.z=tempnode->value_g/10;
    ROS_INFO_STREAM("i= "<<i<<" last point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<plan.poses[i].pose.position.z);
    while(tempnode->parent->flag!=STARTPOINT)
    {
        tempnode=tempnode->parent;
//    plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution-m_resolution/2; //为了使网格到栅格所以减了半个格子
//    plan.poses[i].pose.position.y=tempnode->location_y*m_resolution-m_resolution/2;
        plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution; //为了使网格到栅格所以减了半个格子
        plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
        plan.poses[i].pose.position.z=tempnode->value_g/10;
        ROS_INFO_STREAM("i= "<<i<<" point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<plan.poses[i].pose.position.z);
    }
    tempnode=tempnode->parent;
    plan.poses[--i].pose.position.x=tempnode->location_x*m_resolution; //为了使网格到栅格所以减了半个格子
    plan.poses[i].pose.position.y=tempnode->location_y*m_resolution;
    plan.poses[i].pose.position.z=tempnode->value_g/10;
    ROS_INFO_STREAM("i= "<<i<<" first point in path: "<<plan.poses[i].pose.position.x<<" "<<plan.poses[i].pose.position.y<<" "<<plan.poses[i].pose.position.z);
    nav_plan.publish(plan);
}

