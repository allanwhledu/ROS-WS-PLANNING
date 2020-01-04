#include "astar_global_planner.h"

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
    m_node=NULL;
    m_width=0;m_height=0;
    openlist = NULL;
    closelist= NULL;

    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map",1,&AStartFindPath::map_Callback,this);
    map_sub2 = n.subscribe<trimap::Trimap>("/trimap",1,&AStartFindPath::map2_Callback,this);
    end_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,&AStartFindPath::end_Callback,this);
    nav_plan = n.advertise<nav_msgs::Path>("astar_path", 1);
}

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

//  vector<geometry_msgs::Point>::iterator it;
//  for(it = map.data.begin();it!=map.data.end();++it){
//    if ((x==it->x-1 && y==it->y-1 && time==it->z) || (x==it->x-1 && y==it->y-1 && time==it->z-1)){
//      ROS_WARN_STREAM("conflict in "<<x<<","<<y<<","<<time<<".");
//      flag = false;
//    }
//  }

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
        if(open==NULL||i>30000)
        {
            printf("找不到出口！\n");
            return;
        }
    }
    printf("计算路径成功！！\n");

    nav_msgs::Path plan;

    Node* forstepcount;
    forstepcount= &m_node[endpoint_y][endpoint_x];
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
    tempnode= &m_node[endpoint_y][endpoint_x];
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
unsigned int AStartFindPath::DistanceManhattan(int d_x, int d_y, int x, int y)
{
    unsigned int temp=((d_x - x)>0?(d_x - x):-(d_x - x) + (d_y-y)>0?(d_y-y):-(d_y-y))*DISTANCE;
    return temp;
}

int AStartFindPath::GetPos(int& x,int& y)
{
    //获取当前位置

    try
    {
        transform_listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        printf("ERROR: %s",ex.what());
        ros::Duration(1.0).sleep();
        return 1;
    }
//  x=AGV_transform.getOrigin().x()/m_resolution-1;
//  y=AGV_transform.getOrigin().y()/m_resolution-1;

    int x_0, y_0;
    ros::param::get("~x_0",x_0);
    ros::param::get("~y_0",y_0);
    x=x_0-1;
    y=y_0-1;
    ROS_INFO_STREAM("start point in map "<<x_0<<" "<<y_0<<"\n");
    return 0;
}

void AStartFindPath::map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
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
            m_node[i][j].location_x = j+1;
            m_node[i][j].location_y =i+1;
            m_node[i][j].parent = NULL;
            m_node[i][j].gray_val=msg->data[(i)*m_width+(j)];
            if(msg->data[(i)*m_width+(j)]==100) m_node[i][j].flag = WALL;
            else m_node[i][j].flag = VIABLE;
        }
    }
    ROS_INFO_STREAM("now test the map");
    for(int i=0;i<m_height-1 ;i++)
    {
        for(int j=0;j<m_width-1;j++)
        {
            int gray = msg->data[(i)*m_width+(j)];
            ROS_INFO_STREAM(gray<<" ");
        }
        ROS_INFO_STREAM("\n");
    }
    ROS_INFO_STREAM("now test the nodes");
    for(int i=0;i<m_height;i++)
    {
        for(int j=0;j<m_width;j++)
        {
            int gray = m_node[i][j].gray_val;
            ROS_INFO_STREAM("node "<<m_node[i][j].location_x<<" "<<m_node[i][j].location_y<<" and gray value "<<gray<<" ");
        }
        ROS_INFO_STREAM("\n");
    }
}
void AStartFindPath::map2_Callback(const trimap::Trimap::ConstPtr& msg)
{
    map.data = msg->data;
}
void AStartFindPath::end_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    //  des_x=msg->pose.position.x/m_resolution;
    //  des_y=msg->pose.position.y/m_resolution;
    int x_1, y_1;
    ros::param::get("~x_1",x_1);
    ros::param::get("~y_1",y_1);
    des_x=x_1-1;  //因为，这里des_x,des_y是目标点的-1
    des_y=y_1-1;

    int delay;
    ros::param::get("~delay",delay);
    ros::Duration(delay).sleep();

    ROS_INFO_STREAM("goal and it's flag "<<m_node[des_y][des_x].location_x<<" "<<m_node[des_y][des_x].location_y<<" "<<m_node[des_y][des_x].flag);

    GetPos(x,y);
    ROS_INFO_STREAM("start and it's flag "<<m_node[y][x].location_x<<" "<<m_node[y][x].location_y<<" "<<m_node[y][x].flag);


    if(m_node[des_y][des_x].flag==WALL)
    {
        printf("destination is on the wall!\n");
        return;
    }

    printf("重载节点！！\n");
    if(openlist!=NULL){delete  openlist; openlist=NULL;}
    if(closelist!=NULL){delete closelist; closelist=NULL;}
    openlist = new OpenList;
    closelist= new CloseList;
    closelist->closenode=NULL;
    for(int i=0;i<m_height ;i++)
        for(int j=0;j<m_width;j++)
            if(m_node[i][j].flag!=WALL) m_node[i][j].flag=VIABLE;
    printf("重载节点完成！！\n");

    m_node[y][x].flag = STARTPOINT;
    openlist->next=NULL;
    openlist->opennode= &m_node[y][x];
    startpoint_x=x;
    startpoint_y=y;
    m_node[des_y][des_x].flag = DESTINATION;
    endpoint_x= des_x;
    endpoint_y=des_y;

    FindDestinnation(openlist,closelist);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");
    AStartFindPath planner;
    const OpenList* HEADOPEN= planner.openlist;
    const CloseList* HEADCLOSE= planner.closelist;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
