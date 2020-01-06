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

// 获得目的坐标参数的子函数
int AStartFindPath::GetPos(int& x,int& y)
{
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

// 用于交互的ROS回调函数
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

// 节点主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");

    if(testhfile(1))
        ROS_INFO_STREAM("testhfile success!");

    AStartFindPath planner;
    const OpenList* HEADOPEN= planner.openlist;
    const CloseList* HEADCLOSE= planner.closelist;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
