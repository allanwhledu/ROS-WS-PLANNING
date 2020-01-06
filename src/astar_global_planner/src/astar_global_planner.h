#include<iostream>
#include<fstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "trimap/Trimap.h"
#include <tf/exceptions.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
using namespace std;


const int DISTANCE=10;
const int direction[4][2]={{-1,0},{0,-1},{0,1},{1,0}};// 方向
enum{VIABLE, WALL, INOPEN, INCLOSE, STARTPOINT, DESTINATION};

struct Node
{
    //char perperty;
    int    flag;
    char gray_val;
    unsigned int location_x;
    unsigned int location_y;
    unsigned int location_z;
    unsigned int value_h;
    unsigned int value_g;
    unsigned int value_f;
    Node* parent;
    Node();
};

struct CloseList
{
    Node* closenode;
    CloseList* next;
    CloseList(){ next=NULL;};
};

struct OpenList
{
    Node* opennode;
    OpenList* next;
    OpenList(){next= NULL;};
};


class AStartFindPath
{
public:
    ros::NodeHandle n;
    trimap::Trimap map;
    Node **m_node;
    AStartFindPath();
    virtual ~AStartFindPath(){};
    int GetPos(int &x,int &y);
    void FindDestinnation(OpenList* open,CloseList* close);
    bool Check_and_Put_to_Openlist(OpenList* , int x, int y);
    bool IsInOpenList(int x, int y);
    bool IsInCloseList(int x, int y);
    void IsChangeParent(OpenList*, int x, int y);
    bool IsAvailable(int x, int y, int time);
    unsigned int DistanceManhattan(int d_x, int d_y, int x, int y);
    /*Callback Functions*/
    void map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void map2_Callback(const trimap::Trimap::ConstPtr& msg);
    void end_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    ros::Subscriber map_sub;
    ros::Subscriber map_sub2;
    ros::Subscriber end_sub;
    ros::Publisher nav_plan;
    //TF Scalar Listener
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    unsigned int steps;
    int startpoint_x;
    int startpoint_y;
    int endpoint_x;
    int endpoint_y;
    int m_height,m_width;
    double m_resolution;
    //Lists
    OpenList* openlist;
    CloseList* closelist ;
    int x,y,des_x,des_y;
    char Thrs;
    ros::Publisher map_pub;

    // flag to go
    bool sign_cacul;
};

bool testhfile(int x);
