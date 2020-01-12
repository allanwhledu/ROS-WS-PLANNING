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

extern int m_height;
extern int m_width;
extern int m_resolution;


struct Node
{
    int    flag;
    char gray_val;
    unsigned int location_x;
    unsigned int location_y;
    unsigned int value_h;
    unsigned int value_g;
    unsigned int value_f;
    Node* parent;
    Node();
};

struct CloseNode
{
    Node* PtrToNode;

    bool operator()(const CloseNode& t1,const CloseNode& t2){
        return t1.PtrToNode->value_f<t2.PtrToNode->value_f;    //会产生升序排序,若改为>,则变为降序
    }

    CloseNode(){ PtrToNode=NULL;};
};

struct OpenNode
{
    Node* PtrToNode;

    bool operator()(const OpenNode& t1,const OpenNode& t2){
        return t1.PtrToNode->value_f<t2.PtrToNode->value_f;    //会产生升序排序,若改为>,则变为降序
    }

    OpenNode(){ PtrToNode= NULL;};
};

extern bool CompOpen(OpenNode first, OpenNode second);
extern bool CompClose(CloseNode first, CloseNode second);


class AStartFindPath
{
public:


    nav_msgs::Path plan;
    nav_msgs::Path Null_plan;

    Node** m_node;
    AStartFindPath();
    virtual ~AStartFindPath(){};
    int GetPos(int &x,int &y);
    void FindDestinnation(std::list<OpenNode>* open, std::list<CloseNode>* close);
    bool Check_and_Put_to_Openlist(std::list<OpenNode>* open, std::list<CloseNode>* close);
    bool IsInOpenList(int x, int y);
    bool IsInCloseList(int x, int y);
    void IsChangeParent(std::list<OpenNode>* open, int x, int y);
    bool IsAvailable(int x, int y);
    unsigned int DistanceManhattan(int d_x, int d_y, int x, int y);
    void AddNode2Open(std::list<OpenNode>* openlist, Node* node);
    void AddNode2Close(std::list<CloseNode>* close, std::list<OpenNode>* open);

    void de_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void setTarget();

    unsigned int steps;
    int startpoint_x;
    int startpoint_y;
    int endpoint_x;
    int endpoint_y;
//    int m_height,m_width;
//    double m_resolution;

    //Lists
//    OpenNode* openlist;
//    CloseNode* closelist ;
    std::list<CloseNode>* closelist;
    std::list<OpenNode>* openlist;


    int x,y,des_x,des_y;
    char Thrs;


    // flag to go
    bool sign_cacul;
    void clear_tmpplan();
    bool isRootLoop;

    int last_endpoint_x;
    int last_endpoint_y;

    int loop_count;

    bool arrived;
};

bool testhfile(int x);

class TESTCOPY
{
public:
    int obj1 = 1;
    int *obj2 = NULL;
};



void deepCopyMnode(Node* msg1[], int m_height, int m_width, Node* msg2[], const nav_msgs::OccupancyGrid::ConstPtr& msg, std::list<OpenNode>* open1, std::list<OpenNode>* open2, std::list<CloseNode>* close1, std::list<CloseNode>* close2);
