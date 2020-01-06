// 库文件的引用
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "trimap/Trimap.h"
#include "vector"

trimap::Trimap map;
std::vector<geometry_msgs::Point> temp_map;


void pathCallback_1(const nav_msgs::Path::ConstPtr& msg)
{
  // 对获取的信息进行计算，后发布信息
  std::vector<geometry_msgs::Pose> Poses;
  std::vector<geometry_msgs::Point> Points;
  std::vector<geometry_msgs::PoseStamped> PoseStamped;
  PoseStamped = msg->poses;
  std::vector<geometry_msgs::PoseStamped>::iterator it;
  for(it = PoseStamped.begin();it!=PoseStamped.end();++it){
    Poses.push_back(it->pose);
  }
  std::vector<geometry_msgs::Pose>::iterator it2;
  for(it2 = Poses.begin();it2!=Poses.end();++it2){
    Points.push_back(it2->position);
  }

  std::vector<geometry_msgs::Point>::iterator it3;
  for(it3 = Points.begin();it3!=Points.end();it3++){
    temp_map.push_back(*it3);
  }

  ROS_INFO_STREAM("get robot1's path.");
  map.data.assign(temp_map.begin(), temp_map.end());
}

void pathCallback_2(const nav_msgs::Path::ConstPtr& msg)
{
  // 对获取的信息进行计算，后发布信息
  std::vector<geometry_msgs::Pose> Poses;
  std::vector<geometry_msgs::Point> Points;
  std::vector<geometry_msgs::PoseStamped> PoseStamped;
  PoseStamped = msg->poses;
  std::vector<geometry_msgs::PoseStamped>::iterator it;
  for(it = PoseStamped.begin();it!=PoseStamped.end();++it){
    Poses.push_back(it->pose);
  }
  std::vector<geometry_msgs::Pose>::iterator it2;
  for(it2 = Poses.begin();it2!=Poses.end();++it2){
    Points.push_back(it2->position);
  }

  std::vector<geometry_msgs::Point>::iterator it3;
  for(it3 = Points.begin();it3!=Points.end();it3++){
    temp_map.push_back(*it3);
  }
    ROS_INFO_STREAM("get robot2's path.");
  map.data.assign(temp_map.begin(), temp_map.end());
}

void pathCallback_3(const nav_msgs::Path::ConstPtr& msg)
{
  // 对获取的信息进行计算，后发布信息
  std::vector<geometry_msgs::Pose> Poses;
  std::vector<geometry_msgs::Point> Points;
  std::vector<geometry_msgs::PoseStamped> PoseStamped;
  PoseStamped = msg->poses;
  std::vector<geometry_msgs::PoseStamped>::iterator it;
  for(it = PoseStamped.begin();it!=PoseStamped.end();++it){
    Poses.push_back(it->pose);
  }
  std::vector<geometry_msgs::Pose>::iterator it2;
  for(it2 = Poses.begin();it2!=Poses.end();++it2){
    Points.push_back(it2->position);
  }

  std::vector<geometry_msgs::Point>::iterator it3;
  for(it3 = Points.begin();it3!=Points.end();it3++){
    temp_map.push_back(*it3);
  }
  map.data.assign(temp_map.begin(), temp_map.end());
}

void pathCallback_4(const nav_msgs::Path::ConstPtr& msg)
{
  // 对获取的信息进行计算，后发布信息
  std::vector<geometry_msgs::Pose> Poses;
  std::vector<geometry_msgs::Point> Points;
  std::vector<geometry_msgs::PoseStamped> PoseStamped;
  PoseStamped = msg->poses;
  std::vector<geometry_msgs::PoseStamped>::iterator it;
  for(it = PoseStamped.begin();it!=PoseStamped.end();++it){
    Poses.push_back(it->pose);
  }
  std::vector<geometry_msgs::Pose>::iterator it2;
  for(it2 = Poses.begin();it2!=Poses.end();++it2){
    Points.push_back(it2->position);
  }

  std::vector<geometry_msgs::Point>::iterator it3;
  for(it3 = Points.begin();it3!=Points.end();it3++){
    temp_map.push_back(*it3);
  }
  map.data.assign(temp_map.begin(), temp_map.end());
}

void pathCallback_5(const nav_msgs::Path::ConstPtr& msg)
{
  // 对获取的信息进行计算，后发布信息
  std::vector<geometry_msgs::Pose> Poses;
  std::vector<geometry_msgs::Point> Points;
  std::vector<geometry_msgs::PoseStamped> PoseStamped;
  PoseStamped = msg->poses;
  std::vector<geometry_msgs::PoseStamped>::iterator it;
  for(it = PoseStamped.begin();it!=PoseStamped.end();++it){
    Poses.push_back(it->pose);
  }
  std::vector<geometry_msgs::Pose>::iterator it2;
  for(it2 = Poses.begin();it2!=Poses.end();++it2){
    Points.push_back(it2->position);
  }

  std::vector<geometry_msgs::Point>::iterator it3;
  for(it3 = Points.begin();it3!=Points.end();it3++){
    temp_map.push_back(*it3);
  }
  map.data.assign(temp_map.begin(), temp_map.end());
}

main (int argc, char **argv) // 声明主函数
{
    ros::init (argc, argv, "trimap");  // 初始化一个节点

    ros::NodeHandle nh; // 建立一个节点实体
    ros::Subscriber sub1 = nh.subscribe("/rob1/astar_path", 1, pathCallback_1);
    ros::Subscriber sub2 = nh.subscribe("/rob2/astar_path", 1, pathCallback_2);
    ros::Subscriber sub3 = nh.subscribe("/rob3/astar_path", 1, pathCallback_3);
    ros::Subscriber sub4 = nh.subscribe("/rob4/astar_path", 1, pathCallback_4);
    ros::Subscriber sub5 = nh.subscribe("/rob5/astar_path", 1, pathCallback_5);
    ros::Publisher pub = nh.advertise<trimap::Trimap>("/trimap",100);


    ros::Rate r(1.0); // 设置回调执行频率（每秒1次）
    while (ros::ok())
    {
        map.data.clear();
        temp_map.clear();
        ros::spinOnce();
        pub.publish(map);
        r.sleep();
    }
}
