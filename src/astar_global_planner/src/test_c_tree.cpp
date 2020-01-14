#include <ros/ros.h>
#include "tree.hh"

using std::string;
using std::cin;
using std::cout;
using std::endl;

struct pattern
{
    int robot1;
    int robot2;
};

struct align
{
    int value;
    pattern* pattern_mode;
};


class SubscribeAndPublish
{
public:
//    SubscribeAndPublish()
//    {
//        //Topic you want to publish
//        pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
//
//        //Topic you want to subscribe
//        sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
//    }
//
//    void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
//    {
//        PUBLISHED_MESSAGE_TYPE output;
//        //.... do something with the input and generate the output...
//        pub_.publish(output);
//    }

private:
    ros::NodeHandle n_;
//    ros::Publisher pub_;
//    ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "subscribe_and_publish");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    tree<string> tr;
    tree<string>::iterator top, one, two, loc, banana;

//    top=tr.begin();
//    one=tr.insert(top, "one");
//
//    two=tr.append_child(top, "two");
//    tr.append_child(two, "apple");
//    banana=tr.append_child(two, "banana");
//    tr.append_child(banana,"cherry");
//    tr.append_child(two, "peach");
//    tr.append_child(one,"three");

//    loc=find(tr.begin(), tr.end(), "one");
//    if(loc!=tr.end()) {
//        tree<string>::sibling_iterator sib=tr.begin(loc);
//        while(sib!=tr.end(loc)) {
//            cout << (*sib) << endl;
//            ++sib;
//        }
//        cout << endl;
//        tree<string>::iterator sib2=tr.begin(loc);
//        tree<string>::iterator end2=tr.end(loc);
//        while(sib2!=end2) {
//            for(int i=0; i<tr.depth(sib2)-1; ++i)
//                cout << " ";
//            cout << (*sib2) << endl;
//            ++sib2;
//        }
//    }
//
////    cout << (*one) << endl;


    tree<align> tr2;
    tree<align>::iterator top2, one2;

    pattern p_tmp;
    p_tmp.robot1 = 1;
    p_tmp.robot2 = 2;

    align first;
    first.value=32;
    first.pattern_mode = &p_tmp;

    top2=tr2.begin();
    one2=tr2.insert(top2,first);

    cout << one2->pattern_mode->robot1 << endl;






//    int *p;
//    int g;
//    p=&g;
//
//    cout<<*p<<endl;
//
//    *p = 4;
//    cout<<*p<<endl;

    ros::spin(); //并不是类的实例化会运行函数，而是这里的回调函数会一直运行callback那里的代码

    return 0;
}

