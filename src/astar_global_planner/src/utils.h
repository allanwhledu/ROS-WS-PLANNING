//
// Created by lyunfan on 2020/1/16.
//

#ifndef ASTAR_GLOBAL_PLANNER_UTILS_H
#define ASTAR_GLOBAL_PLANNER_UTILS_H
using namespace std;

inline string intToString(int v) {
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%u", v);

    string str = buf;
    return str;
}

inline void perm(int arr[], int len, vector <vector<int>> &ret) {
    if (len < 2) return;
    int i, j, temp;
    do {
        auto *tmp = new vector<int>();
        for (int k = 0; k < len; ++k) {
            tmp->push_back(arr[k]);
        }
        ret.push_back(*tmp);
        i = j = len - 1;
        //向前查找第一个变小的元素
        while (i > 0 && arr[i] < arr[i - 1]) --i;
        temp = i;
        if (i == 0) break;
        //先后查找第一个比arr[i-1]大的元素
        while (temp + 1 < len && arr[temp + 1] > arr[i - 1]) ++temp;
        swap(arr[i - 1], arr[temp]);  //交换两个值
        reverse(arr + i, arr + len);  //逆序
    } while (true);
}


inline unsigned int DistManhattan(int d_x, int d_y, int x, int y) {
    return (std::abs(d_x - x) + std::abs(d_y - y)) * DISTANCE;
}

#endif //ASTAR_GLOBAL_PLANNER_UTILS_H
