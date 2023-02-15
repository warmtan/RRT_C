#ifndef _RRT_H
#define _RRT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <typeinfo>
#include <time.h>

using namespace std;
using namespace cv;

class node {
private:
    float x, y;                // 节点坐标
    vector<float> pathX, pathY;// 路径
    node* parent;              // 父节点
    float cost;
public:
    node(float _x, float _y);
    float getX();
    float getY();
    void setParent(node*);
    node* getParent();
};

class RRT {
private:
    node* startNode, * goalNode;          // 起始节点和目标节点
    vector< vector<float> > obstacleList; // 障碍物
    vector<node*> nodeList;               // 
    float stepSize;                       // 步长

    int goal_sample_rate;

    // 随机函数产生的是一种伪随机数，它实际是一种序列发生器，有固定的算法，只有当种子不同时，序列才不同，
    // 所以不应该把种子固定在程序中，应该用随机产生的数做种子，如程序运行时的时间等。
    random_device goal_rd;                // random_device可以生成用来作为种子的随机的无符号整数值。
    mt19937 goal_gen;                     // mt19937是一种高效的随机数生成算法
    uniform_int_distribution<int> goal_dis;  //随机数源，随机数源调用随机数算法来生成随机数

    random_device area_rd;
    mt19937 area_gen;
    uniform_real_distribution<float> area_dis;
public:
    RRT(node*, node*, const vector<vector<float>>&, float , int);
    node* getNearestNode(const vector<float>&);
    bool collisionCheck(node*);
    vector<node*> planning();
};

#endif
