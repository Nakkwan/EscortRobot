//
// Created by lihao on 19-7-9.
//

#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


namespace astaralgorithm{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

struct Node{
    Point point;  // node coordinate
    int F, G, H;  // cost
    Node* parent; // parent node

    Node(Point _point = Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

struct AstarConfig{
    bool Euclidean;         // true/false
    int OccupyThresh;       // 0~255
    int InflateRadius;      // integer

    AstarConfig(bool _Euclidean = true, int _OccupyThresh = -1, int _InflateRadius = -1):
        Euclidean(_Euclidean), OccupyThresh(_OccupyThresh), InflateRadius(_InflateRadius)
    {
    }
};

class Astar{

public:
    // Interface function
    void InitAstar(Mat& _Map, AstarConfig _config = AstarConfig());
    void InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config = AstarConfig());

    /*
    @brief: set and return path
    @pre: set start, target point for navigation
    @post: path is return by reference
    @param: start location(_Point), target location(_Point), path(vector<Point>&)
    */
    void PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path);
private:

    /*
    @brief: make map simple for planning
    @pre: get initial map
    @post: make map grayscale, binary, masking noise
    @param: mask(_Mat) for masking noise pass by reference
    */
    void MapProcess(Mat& Mask);

    /*
    @brief:find path
    @pre: set map, start, target point
    @post: path is set in closeList
    */
    Node* FindPath();
    
    /*
    @brief: get path
    @pre: execute findPath
    @post: path is set pathList
    */
    void GetPath(Node* TailNode, vector<Point>& path);

private:
    //Object
    Mat Map;
    Point startPoint, targetPoint;
    Mat neighbor;

    Mat LabelMap;
    AstarConfig config;
    vector<Node*> OpenList;  // open list
    vector<Node*> PathList;  // path list

    int count;
    int TotalLength;
};

}




#endif //ASTAR_H
