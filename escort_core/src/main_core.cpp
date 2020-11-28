#include <string.h>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "../include/escort_core/Astar.h"
#include "../include/escort_core/OccMapTransform.h"

#define ROBOT_SCALE_ACCORD_RESOLUTION 5.0

using namespace cv;
using namespace std;


//-------------------------------- Global variables ---------------------------------//
// Subscriber
//ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
//ros::Subscriber map_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
astaralgorithm::AstarConfig config;
astaralgorithm::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;

// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag = false;
bool mask_flag = true;
int rate;
int height;
int width;
int OccProb;
Mat mappp = imread("map.pgm", CV_LOAD_IMAGE_GRAYSCALE);

void Initialize()
{
    mappp.convertTo(mappp, CV_8UC1, 100.0/255.0);
    width = mappp.cols;
    height = mappp.rows;

    ROS_INFO("width = %d, height = %d", width, height);

//    for(int i=0;i<height;i++)
//    {
//        for(int j=0;j<width;j++)
//        {
//            OccProb = mappp.at<uchar>(i * width + j);
//            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0

//            // The origin of the OccGrid is on the bottom left corner of the map
//            mappp.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
//            //ROS_INFO("%d", mappp.at<uchar>(i,j));
//        }
//    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(ROBOT_SCALE_ACCORD_RESOLUTION);
    astar.InitAstar(mappp, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
//    OccGridMask.info = msg.info;
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
//    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
//    OccGridParam.Map2ImageTransform(src_point, startPoint);
    startPoint.x = msg.pose.pose.position.x;
    startPoint.y = msg.pose.pose.position.y;

    // Set flag
    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
        mask_flag = true;
    }

    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
             startPoint.x, startPoint.y);
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
//    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
//    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    targetPoint.x = msg.pose.position.x;
    targetPoint.y = msg.pose.position.y;

    // Set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
        mask_flag = true;
    }

    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
             targetPoint.x, targetPoint.y);
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char** argv)
{
    //  Initial node
    ros::init(argc, argv, "escort_core");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    Initialize();

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);

    // Subscribe topics
//    map_sub = nh.subscribe("map_Grid", 10, MapCallback);
    startPoint_sub = nh.subscribe("/application/start_loc", 10, StartPointCallback);
    targetPoint_sub = nh.subscribe("/application/target_loc", 10, TargetPointtCallback);

    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("/escort_core/map_mask", 1);
    path_pub = nh.advertise<nav_msgs::Path>("/escort_core/map_path", 10);

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(start_flag)
        {
            // Start planning path
            vector<Point> PathList;
            astar.PathPlanning(startPoint, targetPoint, PathList);
            if(!PathList.empty())
            {
                ROS_INFO("At PathList");
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                for(int i=0;i<PathList.size();i++)
                {
//                    Point2d dst_point;
//                    OccGridParam.Image2MapTransform(PathList[i], dst_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
//                    pose_stamped.pose.position.x = dst_point.x;
//                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.x = PathList[i].x;
                    pose_stamped.pose.position.y = PathList[i].y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                ROS_INFO("Find a valid path successfully");
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            start_flag = false;
        }

        if(map_flag && mask_flag)
        {
            mask_pub.publish(OccGridMask);
            mask_flag = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}

