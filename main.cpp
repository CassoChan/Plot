#include "matplotlibcpp.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <vector>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

namespace plt = matplotlibcpp;
using namespace std;
using namespace cv;

#define radius 0.4
#define PointSize 30
#define origin_x -15.093980
#define origin_y -28.889954
// #define origin_x 0
// #define origin_y 0
#define resolution 0.050000

vector<float> map_x;
vector<float> map_y;
vector<float> map_color;

vector<float> x;
vector<float> y;
vector<float> angle;
vector<float> ValidPoint;

int main(int argc, char **argv)
{
    /**
     * 录入地图
     */
    Mat img = imread("/home/casun/PoseData/map1.pgm");
    cv::Mat dstImage = img.clone();
    for (size_t i = 0; i < dstImage.rows; i++)
    {
        for (size_t j = 0; j < dstImage.cols; j++)
        {
            Vec3b intensity = dstImage.at<Vec3b>(i, j);
            uchar temp = intensity.val[0];
            if (temp != 0 && temp != 128)
            {
                map_x.push_back(j * resolution + origin_x);
                map_y.push_back(-(i * resolution + origin_y));
                map_color.push_back(255.0 - (float)temp);
            }
        }
    }
    plt::figure_size(dstImage.cols, dstImage.rows);
    std::map<string, string> keywords;
    keywords["cmap"] = "Greys";
    plt::scatter_colored(map_x, map_y, map_color, 30, keywords);
    plt::save("/home/casun/plot/drawing/map.png");
    // plt::show();

    /**
     * 轨迹热力图
     */
    // ifstream in("/home/casun/PoseData/2024-03-13 09_14_22.txt");
    // string line;
    // vector<string> v;
    // vector<string> time;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // while (getline(in, line))
    // {
    //     stringstream ss(line);
    //     string tmp;
    //     while (getline(ss, tmp, ','))
    //     {
    //         v.push_back(tmp);
    //     }
    // }

    // for (auto i = v.begin(); i != v.end();)
    // {
    //     time.push_back(*i++);
    //     x.push_back(stof(*i++));
    //     y.push_back(stof(*i++));
    //     angle.push_back(stof(*i++));
    // }

    // cloud->width = x.size();
    // cloud->height = 1;
    // cloud->points.resize(cloud->width * cloud->height);

    // for (size_t i = 0; i < cloud->points.size(); i++)
    // {
    //     cloud->points[i].x = x[i];
    //     cloud->points[i].y = y[i];
    //     cloud->points[i].z = 0;
    // }

    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud(cloud);

    // // pcl::PointXYZ searchPoint;
    // vector<int> pointIdxRadiusSearch;
    // vector<float> pointRadiusSquaredDistance;
    // for (size_t i = 0; i < cloud->points.size(); i++)
    // {
    //     // searchPoint.x = x[i];
    //     // searchPoint.y = y[i];
    //     // searchPoint.z = 0;
    //     // int b = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    //     int b = kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    //     ValidPoint.push_back(b);
    // }

    // std::map<string, string> keywords;
    // keywords["cmap"] = "jet";

    // plt::figure_size(1920, 1080);
    // plt::scatter_colored(x, y, ValidPoint, PointSize, keywords);
    // plt::title("Trajectory");

    // std::string folderpath = "/home/casun/plot/drawing";
    // if (access(folderpath.c_str(), 0) == -1)
    // {
    //     mkdir(folderpath.c_str(), 0777);
    // }
    // plt::save("/home/casun/plot/drawing/basic.png");
    // plt::show();
}