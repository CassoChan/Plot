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

vector<float> OriginOffset;

vector<float> x;
vector<float> y;
vector<float> angle;
vector<float> ValidPoint;

float to_float(string s)
{
    int i = 0, n = 0;
    int point_index = s.find('.');
    float result = 0, under_0 = 0; // under_0存储小数部分
    if (count(s.begin(), s.end(), '.') > 1)
    {
        return 0; // 字符串里只能有1个或0个小数点，不然出错退出
    }
    if (s.find('.') != -1) // 字符串里有小数点
    {
        if (point_index >= 1 && point_index < s.size() - 1) // 小数点位置合理，不能在字符串第1位,且不能在最后一位
        {
            for (i = 0; i <= point_index - 1; i++) // 计算整数部分
            {
                if (s[i] >= '0' && s[i] <= '9')
                {
                    result = result * 10 + s[i] - 48;
                }
            }
            for (i = s.size() - 1; i >= point_index - 1; i--) // 计算小数部分
            {
                if (s[i] >= '0' && s[i] <= '9')
                {
                    if (i == point_index - 1)
                    {
                        under_0 = under_0 * 0.1 + 0; // i=小数点前一位，under_0+0
                    }
                    else
                    {
                        under_0 = under_0 * 0.1 + s[i] - 48;
                    }
                }
            }
            result = result + under_0; // 把整数部分和小数部分相加
        }
    }
    else // 字符串只有整数部分
    {
        for (i = 0; i <= s.size() - 1; i++)
        {
            if (s[i] >= '0' && s[i] <= '9')
            {
                result = result * 10 + s[i] - 48;
            }
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    /**
     * 录入地图
     * rows: 行数，对应y坐标;
     * cols: 列数，对应x坐标;
     * opencv，x轴向右，y轴向下;
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
                map_y.push_back(-(i * resolution - origin_y) +50);
                map_color.push_back(255.0 - (float)temp);
            }
        }
    }
    plt::figure_size(dstImage.cols, dstImage.rows);
    std::map<string, string> keywords;
    keywords["cmap"] = "Greys";
    plt::scatter_colored(map_x, map_y, map_color, 30, keywords);
    // plt::save("/home/casun/plot/drawing/map.png");
    // plt::show();

    /**
     * 读入原点坐标的修改
     */
    ifstream OriginFile("/home/casun/PoseData/OriginPose/OriginPose.txt");
    if (!OriginFile)
    {
        cerr << "Cannot open origin's file.\n";
        return 1;
    }
    OriginFile.seekg(0, ios::end);
    char ch;
    do
    {
        OriginFile.seekg(-2, ios::cur);
        if ((int)OriginFile.tellg() <= 0)
        {
            OriginFile.seekg(0);
            break;
        }
        OriginFile.get(ch);
    } while (ch != '\n');
    string lastLine;
    while (getline(OriginFile, lastLine, ','))
    {
        OriginOffset.push_back(to_float(lastLine));
    }
    OriginFile.close();

    Eigen::Vector3d t1 = Eigen::Vector3d(OriginOffset[0], OriginOffset[1], OriginOffset[2]);
    Eigen::Quaterniond q1(OriginOffset[3], OriginOffset[4], OriginOffset[5], OriginOffset[6]);

    /**
     * 轨迹热力图
     */
    ifstream TrajectoryFile("/home/casun/PoseData/Trajectory/2024-03-22 14_35_49.txt");
    if (!TrajectoryFile)
    {
        cerr << "Cannot open trajectory's file.\n";
        return 1;
    }
    string line;
    vector<string> v;
    vector<string> time;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while (getline(TrajectoryFile, line))
    {
        stringstream ss(line);
        string tmp;
        while (getline(ss, tmp, ','))
        {
            v.push_back(tmp);
        }
    }

    for (auto i = v.begin(); i != v.end();)
    {
        // time.push_back(*i++);
        // x.push_back(stof(*i++));
        // y.push_back(stof(*i++));
        // angle.push_back(stof(*i++));
        time.push_back(*i++);
        Eigen::Vector3d p1 = Eigen::Vector3d(stof(*i++), stof(*i++), 0.0);
        Eigen::Vector3d pw;
        pw = q1 * p1 + t1;
        x.push_back(pw[1]);
        y.push_back(pw[0]);
        angle.push_back(stof(*i++));
    }

    cloud->width = x.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = x[i];
        cloud->points[i].y = y[i];
        cloud->points[i].z = 0;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        int b = kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        ValidPoint.push_back(b);
    }

    std::map<string, string> HeatMap_keywords;
    HeatMap_keywords["cmap"] = "jet";

    // plt::figure_size(1920, 1080);
    plt::scatter_colored(x, y, ValidPoint, PointSize, HeatMap_keywords);
    plt::title("Trajectory");

    std::string folderpath = "/home/casun/plot/drawing";
    if (access(folderpath.c_str(), 0) == -1)
    {
        mkdir(folderpath.c_str(), 0777);
    }
    plt::save("/home/casun/plot/drawing/integration.png");
    plt::show();
}