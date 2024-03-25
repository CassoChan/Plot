#include <matplotlibcpp17/pyplot.h>
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

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;

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

int main()
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile("/home/casun/plot3/config.yaml");
    }
    catch (YAML::BadFile &e)
    {
        std::cout << "read error!" << std::endl;
        return -1;
    }

    float radius = config["radius"].as<float>();
    float TPointSize = config["TPointSize"].as<float>();
    float MapPointSize = config["MapPointSize"].as<float>();
    float origin_x = config["origin_x"].as<float>();
    float origin_y = config["origin_y"].as<float>();
    float resolution = config["resolution"].as<float>();
    float OffSetByHand_x = config["OffSetByHand_x"].as<float>();
    float OffSetByHand_y = config["OffSetByHand_y"].as<float>();

    pybind11::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();

    Mat img = imread(config["MapFile_address"].as<string>());
    cv::Mat dstImage = img.clone();
    for (size_t i = 0; i < dstImage.rows; i++)
    {
        for (size_t j = 0; j < dstImage.cols; j++)
        {
            Vec3b intensity = dstImage.at<Vec3b>(i, j);
            uchar temp = intensity.val[0];
            if (temp != 0 && temp != 128)
            {
                map_x.push_back(j * resolution + origin_x + OffSetByHand_x);
                map_y.push_back(-(i * resolution - origin_y) + OffSetByHand_y);
                map_color.push_back(255.0 - (float)temp);
            }
        }
    }
    plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(dstImage.cols / 10.0, dstImage.rows / 10.0)));
    plt.scatter(Args(map_x, map_y),
               Kwargs("c"_a = map_color, "s"_a = MapPointSize, "cmap"_a = "Greys"));

    // /**
    //  * 读入原点坐标的修改
    //  */
    ifstream OriginFile(config["OriginFile_address"].as<string>());
    assert(OriginFile);
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

    ifstream TrajectoryFile(config["TrajectoryFile_address"].as<string>());
    assert(TrajectoryFile);
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

    plt.scatter(Args(x, y),
               Kwargs("c"_a = ValidPoint, "s"_a = TPointSize, "cmap"_a = "jet"));

    plt.title(Args("Trajectory"));

    // plt.colorbar(Args(ValidPoint.unwrap()), Kwargs("ax"_a = plt.unwrap()));

    std::string folderpath = "/home/casun/plot3/drawing";
    if (access(folderpath.c_str(), 0) == -1)
    {
        mkdir(folderpath.c_str(), 0777);
    }
    plt.savefig(Args("/home/casun/plot3/drawing/integration.png"));
    // plt.show();
    return 0;
}