#include <matplotlibcpp17/pyplot.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;

#define PI 3.1415926

vector<float> map_x;
vector<float> map_y;
vector<float> map_color;

bool inverse_flag = false;
vector<float> OriginOffset;
vector<float> OriginOffset1;

vector<float> x;
vector<float> y;
vector<float> angle;
vector<float> ValidPoint;
bool check = true;

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
                if (s[i] == '-')
                {
                    inverse_flag = true;
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

            if (inverse_flag == true)
            {
                result = -(result + under_0); // 把整数部分和小数部分相加
            }
            else
            {
                result = result + under_0; // 把整数部分和小数部分相加
            }
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
        config = YAML::LoadFile("../config.yaml");
    }
    catch (YAML::BadFile &e)
    {
        std::cout << "config.yaml read error!" << std::endl;
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
    string OutputFileName = config["OutputFileName"].as<string>();

    pybind11::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();

    /**
     * @brief 读入原点坐标的修改
     */
    std::string file = config["OriginFile_address"].as<string>();
    if (access(file.c_str(), 0) == -1)
    {
        OriginOffset1 = {0, 0, 0, 1, 0, 0, 0};
        cout << "No Origin file, the default origin pose is used : t[0, 0, 0] , q[1, 0, 0, 0]." << endl;
    }
    else
    {
        ifstream OriginFile(file);
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
            OriginOffset1.push_back(to_float(lastLine));
            // cout << to_float(lastLine) << endl;
        }
        OriginFile.close();
    }

    OriginOffset = OriginOffset1;
    OriginOffset1[0] = -OriginOffset1[0];
    OriginOffset1[1] = -OriginOffset1[1];

    Eigen::Quaterniond q1(OriginOffset1[3], OriginOffset1[4], OriginOffset1[5], OriginOffset1[6]);
    Eigen::Vector3d Euler = q1.matrix().eulerAngles(2, 1, 0);
    OriginOffset[0] = sqrt(pow(OriginOffset1[0], 2) + pow(OriginOffset1[1], 2)) * cos(atan2(OriginOffset1[1], OriginOffset1[0]) + Euler[0]);
    OriginOffset[1] = sqrt(pow(OriginOffset1[0], 2) + pow(OriginOffset1[1], 2)) * sin(atan2(OriginOffset1[1], OriginOffset1[0]) + Euler[0]);
    Eigen::Vector3d t1 = Eigen::Vector3d(OriginOffset[0], OriginOffset[1], OriginOffset[2]);

    /**
     * 读取地图
     */
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
                float tmp_x = j * resolution + origin_x + OffSetByHand_x;
                float tmp_y = -(i * resolution - origin_y) + OffSetByHand_y;
                map_x.push_back(tmp_x * cos(Euler[0]) - tmp_y * sin(Euler[0]) + OriginOffset[0]);
                map_y.push_back(tmp_x * sin(Euler[0]) + tmp_y * cos(Euler[0]) + OriginOffset[1]);
                map_color.push_back(255.0 - (float)temp);
            }
        }
    }
    if ((Euler[0] >= 0 && Euler[0] <= PI / 90.0) || (Euler[0] >= PI * (1 - 1 / 90.0) && Euler[0] <= PI * (1 + 1 / 90.0)) || Euler[0] >= PI * (2 - 1 / 90.0))
    {
        cout << "Model1--" << Euler[0] / PI * 180 << endl;
        plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(dstImage.cols / 10.0, dstImage.rows / 10.0)));
    }
    else if ((Euler[0] >= PI * (0.5 - 1 / 90.0) && Euler[0] <= PI * (0.5 + 1 / 90.0)))
    {
        cout << "Model2--" << Euler[0] / PI * 180 << endl;
        plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(dstImage.rows / 10.0, dstImage.cols / 10.0)));
    }
    else
    {
        cout << "The angle value is not within the appropriate range." << endl;
        return 0;
    }

    plt.scatter(Args(map_x, map_y),
                Kwargs("c"_a = map_color, "s"_a = MapPointSize, "cmap"_a = "Greys"));

    /**
     * 读取轨迹位姿
     */
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
        float co_x = stof(*i++);
        float co_y = stof(*i++);
        Eigen::Vector3d p1 = Eigen::Vector3d(co_x, co_y, 0.0); // 不要用指针，x86会从右往左读，导致先计算p[1]，再计算p[0];
        Eigen::Vector3d pw;
        pw = q1 * p1 + t1;
        x.push_back(pw[0]);
        y.push_back(pw[1]);
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

    // 初始化后的原点
    plt.scatter(Args(0, 0),
                Kwargs("c"_a = "r", "s"_a = MapPointSize * 100, "marker"_a = "*", "label"_a = "inital_origin"));
    // 图片原点
    plt.scatter(Args(OriginOffset[0], OriginOffset[1]),
                Kwargs("c"_a = "g", "s"_a = MapPointSize * 100, "marker"_a = ".", "label"_a = "origin"));

    // 箭头指向，坐标变换用，图片原点指向初始化后原点
    float dx = 0 - OriginOffset[0];
    float dy = 0 - OriginOffset[1];
    plt.quiver(Args(OriginOffset[0], OriginOffset[1], dx, dy),
               Kwargs("color"_a = "y", "angles"_a = "xy", "scale"_a = 1.03, "scale_units"_a = "xy", "width"_a = 0.005));

    auto cs = plt.scatter(Args(x, y),
                          Kwargs("c"_a = ValidPoint, "s"_a = TPointSize, "cmap"_a = "jet"));

    plt.title(Args("Trajectory"));
    plt.colorbar(Args(cs.unwrap()));
    plt.legend();

    std::string folderpath = "../drawing";
    if (access(folderpath.c_str(), 0) == -1)
    {
        mkdir(folderpath.c_str(), 0777);
    }
    string Output = folderpath + "/" + OutputFileName + ".png";
    plt.savefig(Args(Output));
    return 0;
}