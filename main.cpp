#include "matplotlibcpp.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <vector>
#include <sstream>

namespace plt = matplotlibcpp;
using namespace std;

vector<float> x;
vector<float> y;
vector<string> color;
vector<float> angle;

int main(int argc, char **argv)
{
    int count = 0;
    ifstream in("/home/casun/PoseData/2024-03-05 09_38_38.txt");
    string line;
    vector<string> v;
    vector<string> time;
    while (getline(in, line))
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
        x.push_back(stof(*i++));
        y.push_back(stof(*i++));
        color.push_back("b");
        angle.push_back(stof(*i++));
        count++;
    }

    plt::figure_size(1200, 780);
    plt::scatter_colored(x, y, color, 10);
    plt::title("Trajectory");

    std::string folderpath = "/home/casun/plot/drawing";
    if (access(folderpath.c_str(), 0) == -1){
        mkdir(folderpath.c_str(), 0777);
    }
    plt::save("/home/casun/plot/drawing/basic.png");
}