// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

template <class Type>
Type stringToNum(const string& str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

// 主函数
int main()
{
    // 图像矩阵
    cv::Mat rgb, left;
    // 使用cv::imread()来读取图像
    //该图像为双目摄像机左目所拍摄的图像
    rgb = cv::imread( "/home/maki/code/ELAS/img/raindeer_left.pgm");

    //指定数据文件
    ifstream myfile ("/home/maki/code/ELAS/data.txt",ios::in);
    if(!myfile){
      cout << "Unable to open myfile";
            exit(1); // terminate with error

    }

    int w=rgb.cols;
    int h=rgb.rows;

    float data[w*h+1];
    string line;

    int c=0;
    while (getline (myfile, line)) // line中不包括每行的换行符
    {
               string aa=line;
               data[c]=stringToNum<double>(aa);
               c++;
     }


    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud(new PointCloud);
    // 遍历视差图
    for (int m = 0; m < h; m++)
    for (int n = 0; n < w; n++)
    {

        // 获取深度图中(m,n)处的值
        float d = data[m*1342+n];
        //cout << d << endl;



        // d 可能没有值，若如此，跳过此点
        if (d == 0)
            continue;
        // d 存在值，则向点云增加一个点
        d=((camera_fx+camera_fy)/2)*50/d;

        PointT p;

        // 计算这个点的空间坐标
        p.z = float(d) / camera_factor;
        //p.z = d;
        p.x = (n - camera_cx) * p.z / camera_fx;
        p.y = (m - camera_cy) * p.z / camera_fy;

        // 从rgb图像中获取它的颜色
        // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
        p.b = rgb.ptr<uchar>(m)[n*3];
        p.g = rgb.ptr<uchar>(m)[n*3+1];
        p.r = rgb.ptr<uchar>(m)[n*3+2];


        // 把p加入到点云中
        cloud->points.push_back(p);
    }

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile("/home/maki/code/change/data/pointcloud.pcd", *cloud);

    //可视化
    //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //viewer.ShowCloud(cloud);

    //while (!viewer.wasStopped())
    //{
    //viewer.spinOnce();
    //}
    // 清除数据并退出
    cloud->points.clear();
    cout << "Point cloud saved." << endl;
    return 0;
}

