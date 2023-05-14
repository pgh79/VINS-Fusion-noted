/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10
using namespace std;


/*
定义了一些变量和数据结构：

第一行定义了一个类型为 queue<sensor_msgs::ImageConstPtr> 的变量 image_buf，这个变量是一个队列，用于存储 ROS 系统中传感器的图像数据。

第二行定义了一个类型为 queue<sensor_msgs::PointCloudConstPtr> 的变量 point_buf，这个变量也是一个队列，用于存储 ROS 系统中传感器的点云数据。

第三行定义了一个类型为 queue<nav_msgs::Odometry::ConstPtr> 的变量 pose_buf，同样是一个队列，用于存储 ROS 系统中无人机的位置和姿态数据。

第四行定义了一个类型为 queue<Eigen::Vector3d> 的变量 odometry_buf，同样是一个队列，用于存储计算得到的无人机在三维空间中的位移信息。

第五行和第六行定义了两个互斥锁，m_buf 和 m_process，用于保护这些队列的读写操作，防止多线程并发访问发生竞态条件。

第七行和第八行定义了一些变量，包括 frame_index、sequence、posegraph、skip_first_cnt、SKIP_CNT、skip_cnt、load_flag、start_flag、
SKIP_DIS，它们被用于控制算法的运行参数和状态等。

第九行到第十七行定义了一些参数，包括 VISUALIZATION_SHIFT_X、VISUALIZATION_SHIFT_Y、ROW、COL、DEBUG_IMAGE、m_camera、tic、qic、
pub_match_img、pub_camera_pose_visual、pub_odometry_rect、BRIEF_PATTERN_FILE、POSE_GRAPH_SAVE_PATH、VINS_RESULT_PATH、
cameraposevisual、last_t、last_image_time、pub_point_cloud、pub_margin_cloud，它们被用于控制程序的输入输出、图像的可视化等。
*/

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;

/*
创建一个新的序列。在此序列中，会首先打印出 "new sequence" 这个消息，然后将变
量 sequence 的值加一，并打印出当前 sequence 的计数器值（即 sequence 变量的值）。
接着，会检查 sequence 是否大于 5，如果是的话，则会打印 "only support 5 sequences 
since it's boring to copy code for more sequences." 这个 ROS_WARN 错误消息，并且使用
 ROS_BREAK 函数终止程序。 如果 sequence 变量的值小于或等于 5，那么程序会执行以
下操作： - 首先会重置姿态图并进行可视化。 - 接着会发布姿态图。 - 然后会对图片缓存、
点云缓存、姿态缓存和里程计缓存进行操作，将它们全部清空。 最后，线程锁被释放，该
函数执行结束。
*/
void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}
/*
回调函数，用于处理从相机传递过来的图像消息。以下是代码的逐行解释：

匹配一个 sensor_msgs::ImageConstPtr 类型的 image_msg 参数。

线程锁定，避免其他线程对缓存进行随意修改。

将相机传来的图像消息压入 image_buf 缓存。

释放线程锁。

打印出图像消息的时间。

检测相机流是否不稳定。

如果 last_image_time 为-1，则表明这是第一次检测到来自相机的图像消息，我们将 last_image_time 设置为当前图像消息的时间戳。

否则，我们将比较新来的图像消息时间戳与上一条的时间戳。如果它们之间的差大于 1.0 秒，或者新来的时间戳小于上一条的时间戳，
则说明相机流不稳定，会发出警告信息，并调用 new_sequence 函数开始新的序列。

最后，将 last_image_time 更新为当前图像消息的时间戳。
*/
void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    //ROS_INFO("image_callback!");
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}
/*
第一行：定义了一个名为point_callback的函数，并将输入类型设定为sensor_msgs::PointCloudConstPtr的常指针。

第三行：输出一条ROS消息，表示进入了point_callback函数。

第四行：使用m_buf的互斥锁来锁定point_buf。这是为了避免多个线程同时访问point_buf，从而导致竞态条件。

第五行：将输入的点云数据point_msg推入point_buf中，以便后续处理。

第六至17行：这是注释掉的代码块。如果被取消注释，则该代码块将遍历输入点云的每个点，并输出其三维坐标和二维值。
输出的内容中包括i、3D点坐标、2D点坐标等。

第19至23行：定义sensor_msgs::PointCloud类型的变量point_cloud，并将其header设置为输入点云的header。

第25至32行：使用for循环遍历输入点云的每个点。首先，将该点的三维坐标存储在名为p_3d的cv::Point3f类型变量中。
接下来，使用姿态图posegraph的平移矢量t_drift和光滑矩阵r_drift对该点进行变换，并将变换后的结果存储在名为tmp的Eigen::Vector3d类型变量中。
最后，将变换后的结果存储在名为p的geometry_msgs::Point32类型变量中，然后将其添加到point_cloud中。

第34行：使用pub_point_cloud的发布器将point_cloud发布出去，以便进行可视化处理。
*/
void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);
}

/*
ROS节点中的一个函数，用于处理sensor_msgs/PointCloud类型的消息。

void是C++中的一个关键字，表示这个函数不返回任何值。

margin_point_callback函数的参数是一个指向sensor_msgs::PointCloud类型的常量指针，被命名为point_msg。

函数中新建了一个类型为sensor_msgs::PointCloud的变量point_cloud，用来存储处理后的点云数据。
同时，将point_msg的header字段复制给point_cloud的header字段。

接下来，使用for循环遍历point_msg发布的点云数据，对其中的每个点做如下的操作：

1.新建一个类型为cv::Point3f的变量p_3d，用来存储当前处理的点的3D坐标。

2.将点云消息point_msg中的第i个点的x、y、z坐标信息分别存储到p_3d的x、y、z字段中。

3.使用Eigen库中的Eigen::Vector3d类型创建一个名为tmp的向量，这个向量是通过将p_3d的x、y、z值组装成一个三维向量而得到的。

4.对tmp向量进行计算，将其乘以posegraph.r_drift 并加上posegraph.t_drift，得到一个新的向量，
然后将其分别存储到geometry_msgs::Point32类型的变量p中的x、y、z字段中。

5.将p添加到point_cloud的点云数据中。

最后，将处理过的点云数据point_cloud发布给话题pub_margin_cloud。
*/
// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_margin_cloud.publish(point_cloud);
}
/*
ROS节点的回调函数，它会在接收到来自导航消息的位置和姿态信息（nav_msgs::Odometry）时调用该函数。

定义了一个名为pose_callback的函数，该函数有一个名为pose_msg的参数，
其类型为const nav_msgs::Odometry::ConstPtr &，这个类型是一个常量指针，指向nav_msgs::Odometry消息。

接下来，使用m_buf锁保护了该函数的关键部分，pose_buf.push（pose_msg）将接收到的导航信息数据保存到pose_buf队列中。

使用m_buf解锁的方式来确保对同一个pose_buf变量的访问是互斥的，以避免多线程访问的冲突。

最后是一段注释掉的printf语句，它可以在控制台上打印出接收到的位置和姿态信息中各个变量的值。

整个pose_callback函数的作用是将ROS消息传递给相应的处理单元以供进一步处理，同时确保线程安全。
*/
void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}
/*
ROS函数，定义为 vio_callback。该函数的输入参数是一个 nav_msgs::Odometry 类型的指针 pose_msg。

第2-5行，将消息中的位置信息和方向信息分别提取出来，分别存入一个 Vector3d 类型的变量 vio_t 和一个 Quaterniond 类型的变量 vio_q 中。
其中，pose_msg->pose.pose.position.x 表示位置信息的x坐标，pose_msg->pose.pose.orientation.w 表示方向信息的w分量。

第7-8行，使用全局变量 posegraph 中存储的旋转矩阵 w_r_vio 和平移向量 w_t_vio，将 vio_t 和 vio_q 从相机坐标系转换到了世界坐标系。

第10-11行，使用全局变量 posegraph 中存储的旋转矩阵 r_drift 和平移向量 t_drift，将 vio_t 和 vio_q 进行了漂移校正，
即根据当前时刻之前的位姿信息，对当前时刻的位姿进行调整。

第13-21行，将校正后的位置信息和方向信息分别存入一个新的 nav_msgs::Odometry 类型的变量 odometry 中，并将其发布出去。

第23-24行，根据相机到IMU的转换矩阵 tic 和旋转矩阵 qic，将 vio_t 和 vio_q 从IMU坐标系转换到相机坐标系，并存入新的变量 vio_t_cam 和 vio_q_cam 中。

第26-28行，使用RViz可视化 cameraposevisual 中保存的相机位姿信息，并将其发布出去，消息头信息与原始消息一致。
*/
void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = vio_t.x();
    odometry.pose.pose.position.y = vio_t.y();
    odometry.pose.pose.position.z = vio_t.z();
    odometry.pose.pose.orientation.x = vio_q.x();
    odometry.pose.pose.orientation.y = vio_q.y();
    odometry.pose.pose.orientation.z = vio_q.z();
    odometry.pose.pose.orientation.w = vio_q.w();
    pub_odometry_rect.publish(odometry);

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    cameraposevisual.reset();
    cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);


}
/*
定义了一个名为 extrinsic_callback 的函数，其参数为由nav_msgs::Odometry类型构建的指针 pose_msg。

函数的主体包含三个步骤：首先，使用 m_process锁来避免多个线程同时访问共享数据；然后，
将pose_msg指针的x，y，z位置信息分别传递给向量 tic；接着，将pose_msg指针的w、x、y、z方向信息分别传递给四元数 qic，并将其转换为旋转矩阵。

最后，使用 m_process 解锁来释放锁，函数结束。
*/
void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_fusion");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);
    
    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

    if(argc != 2)
    {
        printf("please intput: rosrun loop_fusion loop_fusion_node [config file] \n"
               "for example: rosrun loop_fusion loop_fusion_node "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }
    
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);

    std::string IMAGE_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    std::string pkg_path = ros::package::getPath("loop_fusion");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    printf("cam calib path: %s\n", cam0Path.c_str());
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

    fsSettings["image0_topic"] >> IMAGE_TOPIC;        
    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    fsSettings["save_image"] >> DEBUG_IMAGE;

    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.csv";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    int USE_IMU = fsSettings["imu"];
    posegraph.setIMUFlag(USE_IMU);
    fsSettings.release();

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("no previous pose graph\n");
        load_flag = 1;
    }

    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
    ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    ros::Subscriber sub_margin_point = n.subscribe("/vins_estimator/margin_cloud", 2000, margin_point_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
    pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);

    std::thread measurement_process;
    std::thread keyboard_command_process;

    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);
    
    ros::spin();

    return 0;
}
