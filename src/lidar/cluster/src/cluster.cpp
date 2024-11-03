#include "cluster.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <rclcpp/duration.hpp>
#include <pcl/segmentation/extract_clusters.h>
namespace tdt_radar{

    Cluster::Cluster(const rclcpp::NodeOptions& node_options): Node("cluster_node", node_options)
    {
        RCLCPP_INFO(this->get_logger(), "cluster_node start");
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 10, std::bind(&Cluster::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10);
    }

/**
 * @brief 
 *  1订阅原始点云数据
    Cluster 类的构造函数中通过 this->create_subscription 方法订阅了 /livox/lidar_dynamic 主题上的点云数据，消息类型为 sensor_msgs::msg::PointCloud2，并指定了回调函数 callback。当有新的点云数据到达时，callback 函数会自动调用。

    2回调函数 callback 的执行流程
    在 callback 函数中：

    3时间记录
    记录了当前时间 t1，用于后续计算回调的执行时间。

    4点云数据转换
    pcl::fromROSMsg(*msg, *cloud); 将 ROS 的 sensor_msgs::msg::PointCloud2 类型的点云数据转换为 PCL 的 pcl::PointCloud<pcl::PointXYZ> 类型，方便后续处理。如果点云为空，则直接返回，不进行后续处理。

    5构建 KD 树
    使用 pcl::search::KdTree 构建了一个 KD 树加速结构，并将点云数据 cloud 作为输入。KD 树是高效的邻域搜索结构，在聚类算法中用于加速空间查询。

    6设置并执行欧几里得聚类
    使用 pcl::EuclideanClusterExtraction 进行欧几里得聚类，设置了聚类的参数：

    setClusterTolerance(0.25)：设置点之间的最大距离，以判定是否属于同一簇。
    setMinClusterSize(5)：最小簇大小为 5。
    setMaxClusterSize(1000)：最大簇大小为 1000。
    setSearchMethod(tree) 和 setInputCloud(cloud)：指定了搜索方法和输入点云。
    7调用 ec.extract(cluster_indices); 提取聚类结果，并将每个簇的点索引保存在 cluster_indices 中。

    8计算质心并存储
    对每个簇 it，从 cluster_indices 中提取每个点索引，将这些点复制到 cloud_cluster。然后计算 cloud_cluster 中点的质心，将其存储到 move_point，并将 move_point 添加到 out_cloud。

    9发布聚类结果
    将计算出的质心点云 out_cloud 转换为 ROS 消息类型 sensor_msgs::msg::PointCloud2，并发布到主题 /livox/lidar_cluster。这样，所有订阅了 /livox/lidar_cluster 的节点都可以接收到聚类后的质心点云数据。

    10计算并记录回调执行时间
    计算从 t1 到当前时间 t2 的回调函数执行时间，以微秒为单位输出到日志中。
 * @param msg 
 */
void Cluster::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //使用 std::chrono 库记录当前时间，便于后续计算函数的执行时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {return;}//如果点云为空，直接返回
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //创建一个KdTree对象，用于搜索点云中的点
    tree->setInputCloud(cloud);//将点云传入KdTree对象
    auto time = std::chrono::system_clock::now();//记录当前时间

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//点云数据进行聚类。它使用欧几里得距离来确定点之间的相似性，适用于处理空间中的点云数据
    ec.setClusterTolerance (0.25);//设置聚类的容差（距离阈值），单位为米
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (1000);
    //聚类的最小和最大点数，如果聚类的点数小于最小点数，则不会被认为是一个聚类。
    ec.setSearchMethod (tree);
    //设置搜索方法，这里使用的是KdTree
    ec.setInputCloud (cloud);//设置输入点云,即需要聚类的点云
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);//执行聚类提取操作，将聚类结果存储在cluster_indices中
    // std::cout<<(std::chrono::system_clock::now()-time).count()<<"ms"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> *out_cloud(new pcl::PointCloud<pcl::PointXYZ>); 


        
    //begin()返回指向容器中第一个元素的迭代器
    /*end()返回指向容器最后一个元素的迭代器
        可以使用下标操作符 [] 或 at() 方法访问 vector 中的元素：
        int x = myVector[0]; // 获取第一个元素
        int y = myVector.at(1); // 获取第二个元素
    */
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }        
        cloud_cluster->width = cloud_cluster->points.size();
        """
        将 cloud_cluster 的宽度设置为点的数量（cloud_cluster->points.size()）。
        在点云数据中，width 表示点云的宽度，如果点云是以单行存储的，则宽度就是点的总数量
        """
        cloud_cluster->height = 1;//无序点云数据,高度为1
        cloud_cluster->is_dense = true;//点云数据密集，没有nan点

        pcl::PointXYZ move_point;
        """auto point : cloud_cluster->points 能迭代是因为 cloud_cluster->points 是一个点的集合
        通常是一个 std::vector 类型），
        而标/（准库的容器（如 std::vector）支持范围基 for 循环迭代
        """
        for(auto point:cloud_cluster->points)
        {
            move_point.x += point.x;
            move_point.y += point.y;
            move_point.z += point.z;
        }
        move_point.x /= cloud_cluster->points.size();
        move_point.y /= cloud_cluster->points.size();
        move_point.z /= cloud_cluster->points.size();
        out_cloud->points.push_back(move_point);//将质心点加入到out_cloud中       
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*out_cloud, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);
    //还会有别的发布者（雷达数据）在这个主题上发布消息，使得回调触发！！！！！！
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Cluster callback time: %f", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
}
}//namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Cluster)
//将 Cluster 类注册为一个可以在 ROS 2 系统中动态使用的组件，这样可以通过 ros2 run 命仍然启动 Cluster 节点。