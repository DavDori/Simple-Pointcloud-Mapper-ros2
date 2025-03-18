#include <chrono>
#include <functional>
#include <memory>
#include <string>
// Matrix/vector utils
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
// Point cloud utils
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS2 utils
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
// time
#include <chrono>

/*
    * Simple pointcloud mapper node
    * This node subscribes to a pointcloud topic and an odometry topic, and
    * builds a 3D map of the environment.
    * The map is published as a pointcloud.
Assumptions:
    - The pointcloud is registered and in a fixed frame (camera_init)
*/
class MapperNode : public rclcpp::Node 
{
using AffineTransform3f = Eigen::Transform<float, 3, Eigen::Affine>;
public:
    MapperNode() : Node("simple_pointcloud_mapper") 
    {
        declare_parameter("topic.pointcloud.in", "/cloud_registered");
        declare_parameter("topic.pointcloud.out", "/map");
        declare_parameter("topic.odometry.in", "/Odometry");
        declare_parameter("voxel_size_m", 0.1);
        declare_parameter("max_range_m", 5.0);
        declare_parameter("min_range_m", 1.0);
        declare_parameter("filter_step", 50); // Filter every N frames (voxel grid)
        declare_parameter("path", "./");
        declare_parameter("name", "close.pcd");

        std::string topic_cloud_in = get_parameter("topic.pointcloud.in").as_string();
        std::string topic_odom_in = get_parameter("topic.odometry.in").as_string();
        filepath_ = get_parameter("path").as_string();
        filename_ = get_parameter("name").as_string();

        float min_range_m =
            (float)(get_parameter("min_range_m").as_double());
        min_range_m_sq_ = min_range_m * min_range_m;
        float max_range_m =
            (float)(get_parameter("max_range_m").as_double());
        max_range_m_sq_ = max_range_m * max_range_m;
        voxel_size_m_ =
            (float)(get_parameter("voxel_size_m").as_double());

        voxel_filter_step_ = uint(get_parameter("filter_step").as_int()); 
        map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_simple_map",
            std::bind(&MapperNode::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));

        point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_cloud_in, 10,
            std::bind(&MapperNode::pointcloudCallback, this,
            std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            topic_odom_in, 10,
            std::bind(&MapperNode::odomCallback, this,
            std::placeholders::_1));

        std::ostringstream general_params;
        general_params
            << "\n--- Parameters ---\n"
            << "  - Voxel Filter step: " << voxel_filter_step_ << "\n"
            << "  - Pointcloud input topic: " << topic_cloud_in << "\n"
            << "  - Max range squared: " << max_range_m_sq_ << " m^2\n"
            << "  - Min range squared: " << min_range_m_sq_ << " m^2\n"
            << "  - Voxel size: " << voxel_size_m_ << " m\n"
            << "To save the map, call the service /save_simple_map";
        RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "Simple Pointcloud mapper: initialized successfully.");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_;
    Eigen::Vector3f current_position_;
    std::string filepath_, filename_;
    float min_range_m_sq_;
    float max_range_m_sq_;
    float voxel_size_m_;
    uint voxel_filter_step_;
    uint current_frame_ = 0;

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2 &msg) 
    {
        try 
        {
            // Convert to PCL format
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(msg, *cloud);
            // Filter points based on range
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            float c_x = current_position_[0];
            float c_y = current_position_[1];
            float c_z = current_position_[2];
            for (const auto &point : cloud->points)
            {
                float dx = point.x - c_x;
                float dy = point.y - c_y;
                float dz = point.z - c_z;
                float range_sq = dx * dx + dy * dy + dz * dz;
                if (range_sq > min_range_m_sq_ && range_sq < max_range_m_sq_)
                {
                    filtered_cloud->push_back(point);
                }
            }
            // Add point cloud to the map
            updateMap(filtered_cloud);
            if(current_frame_ % voxel_filter_step_ == 0)
            {
                applyVoxelFilter();
            }
            current_frame_++;
        } 
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in pointcloudCallback: %s",
                e.what());
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry &msg) 
    {
        try 
        {
            current_position_ << 
                msg.pose.pose.position.x, 
                msg.pose.pose.position.y,
                msg.pose.pose.position.z;
        }
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error in odomCallback: %s",
                e.what());
        }
    }

    void applyVoxelFilter() 
    {
        // Apply voxel grid filter
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(map_);
        voxel_grid.setLeafSize(voxel_size_m_, voxel_size_m_, voxel_size_m_);
        voxel_grid.filter(*map_);
    }

    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused

        if (map_->empty())
        {
            RCLCPP_ERROR(this->get_logger(), "PointCloud is empty, not saving.");
            response->success = false;
            response->message = "PointCloud is empty.";
            return;
        }

        applyVoxelFilter();

        std::string path = filepath_ + filename_; // Default filename
        int result = pcl::io::savePCDFileBinary(path, *map_);
        
        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved map to %s", path.c_str());
            response->success = true;
            response->message = "Map saved successfully.";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map.");
            response->success = false;
            response->message = "Failed to save map.";
        }
    }

    void updateMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) 
    {
        *map_ += *cloud;
    }

    AffineTransform3f convertToAffine3f(const nav_msgs::msg::Odometry &msg) 
    {
        // Convert to Eigen Affine3f
        Eigen::Quaternionf q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        Eigen::Vector3f t(msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z);
        AffineTransform3f center = AffineTransform3f::Identity();
        center.rotate(q);
        center.translate(t);
        return center;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}
