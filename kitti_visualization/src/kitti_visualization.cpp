#include<pcl/point_cloud.h>
#include<iostream>
#include<vector>
#include <fstream>
#include <pcl/point_types.h>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
using namespace std;
using namespace pcl;
using namespace boost::filesystem;
vector<string> LoadKittiVelodybeFiles(const string lidarFolderPath)
{
    vector<string> lidarFileList;
    directory_iterator end;
    for (directory_iterator dir(lidarFolderPath); dir!=end; dir++)
    {
        if (dir->path().extension() == ".bin")
        {
            lidarFileList.push_back(dir->path().string());
        }
    }
    sort(lidarFileList.begin(), lidarFileList.end());
    return lidarFileList;
}

pcl::PointCloud<pcl::PointXYZI> KittiVelodyneToPointCloud(const std::string lidarDataPath)
{
    std::ifstream lidarDataFile(lidarDataPath, std::ifstream::in | std::ifstream::binary);
    lidarDataFile.seekg(0, ios::end);
    const size_t numElements = lidarDataFile.tellg() / sizeof(float);
    lidarDataFile.seekg(0, ios::beg);
    vector<float> lidarDataBuffer(numElements);
    lidarDataFile.read(reinterpret_cast<char*>(&lidarDataBuffer[0]),numElements*sizeof(float));
    std::cout<<"totally: "<<lidarDataBuffer.size() / 4.0 << "points in this lidar frame \n";
    std::vector<Eigen::Vector3d> lidarPoints;

    pcl::PointCloud<pcl::PointXYZI> laserCloud;
    for (size_t i=0; i < lidarDataBuffer.size(); i+=4)
    {
        lidarPoints.emplace_back(lidarDataBuffer[i], lidarDataBuffer[i+1], lidarDataBuffer[i+2]);
        pcl::PointXYZI point;
        point.x = lidarDataBuffer[i];
        point.y = lidarDataBuffer[i+1];
        point.z = lidarDataBuffer[i+2];
        point.intensity = lidarDataBuffer[i+3];
        laserCloud.push_back(point);
    }
    return laserCloud;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_pointcloud_node");
    ros::NodeHandle kitti_n("~");
    boost::program_options::options_description opts("all options");
    boost::program_options::variables_map vm;
    opts.add_options()
    ("lidar_folder_path", boost::program_options::value<string>(), "the folder path of kitti velodyne");
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    string lidarFolderPath = vm["lidar_folder_path"].as<string>();
    vector<string> lidarFileList = LoadKittiVelodybeFiles(lidarFolderPath);
    float startTimeStamp = 0.0;
    float deltaTimeValue = 0.1; // 100ms
    ros::Rate loop_rate(5);
    ros::Publisher pub_pointcloud=kitti_n.advertise<sensor_msgs::PointCloud2>("pub_kitti_pointcloud", 2);
    int n=0;
    while ( n < lidarFileList.size()       &&ros::ok()        )
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_pc=KittiVelodyneToPointCloud(lidarFileList[n]);
        sensor_msgs::PointCloud2 ros_pc ;
        pcl::toROSMsg(pcl_pc,ros_pc);
        float curTimeStamp = startTimeStamp + n * deltaTimeValue;
        ros_pc.header.stamp = ros::Time().fromSec(curTimeStamp);
        ros_pc.header.frame_id = "/laser";
        pub_pointcloud.publish(ros_pc);
        n++;
        loop_rate.sleep();
    }
    return 0;
}