#include <synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

static Synchronizer::Ptr synchronizer_ptr_;

void pc1Callback(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) {
    SyncSensor::Ptr sensor_ptr(new SyncSensor(SyncSensorType::LIDAR, msg_ptr));
    sensor_ptr->setFrameID("pc1");
    sensor_ptr->setTimestamp(msg_ptr->header.stamp.toSec());
    synchronizer_ptr_->addData(sensor_ptr);
}

void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) {
    SyncSensor::Ptr sensor_ptr(new SyncSensor(SyncSensorType::LIDAR, msg_ptr));
    sensor_ptr->setFrameID("pc2");
    sensor_ptr->setTimestamp(msg_ptr->header.stamp.toSec());
    synchronizer_ptr_->addData(sensor_ptr);
}

void pc3Callback(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) {
    SyncSensor::Ptr sensor_ptr(new SyncSensor(SyncSensorType::LIDAR, msg_ptr));
    sensor_ptr->setFrameID("pc3");
    sensor_ptr->setTimestamp(msg_ptr->header.stamp.toSec());
    synchronizer_ptr_->addData(sensor_ptr);
}

void pc4Callback(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) {
    SyncSensor::Ptr sensor_ptr(new SyncSensor(SyncSensorType::LIDAR, msg_ptr));
    sensor_ptr->setFrameID("pc4");
    sensor_ptr->setTimestamp(msg_ptr->header.stamp.toSec());
    synchronizer_ptr_->addData(sensor_ptr);
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg_ptr) {
    SyncSensor::Ptr sensor_ptr(new SyncSensor(SyncSensorType::POSE, msg_ptr));
    sensor_ptr->setFrameID("pose");
    sensor_ptr->setTimestamp(msg_ptr->header.stamp.toSec());
    synchronizer_ptr_->addData(sensor_ptr);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "time_sync_demo");
    ros::NodeHandlePtr nh_ptr;
    nh_ptr.reset(new ros::NodeHandle);

    SynchronizerOptions options;
    options.frame_id_vec_ = { "pc1", "pc2", "pc3", "pc4", "pose" };
    synchronizer_ptr_.reset(new Synchronizer(options));

    auto func = [&](const std::vector<SyncSensor::Ptr>& msg_ptr_vec) {
        for (size_t i = 0; i < msg_ptr_vec.size(); ++i) {
            std::cout << msg_ptr_vec[i]->getFrameID() << ": " << std::to_string(msg_ptr_vec[i]->getTimestamp()) << std::endl;
        }
        std::cout << "sync succeed! start process..." << std::endl;
    };
    synchronizer_ptr_->regSyncCallback(func);
    synchronizer_ptr_->start();

    ros::Subscriber sub_pc1 = nh_ptr->subscribe("/mems_front/rslidar_points", 10, pc1Callback);
    ros::Subscriber sub_pc2 = nh_ptr->subscribe("/mems_left/rslidar_points", 10, pc2Callback);
    ros::Subscriber sub_pc3 = nh_ptr->subscribe("/mems_back/rslidar_points", 10, pc3Callback);
    ros::Subscriber sub_pc4 = nh_ptr->subscribe("/mems_right/rslidar_points", 10, pc4Callback);
    ros::Subscriber sub_pose = nh_ptr->subscribe("/loc/car_pose_1", 10, poseCallback);

    ros::spin();
}