#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include <numeric>

#include "read_configs.h"
#include "dataset.h"
#include "map_user.h"

MapUser* p_map_user;
std::vector<std::pair<std::string, Eigen::Matrix4d>> trajectory;
std::vector<double> processing_times;
int total_frames = 0;
int success_num = 0;
std::chrono::steady_clock::time_point start_time;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        std::string image_idx = "frame_" + std::to_string(total_frames);

        auto before_infer = std::chrono::high_resolution_clock::now();
        if(p_map_user->Relocalization(cv_ptr->image, pose)){
            image_idx = "success_" + image_idx;
            success_num++;
        } else {
            image_idx = "fail_" + image_idx;
        }
        auto after_infer = std::chrono::high_resolution_clock::now();
        auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
        
        processing_times.push_back(cost_time);
        
        trajectory.emplace_back(std::make_pair(image_idx, pose));
        total_frames++;

        double recall = static_cast<double>(success_num) / total_frames;
        
        std::cout << "Frame " << total_frames << " Processing Time: " << cost_time << " ms." << std::endl;
        std::cout << "Total frames: " << total_frames << ", Success: " << success_num << ", Recall: " << recall << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "air_slam");
    ros::NodeHandle nh;

    std::string config_path, model_dir, map_root, voc_path, traj_path, camera_topic;
    ros::param::get("~config_path", config_path);
    ros::param::get("~model_dir", model_dir);
    ros::param::get("~map_root", map_root);
    ros::param::get("~voc_path", voc_path);
    ros::param::get("~traj_path", traj_path);
    ros::param::get("~camera_topic", camera_topic);

    RelocalizationConfigs configs(config_path, model_dir);
    ros::param::get("~camera_config_path", configs.camera_config_path);

    p_map_user = new MapUser(configs, nh);
    p_map_user->LoadMap(map_root);
    p_map_user->LoadVocabulary(voc_path);

    Eigen::Matrix4d base_frame_pose = p_map_user->GetBaseFramePose();
    double base_frame_time = p_map_user->GetBaseFrameTimestamp();
    trajectory.emplace_back(std::make_pair(("base "+DoubleTimeToString(base_frame_time)), base_frame_pose));

    start_time = std::chrono::steady_clock::now();
    ros::Subscriber sub = nh.subscribe(camera_topic, 1, imageCallback);

    ros::spin();

    auto end_time = std::chrono::steady_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;

    SaveTumTrajectoryToFile(traj_path, trajectory);
    
    double total_processing_time = std::accumulate(processing_times.begin(), processing_times.end(), 0.0) / 1000.0;
    double average_processing_time = 1000 * total_processing_time / total_frames;
    double total_fps = total_frames / total_duration;
    double average_fps = 1000.0 / average_processing_time;

    std::cout << "\nFinal statistics:" << std::endl;
    std::cout << "Total frames: " << total_frames << ", Success: " << success_num << ", Recall: " << static_cast<double>(success_num) / total_frames << std::endl;
    std::cout << "Total runtime: " << total_duration << " seconds" << std::endl;
    std::cout << "Total processing time: " << total_processing_time << " seconds" << std::endl;
    std::cout << "Average processing time per frame: " << average_processing_time << " ms" << std::endl;
    std::cout << "Total FPS (including all overheads): " << total_fps << std::endl;
    std::cout << "Average FPS (based on processing time only): " << average_fps << std::endl;

    p_map_user->StopVisualization();
    delete p_map_user;
    ros::shutdown();

    return 0;
}