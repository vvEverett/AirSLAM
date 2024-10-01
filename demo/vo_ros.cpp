#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <thread>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"

MapBuilder* p_map_builder;
int total_frames = 0;
double total_processing_time = 0.0;

void GrabStereo(const sensor_msgs::ImageConstPtr& imgLeft, const sensor_msgs::ImageConstPtr& imgRight){
    // Copy the ros image messages to cvMat
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try{
        cv_ptrLeft = cv_bridge::toCvShare(imgLeft);
        cv_ptrRight = cv_bridge::toCvShare(imgRight);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    static int frame_id = 0;
    auto before_infer = std::chrono::steady_clock::now();

    InputDataPtr input_data = std::shared_ptr<InputData>(new InputData());
    input_data->index = frame_id;
    input_data->image_left = cv_ptrLeft->image.clone();
    input_data->image_right = cv_ptrRight->image.clone();
    input_data->time = imgLeft->header.stamp.toSec();

    if(input_data == nullptr) return;
    p_map_builder->AddInput(input_data);

    auto after_infer = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>
    (after_infer - before_infer).count();
    total_frames++;
    total_processing_time += cost_time / 1000.0;
    std::cout << "i ===== " << frame_id++ << " Processing Time: " << cost_time << " ms." << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "air_slam");

  std::string config_path, model_dir;
  ros::param::get("~config_path", config_path);
  ros::param::get("~model_dir", model_dir);
  VisualOdometryConfigs configs(config_path, model_dir);
  std::cout << "config done" << std::endl;

  ros::param::get("~camera_config_path", configs.camera_config_path);
  ros::param::get("~saving_dir", configs.saving_dir);

  // ROS
  std::string left_topic, right_topic;
  ros::param::get("~left_topic", left_topic);
  ros::param::get("~right_topic", right_topic);
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, left_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, right_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&GrabStereo, _1, _2));
  p_map_builder = new MapBuilder(configs, nh);

  ros::spin();

//  std::cout << "Average FPS = " << image_num / (sum_time / 1000.0) << std::endl;


  std::cout << "Waiting to stop..." << std::endl; 
  p_map_builder->Stop();
  while(!p_map_builder->IsStopped()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Map building has been stopped" << std::endl; 

  // Calculate the FPS
  double average_fps = total_frames / total_processing_time;
  std::cout << "Total frames processed: " << total_frames << std::endl;
  std::cout << "Total processing time: " << total_processing_time << " seconds" << std::endl;
  std::cout << "Average FPS = " << average_fps << std::endl;

  std::string trajectory_path = ConcatenateFolderAndFileName(configs.saving_dir, "trajectory_v0.txt");
  p_map_builder->SaveTrajectory(trajectory_path);
  p_map_builder->SaveMap(configs.saving_dir);
  ros::shutdown();

  return 0;
}
