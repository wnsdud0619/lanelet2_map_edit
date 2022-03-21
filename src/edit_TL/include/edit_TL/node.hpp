#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>

#include "autoware_perception_msgs/TrafficLightRoiArray.h"

namespace traffic_light
{
    class MapBasedEditor
    {
        public:
            MapBasedEditor();
            ~MapBasedEditor() {}

        private:
            struct Config
            {
                int tunning_scale;
                std::string dir_path;
                float lat;
                float lon;
                float alt;
                float x;
                float y;
                float z;
            };

        private:
            ros::NodeHandle nh_;
            ros::Subscriber map_sub_;
            ros::Subscriber camera_info_sub_;
            ros::Subscriber roi_sub_;
            ros::Subscriber img_sub_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;

            cv::Mat origin_img = cv::Mat::zeros(2048, 2448, CV_8UC3);
            std::vector<autoware_perception_msgs::TrafficLightRoi> tl_rois;
            cv::Mat cameraMat = cv::Mat::zeros(3, 3, CV_32FC1);
            cv::Mat distCoeff = cv::Mat::zeros(5, 1, CV_32FC1);
            int move_x, move_y, select_roi;
            lanelet::LaneletMapPtr lanelet_map_ptr_;
            bool is_map = false;
            
            Config config_;

            void imgCallback(const sensor_msgs::ImageConstPtr &input_msg);
            void roiCallback(const autoware_perception_msgs::TrafficLightRoiArrayConstPtr &input_msg);
            void cameraInfoCallback(const sensor_msgs::CameraInfo & input_msg);
            void mapCallback(const autoware_lanelet2_msgs::MapBin & input_msg);
            void reviseMap(const tf::Vector3 diff_map3d, const int tl_id);
    };
}  // namespace traffic_light
