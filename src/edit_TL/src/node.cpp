
#include <boost/filesystem.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>

#include <edit_TL/node.hpp>

namespace traffic_light
{
    MapBasedEditor::MapBasedEditor() : nh_("~"), tf_listener_(tf_buffer_)
    {
        img_sub_ = nh_.subscribe("input/image_raw", 1, &MapBasedEditor::imgCallback, this);
        camera_info_sub_ = nh_.subscribe("input/camera_info", 1, &MapBasedEditor::cameraInfoCallback, this);
        roi_sub_ = nh_.subscribe("/TL/rois", 1, &MapBasedEditor::roiCallback, this);
        map_sub_ = nh_.subscribe("input/vector_map", 1, &MapBasedEditor::mapCallback, this);

        cv::namedWindow("image", CV_WINDOW_NORMAL);
        cv::resizeWindow("image", 640, 480);
        cv::Mat roi_img;
        tf::TransformListener listener;

        nh_.getParam("tunning_scale", config_.tunning_scale);
        nh_.getParam("dir_path", config_.dir_path);
        nh_.getParam("lat", config_.lat);
        nh_.getParam("lon", config_.lon);
        nh_.getParam("alt", config_.alt);
        nh_.getParam("x", config_.x);
        nh_.getParam("y", config_.y);
        nh_.getParam("z", config_.z);

        while (ros::ok())
        {
            tf::StampedTransform transform;

            try 
            {
                listener.lookupTransform("map", "camera_TL_optical_link", ros::Time(0), transform);
            }
            catch (tf2::TransformException& e)
            {
                ROS_WARN_THROTTLE(5, "cannot get transform from map frame to camera frame");
            }

            int x, y, s_x, s_y;
            int width, height, s_width, s_height;
            int tl_id = 0;
            float dist_cam_z;

            ros::spinOnce();
            origin_img.copyTo(roi_img);

            if (select_roi > tl_rois.size())
            {
                select_roi = 0;
            }
                    
            for(int i = 0; i < tl_rois.size(); i++)
            {
                x = tl_rois[i].roi.x_offset;
                y = tl_rois[i].roi.y_offset;
                width = tl_rois[i].roi.width;
                height = tl_rois[i].roi.height;
                s_x = tl_rois[select_roi].roi.x_offset + move_x;
                s_y = tl_rois[select_roi].roi.y_offset + move_y;
                s_width = tl_rois[select_roi].roi.width;
                s_height = tl_rois[select_roi].roi.height;
                if (select_roi != i)
                {
                    cv::rectangle(roi_img, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 3);
                    cv::putText(roi_img, std::to_string(tl_rois[i].id), cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 255, 0), 1, CV_AA);
                }
                else
                {
                    cv::rectangle(
                        roi_img, 
                        cv::Rect(s_x, s_y, s_width, s_height), 
                        cv::Scalar(0, 0, 255), 
                        3);
                    cv::putText(
                        roi_img, 
                        std::to_string(tl_rois[select_roi].id), 
                        cv::Point(s_x, s_y), 
                        cv::FONT_HERSHEY_COMPLEX, 
                        1.0, 
                        cv::Scalar(0, 0, 255), 
                        1, 
                        CV_AA);
                    dist_cam_z = tl_rois[select_roi].dist_cam_z;
                    tl_id = tl_rois[select_roi].id;
                }
            }

            cv::imshow("image", roi_img);
            //img2map
            //cv::Mat s_cam3d, cam3d;
            std::vector<cv::Point2f> s_roi_point, roi_point;
            s_roi_point.push_back(cv::Point2f(s_x, s_y));
            roi_point.push_back(cv::Point2f(s_x-move_x, s_y-move_y));
            std::vector<cv::Point2f> s_points_undistorted, points_undistorted;
            cv::undistortPoints(s_roi_point, s_points_undistorted, cameraMat, distCoeff, cv::noArray(), cv::noArray());
            cv::undistortPoints(roi_point, points_undistorted, cameraMat, distCoeff, cv::noArray(), cv::noArray());

            if (isnan(s_points_undistorted[0].x) || isnan(points_undistorted[0].x))
            {
                continue;
            }

            tf::Vector3 s_cam3d(s_points_undistorted[0].x * dist_cam_z, s_points_undistorted[0].y * dist_cam_z, dist_cam_z);
            tf::Vector3 cam3d(points_undistorted[0].x * dist_cam_z, points_undistorted[0].y * dist_cam_z, dist_cam_z);

            tf::Vector3 s_map3d = transform * s_cam3d;     
            tf::Vector3 map3d = transform * cam3d;
            tf::Vector3 diff_map3d = s_map3d - map3d;                                             

            int chkey = cv::waitKey(1);

            switch (chkey)
            {
                case 'a' : move_x -= config_.tunning_scale; break;
                case 's' : move_y += config_.tunning_scale; break;
                case 'd' : move_x += config_.tunning_scale; break;
                case 'w' : move_y -= config_.tunning_scale; break;
                case 9   : select_roi++; move_x = 0; move_y = 0; break; // tab
                case 32  : // space
                    ROS_INFO("Target = [ %d ]", tl_id);
                    ROS_INFO("Vector = [ %lf, %lf, %lf ]", 
                        diff_map3d.getX(),
                        diff_map3d.getY(),
                        diff_map3d.getZ()
                    );
                    reviseMap(diff_map3d, tl_id);
                    break;
                case 27  : ros::shutdown();  // esc
            }
        }
    }

    void MapBasedEditor::imgCallback(const sensor_msgs::ImageConstPtr &input_msg)
    {
        origin_img = cv_bridge::toCvCopy(input_msg, sensor_msgs::image_encodings::BGR8)->image;
    }

    void MapBasedEditor::roiCallback(const autoware_perception_msgs::TrafficLightRoiArrayConstPtr &input_msg)
    {
        std::vector<autoware_perception_msgs::TrafficLightRoi> tl_rois_tmp;
        for (auto tl_roi : input_msg->rois)
        {
            tl_rois_tmp.push_back(tl_roi);
        }
        tl_rois = tl_rois_tmp;
    }

    void MapBasedEditor::cameraInfoCallback(const sensor_msgs::CameraInfo & input_msg)
    {
        cameraMat = (
            cv::Mat_<float>(3, 3) << input_msg.K[0], 
            input_msg.K[1], 
            input_msg.K[2],
            input_msg.K[3], 
            input_msg.K[4], 
            input_msg.K[5],
            input_msg.K[6], 
            input_msg.K[7], 
            input_msg.K[8]);
        distCoeff = (
            cv::Mat_<float>(5, 1) << input_msg.D[0], 
            input_msg.D[1], 
            input_msg.D[2], 
            input_msg.D[3], 
            input_msg.D[4]);
    }

    void MapBasedEditor::mapCallback(const autoware_lanelet2_msgs::MapBin & input_msg)
    {
        lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
        lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
        lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

        lanelet::utils::conversion::fromBinMsg(
            input_msg, 
            lanelet_map_ptr_, 
            &traffic_rules_ptr_, 
            &routing_graph_ptr_
            );

        for (auto iter = lanelet_map_ptr_->pointLayer.begin(); iter != lanelet_map_ptr_->pointLayer.end(); iter++)
        {
            lanelet::Point3d p = *iter;
            p.basicPoint().x() += config_.x;
            p.basicPoint().y() += config_.y;
            p.basicPoint().z() += config_.z;
        }

        is_map = true;

        ROS_INFO("Intialized map data");
    }

    void MapBasedEditor::reviseMap(const tf::Vector3 diff_map3d, const int tl_id)
    {
        if (!is_map)
        {
            ROS_ERROR("Map data is not initialized!");
            return;
        }

        lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
        std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems = lanelet::utils::query::autowareTrafficLights(all_lanelets);

        lanelet::ConstLineString3d bl;
        bool bulb_flag = false;

        for (auto tli = aw_tl_reg_elems.begin(); tli != aw_tl_reg_elems.end(); tli++) 
        {
            lanelet::AutowareTrafficLightConstPtr tl = *tli;

            const auto light_bulbs = tl->lightBulbs();
            for (auto ls : light_bulbs) 
            {
                for (auto attr : ls.attributes())
                {
                    if (attr.first == "traffic_light_id" && attr.second == tl_id)
                    {
                        bl = static_cast<lanelet::ConstLineString3d>(ls);
                        bulb_flag = true;
                        break;
                    }
                }
            }

            if (bulb_flag)
            {
                break;
            }
        }

        if (!bulb_flag)
        {
            ROS_WARN("Cannot find matching bulb!");
            return;
        }

        auto light = lanelet_map_ptr_->lineStringLayer.get(tl_id);
        auto bulb = lanelet_map_ptr_->lineStringLayer.get(bl.id());

        for (auto ls : { light, bulb })
        {
            for (size_t i = 0; i < ls.size(); i++)
            {
                ls[i].basicPoint().x() += diff_map3d.getX();
                ls[i].basicPoint().y() += diff_map3d.getY();
                ls[i].basicPoint().z() += diff_map3d.getZ();
            }
        }

        // 3. save
        if (!boost::filesystem::is_directory(config_.dir_path))
        {
            boost::filesystem::create_directories(config_.dir_path);
        }

        lanelet::GPSPoint gps_point({config_.lat, config_.lon, config_.alt});  
        lanelet::Origin origin(gps_point);
        try
        {
            lanelet::write(config_.dir_path + "/lanelet2_map.osm", *lanelet_map_ptr_, lanelet::projection::UtmProjector(origin, false ,false)); 
        }
        catch (const lanelet::WriteError& e)
        {

        }

        ROS_INFO("Saved map data : %s/lanelet2_map.osm", config_.dir_path.c_str());
    }
}  
