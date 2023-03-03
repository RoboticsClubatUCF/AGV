/**
 * @file cv_detection_layer.h
 * @author Wesley Fletcher, Marcus Simmonds (wkfletcher@knights.ucf.edu, mas1660@knights.ucf.edu)
 * @brief class definition for the custom costmap_2d "Computer Vision Detection Layer"
 * @version 0.1
 * @date 2023-02-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CV_DETECTION_LAYER_H_
#define CV_DETECTION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

namespace cv_detection_layer_namespace
{

    struct PointInt {
        int x;
        int y;
    };

class CVDetectionLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
    public:
        using Polygon = std::vector<geometry_msgs::Point32>;
        CVDetectionLayer();
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual void matchSize();
        virtual void setPolyCost(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j, Polygon poly);
        void polygonStampedCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);
        bool isDiscretized()
        {
            return true;
        }

        std::vector<ros::Subscriber> _subs;
    
    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
        std::vector<Polygon> _ground_obstacles;
};

} // namespace cv_detection_layer_namespace

#endif