/**
 * @file cv_detection_layer.h
 * @author Wesley Fletcher (wkfletcher@knights.ucf.edu)
 * @brief class definition for the custom costmap_2d "Computer Vision Detection Layer"
 * @version 0.1
 * @date 2022-04-29
 * 
 * @copyright Copyright (c) 2022
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

namespace cv_detection_layer_namespace
{

class CVDetectionLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
    public:
        CVDetectionLayer();
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual void matchSize();
        void polygonStampedCallback(const geometry_msgs::PolygonStamped& msg);
        bool isDiscretized()
        {
            return true;
        }


    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

} // namespace cv_detection_layer_namespace

#endif