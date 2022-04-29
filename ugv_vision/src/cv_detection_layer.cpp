#include <cv_detection_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cv_detection_layer_namespace::CVDetectionLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace cv_detection_layer_namespace
{

CVDetectionLayer::CVDetectionLayer() {}

void CVDetectionLayer::onInitialize()
{
    // initialize node
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;
    this->matchSize();   // ensure that this layer's stored map matches size of overall map

    // configure dynamic_reconfigure to allow us to enable/disable this layerduring runtime
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CVDetectionLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // subscribe to our PolygonStamped message 
    // TODO: for topic in topics_to_subscribe_to (defined by parameter)
    ros::Subscriber polySub = nh.subscribe("/potholes", 1, &CVDetectionLayer::polygonStampedCallback, this);
    // ("/potholes", 1, polygonStampedCallback);
}

void CVDetectionLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

void CVDetectionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
        return;

    // calculate point we wish to modify
    double mark_x_ = robot_x + cos(robot_yaw);
    double mark_y_ = robot_y + sin(robot_yaw);

    // // expand min/max bounds of map to contain this point
    // *min_x = std::min(*min_x, mark_x_);
    // *min_y = std::min(*min_y, mark_y_);
    // *max_x = std::max(*max_x, mark_x_);
    // *max_y = std::max(*max_y, mark_y_);
}

void CVDetectionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;

    unsigned int mx, my;

    // // if to-be-marked point is inside map bounds, mark it as lethal
    // if (master_grid.worldToMap(mark_x_, mark_y_, mx, my))
    // {
    //     master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    // }
}

void CVDetectionLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), 
              master->getResolution(), master->getOriginX(), master->getOriginY());
}


/**
 * @brief callback for PolygonStamped messages;
 *        Polygon is filled, then marked on the map iff it exists within our field of view
 * 
 */
void CVDetectionLayer::polygonStampedCallback(const geometry_msgs::PolygonStamped& msg)
{
    return;
}

}   // namespace cv_detection_layer_namespace