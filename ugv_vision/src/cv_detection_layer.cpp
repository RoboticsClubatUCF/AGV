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
    _subs.push_back(nh.subscribe("/ground_marks", 10, &CVDetectionLayer::polygonStampedCallback, this));
    // ("/potholes", 1, polygonStampedCallback);
}

/**
 * @brief The dynamic_reconfigure callback.
 * 
 * @param config 
 * @param level 
 */
void CVDetectionLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

/**
 * @brief Define area of costmap that will be updated.
 * 
 * @param robot_x 
 * @param robot_y 
 * @param robot_yaw 
 * @param min_x 
 * @param min_y 
 * @param max_x 
 * @param max_y 
 */
void CVDetectionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    // ROS_INFO("updateBounds()");

    if (!enabled_)
        return;

    // calculate point we wish to modify
    double mark_x_ = robot_x + cos(robot_yaw);
    double mark_y_ = robot_y + sin(robot_yaw);

    // expand min/max bounds of map to contain this point
    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
}

void CVDetectionLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), 
              master->getResolution(), master->getOriginX(), master->getOriginY());
}

/**
 * @brief Set cost of cells within the area calculated by updateBounds(). Called by main layered_costmap loop.
 * 
 * @param master_grid The layered_costmap "main" grid to be updated with this layer's info.
 * @param min_i 
 * @param min_j 
 * @param max_i 
 * @param max_j 
 */
void CVDetectionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    ROS_INFO("updateCosts()");

    if (!enabled_)
        return;

    for (int j = min_j; j < max_j; j++)
    {
        for (int i = min_i; i < max_i; i++)
        {
            int index = getIndex(i, j);
            if (costmap_[index] == NO_INFORMATION)
                continue;
            master_grid.setCost(i, j, costmap_[index]); 
            ROS_INFO("index value: %d", costmap_[index]);
            ROS_INFO("master grid value: %d", master_grid.getCost(i, j));
        }
    }
}

/**
 * @brief PolygonStamped callback; marks polygon on internal costmap
 * 
 * @param msg PolygonStamped message containing points
 */
void CVDetectionLayer::polygonStampedCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    ROS_INFO("polygonStampedCallback()");

    if (!enabled_)
        return;

    double world_x, world_y;
    unsigned int mx, my;

    // loop over all points in polygon to mark map
    for (int idx = 0; idx < msg->polygon.points.size(); idx++)
    {
        world_x = msg->polygon.points.at(idx).x;
        world_y = msg->polygon.points.at(idx).y;

        // convert world points to map points
        worldToMap(world_x, world_y, mx, my);

        // costmaps are stored as 1D, so we need to get the index
        int index = getIndex(mx, my);

        // mark every point in the polygon as lethal
        costmap_[index] = LETHAL_OBSTACLE;
    }

    return;
}

}   // namespace cv_detection_layer_namespace