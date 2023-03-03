#include <cv_detection_layer.h>
#include <pluginlib/class_list_macros.h>

// TODO: Add polygons to vector when subscribed to, then update costs for all polygons in updateCosts()

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

    // bounds for polygon
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

        }
    }

    // Mark polygons

    for (int i = 0; i<_ground_obstacles.size(); i++)
    {
        setPolyCost(master_grid, min_i, min_j, max_i, max_j, _ground_obstacles[i]);
    }

    _ground_obstacles.clear();

}

/**
 * @brief Sets the cost of the surrounding cells in a polygon in the master grid.
 * 
 * @param master_grid The layered_costmap "main" grid to be updated with this layer's info.
 * @param min_i 
 * @param min_j 
 * @param max_i 
 * @param max_j
 * @param poly The actual polygon we want to mark on the grid.
 */
void CVDetectionLayer::setPolyCost(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j, Polygon poly)
{
    // map to world bounds
    std::vector<PointInt> mapped_poly;
    for(int i = 0; i<poly.size(); i++)
    {
        PointInt loc;

        master_grid.worldToMapNoBounds(poly[i].x, poly[i].y, loc.x, loc.y);
        mapped_poly.push_back(loc);
    }

    // actually mark points
    int mx, my;
    for (int i = 0; i<mapped_poly.size(); i++)
    {
        mx = mapped_poly[i].x;
        my = mapped_poly[i].y;
        // check if point is within bounds
        if(mx < min_i || my > max_i )
            continue;

        if(my < min_j || my > max_j)
            continue;
        
        // mark point in master grid 
        master_grid.setCost(mx,my,LETHAL_OBSTACLE);

    }
}

/**
 * @brief PolygonStamped callback; adds polygon points to point vector
 * 
 * @param msg PolygonStamped message containing points
 */
void CVDetectionLayer::polygonStampedCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    
    Polygon pts;
    
    // Add polygons to global polygon structure.
    for (int i = 0; i<msg->polygon.points.size(); i++)
    {
        pts.push_back(msg->polygon.points.at(i));
    }

    _ground_obstacles.push_back(pts);

}

}   // namespace cv_detection_layer_namespace