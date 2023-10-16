#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>

class ObstacleMapUpdater
{
public:
    ObstacleMapUpdater()
    {
        // Subscribe to the "/static_obstacles" topic
        obstacle_subscriber = nh.subscribe("/static_obstacles", 10, &ObstacleMapUpdater::obstacleCallback, this);

        // Subscribe to the "/map" topic
        map_subscriber = nh.subscribe("/map", 1, &ObstacleMapUpdater::mapCallback, this);

        // Advertise the updated map
        updated_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/updated_map", 1);
    }

    void obstacleCallback(const std_msgs::Int32MultiArray::ConstPtr& obstacle_data)
    {
        // Process the obstacle data and update the map
        updateMapWithObstacles(obstacle_data);

        // Publish the updated map
        updated_map_publisher.publish(current_map);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
    {
        // Store the map received from the "/map" topic
        original_map = *map;
        current_map = *map;
    }

    void updateMapWithObstacles(const std_msgs::Int32MultiArray::ConstPtr& obstacle_data)
    {
        // Ensure that the map size is valid
        current_map = original_map;
        int map_width = static_cast<int>(original_map.info.width);
        int map_height = static_cast<int>(original_map.info.height);
        unsigned int map_size = original_map.data.size();

        if (map_size != static_cast<unsigned int>(map_width) * static_cast<unsigned int>(map_height)) {
            ROS_ERROR("Map size is not valid.");
            return;
        }

        // Process the obstacle data and update the occupancy grid map.
        for (std::vector<int>::size_type i = 0; i < obstacle_data->data.size(); i += 5)
        {
            int x = obstacle_data->data[i + 1];
            int y = obstacle_data->data[i + 2];
            int vel = obstacle_data->data[i + 4];

            // Check if vel is 0, and the coordinates are within the map boundaries
            if (vel == 0 && x >= 0 && x < map_width && y >= 0 && y < map_height)
            {
                // Calculate the map index for the given x and y
                int map_index = x + y * map_width;
                int map_index_1 = x-1 + y * map_width;
                int map_index_2 = x+1 + y * map_width;
                int map_index_3 = x + (y+1) * map_width;
                int map_index_4 = x-1 + (y+1) * map_width;
                int map_index_5 = x+1 + (y+1) * map_width;
                int map_index_6 = x + (y-1) * map_width;
                int map_index_7 = x-1 + (y-1) * map_width;
                int map_index_8 = x+1 + (y-1) * map_width;

                // Set the map cell at the calculated index to 100 (occupied)
                if (static_cast<unsigned int>(map_index) < map_size) {
                    current_map.data[map_index] = 100;
                    current_map.data[map_index_1] = 100;
                    current_map.data[map_index_2] = 100;
                    current_map.data[map_index_3] = 100;
                    current_map.data[map_index_4] = 100;
                    current_map.data[map_index_5] = 100;
                    current_map.data[map_index_6] = 100;
                    current_map.data[map_index_7] = 100;
                    current_map.data[map_index_8] = 100;
                }
            }
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber obstacle_subscriber;
    ros::Subscriber map_subscriber;
    ros::Publisher updated_map_publisher;
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid current_map;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_obstacle_updater");
    ObstacleMapUpdater updater;
    ros::spin();
    return 0;
}
