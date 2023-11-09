// // #include <ros/ros.h>
// // #include <std_msgs/Int32MultiArray.h>
// // #include <nav_msgs/OccupancyGrid.h>

// // class ObstacleMapUpdater
// // {
// // public:
// //     ObstacleMapUpdater()
// //     {
// //         // Subscribe to the "/static_obstacles" topic
// //         // obstacle_subscriber = nh.subscribe("/static_obstacles", 10, &ObstacleMapUpdater::obstacleCallback, this);
// //         obstacle_subscriber = nh.subscribe("/rccar_pose", 10, &ObstacleMapUpdater::obstacleCallback, this);

// //         // Subscribe to the "/map" topic
// //         map_subscriber = nh.subscribe("/map", 1, &ObstacleMapUpdater::mapCallback, this);

// //         // Advertise the updated map
// //         updated_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/updated_map", 1);
// //     }

// //     void obstacleCallback(const std_msgs::Int32MultiArray::ConstPtr& obstacle_data)
// //     {
// //         // Process the obstacle data and update the map
// //         updateMapWithObstacles(obstacle_data);

// //         // Publish the updated map
// //         updated_map_publisher.publish(current_map);
// //     }

// //     void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
// //     {
// //         // Store the map received from the "/map" topic
// //         original_map = *map;
// //         current_map = *map;
// //     }

// //     void updateMapWithObstacles(const std_msgs::Int32MultiArray::ConstPtr& obstacle_data)
// //     {
// //         // Ensure that the map size is valid
// //         current_map = original_map;
// //         int map_width = static_cast<int>(original_map.info.width);
// //         int map_height = static_cast<int>(original_map.info.height);
// //         unsigned int map_size = original_map.data.size();

// //         if (map_size != static_cast<unsigned int>(map_width) * static_cast<unsigned int>(map_height)) {
// //             ROS_ERROR("Map size is not valid.");
// //             return;
// //         }

// //         // Process the obstacle data and update the occupancy grid map.
// //         for (std::vector<int>::size_type i = 0; i < obstacle_data->data.size(); i += 5)
// //         {
// //             int x = obstacle_data->data[i + 1];
// //             int y = obstacle_data->data[i + 2];
// //             int vel = obstacle_data->data[i + 4];

// //             // Check if vel is 0, and the coordinates are within the map boundaries
// //             if (vel == 0 && x >= 0 && x < map_width && y >= 0 && y < map_height)
// //             {
// //                 // Calculate the map index for the given x and y
// //                 int map_index = x + y * map_width;
                
// //                 if (static_cast<unsigned int>(map_index) < map_size) {
// //                     current_map.data[map_index] = 100; 
// //                 }
// //                 for (int pix =1; pix<12; pix++){
                    
// //                     int map_index_1 = x-pix + y * map_width;
// //                     int map_index_2 = x+pix + y * map_width;
// //                     int map_index_3 = x + (y+pix) * map_width;
// //                     int map_index_4 = x-pix + (y+pix) * map_width;
// //                     int map_index_5 = x+pix + (y+pix) * map_width;
// //                     int map_index_6 = x + (y-pix) * map_width;
// //                     int map_index_7 = x-pix + (y-pix) * map_width;
// //                     int map_index_8 = x+pix + (y-pix) * map_width;
// //                     if (static_cast<unsigned int>(map_index_1) < map_size) {
// //                         current_map.data[map_index_1] = 100;
// //                     }
// //                     if (static_cast<unsigned int>(map_index_2) < map_size) {
// //                         current_map.data[map_index_2] = 100;
// //                     }
// //                     if (static_cast<unsigned int>(map_index_3) < map_size) {
// //                         current_map.data[map_index_3] = 100;
// //                     }
// //                     if (static_cast<unsigned int>(map_index_4) < map_size) {
// //                         current_map.data[map_index_4] = 100;
// //                     }
// //                     if (static_cast<unsigned int>(map_index_5) < map_size) {
// //                         current_map.data[map_index_5] = 100;
// //                     }
// //                     if (static_cast<unsigned int>(map_index_6) < map_size) {
// //                         current_map.data[map_index_6] = 100;
// //                     }
// //                     if (static_cast<unsigned int>(map_index_7) < map_size) {
// //                         current_map.data[map_index_7] = 100;
// //                     }
// //                     if (static_cast<unsigned int>(map_index_8) < map_size) {
// //                         current_map.data[map_index_8] = 100;
// //                     }
// //                 }
// //                 // int map_index = x + y * map_width;
// //                 // current_map.data[map_index] = 100;
// //                 // int map_index_1 = x-1 + y * map_width;
// //                 // int map_index_2 = x+1 + y * map_width;
// //                 // int map_index_3 = x + (y+1) * map_width;
// //                 // int map_index_4 = x-1 + (y+1) * map_width;
// //                 // int map_index_5 = x+1 + (y+1) * map_width;
// //                 // int map_index_6 = x + (y-1) * map_width;
// //                 // int map_index_7 = x-1 + (y-1) * map_width;
// //                 // int map_index_8 = x+1 + (y-1) * map_width;

// //                 // Set the map cell at the calculated index to 100 (occupied)
// //                 // if (static_cast<unsigned int>(map_index) < map_size) {
// //                 //     current_map.data[map_index] = 100;
// //                 //     current_map.data[map_index_1] = 100;
// //                 //     current_map.data[map_index_2] = 100;
// //                 //     current_map.data[map_index_3] = 100;
// //                 //     current_map.data[map_index_4] = 100;
// //                 //     current_map.data[map_index_5] = 100;
// //                 //     current_map.data[map_index_6] = 100;
// //                 //     current_map.data[map_index_7] = 100;
// //                 //     current_map.data[map_index_8] = 100;
// //                 // }
// //             }
// //         }
// //     }

// // private:
// //     ros::NodeHandle nh;
// //     ros::Subscriber obstacle_subscriber;
// //     ros::Subscriber map_subscriber;
// //     ros::Publisher updated_map_publisher;
// //     nav_msgs::OccupancyGrid original_map;
// //     nav_msgs::OccupancyGrid current_map;
// // };

// // int main(int argc, char** argv)
// // {
// //     ros::init(argc, argv, "map_obstacle_updater");
// //     ObstacleMapUpdater updater;
// //     ros::spin();
// //     return 0;
// // }
// #include <ros/ros.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <nav_msgs/OccupancyGrid.h>

// class ObstacleMapUpdater
// {
// public:
// bool scale_100_3 = false;
//     ObstacleMapUpdater()
//     {
//         obstacle_subscriber = nh.subscribe("/rccar_pose", 10, &ObstacleMapUpdater::obstacleCallback, this);
//         map_subscriber = nh.subscribe("/map", 1, &ObstacleMapUpdater::mapCallback, this);
//         updated_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/updated_map", 1);
//     }

//     void obstacleCallback(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data)
//     {
//         updateMapWithObstacles(obstacle_data);
//         updated_map_publisher.publish(current_map);
//     }

//     void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
//     {
//         original_map = *map;
//         current_map = *map;
//     }

//     void updateMapWithObstacles(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data)
//     {
//         current_map = original_map;
//         int map_width = static_cast<int>(original_map.info.width);
//         int map_height = static_cast<int>(original_map.info.height);
//         unsigned int map_size = original_map.data.size();

//         if (map_size != static_cast<unsigned int>(map_width) * static_cast<unsigned int>(map_height)) {
//             ROS_ERROR("Map size is not valid.");
//             return;
//         }
//         int scale_factor = 1;
//         int translate_x = 0;
//         int translate_y = 0;
//         int extend = 0;
//         if (scale_100_3 ==true){
//             scale_factor = 100;
//             translate_x = -30;
//             translate_y = 30;
//             extend = 13;
//         }
//         else{
//             scale_factor = 10;
//             translate_x = -3;
//             translate_y = 3;
//             extend = 3;
//         }
        
        

//         for (std::vector<float>::size_type i = 0; i < obstacle_data->data.size(); i += 5)
//         {
//             // std::cout<<" The original values are: "<< obstacle_data->data[i + 1]<< " "<< obstacle_data->data[i + 2]<< " "<< std::endl;
//             // double x_og = obstacle_data->data[i + 1];
//             // double y_og = obstacle_data->data[i + 2];
//             int id = (obstacle_data->data[i]);
//             int x = static_cast<int>(obstacle_data->data[i + 1]*scale_factor)+translate_x;
//             int y = static_cast<int>(obstacle_data->data[i + 2]*scale_factor)+translate_y;
//             double vel = std::abs(obstacle_data->data[i + 3]);
//             // std::cout<< "velocity is                             "<<vel << "                           "<< i << std::endl;
//             // std::cout<<" The Scaled values are: "<< x<< " "<< y<< " "<< std::endl;
//             if (id < 1000 && vel <= 0.05 && x >= 0 && x < map_width && y >= 0 && y < map_height)
//             {
//                 // std::cout<< "I have an obstical so something has 0 vel" << std::endl;
//                 int map_index = x + y * map_width;
                
//                 if (static_cast<unsigned int>(map_index) < map_size) {
//                     current_map.data[map_index] = 100; 
//                 }
                
//                 for (int pix = 1; pix < extend; pix++){
//                     // Update surrounding cells
//                     for (int dx = -pix; dx <= pix; dx++) {
//                         for (int dy = -pix; dy <= pix; dy++) {
//                             int new_x = x + dx;
//                             int new_y = y + dy;
//                             if (new_x >= 0 && new_x < map_width && new_y >= 0 && new_y < map_height) {
//                                 int index = new_x + new_y * map_width;
//                                 if (static_cast<unsigned int>(index) < map_size) {
//                                     current_map.data[index] = 100;
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//             // double backx = static_cast<double>(x+30)/100;
//             // double backy = static_cast<double>(y-48)/100;
//             // std::cout<<" The Scaled back to original values are: "<< backx<< " "<< backy<< " "<< std::endl;
//             // std::cout<<"##########################ERROR######################"<< x_og-backx << " "<< y_og-backy<<std::endl;
            
//         }
//     }

// private:
//     ros::NodeHandle nh;
//     ros::Subscriber obstacle_subscriber;
//     ros::Subscriber map_subscriber;
//     ros::Publisher updated_map_publisher;
//     nav_msgs::OccupancyGrid original_map;
//     nav_msgs::OccupancyGrid current_map;
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "map_obstacle_updater");
//     ObstacleMapUpdater updater;
//     ros::spin();
//     return 0;
// }





#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>

class ObstacleMapUpdater
{
public:
bool scale_100_3 = false;
    ObstacleMapUpdater()
    {
        obstacle_subscriber = nh.subscribe("/rccar_pose", 10, &ObstacleMapUpdater::obstacleCallback, this);
        map_subscriber = nh.subscribe("/map", 1, &ObstacleMapUpdater::mapCallback, this);
        updated_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/updated_map", 1);
    }

    void obstacleCallback(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data)
    {
        updateMapWithObstacles(obstacle_data);
        updated_map_publisher.publish(current_map);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
    {
        original_map = *map;
        current_map = *map;
    }

    void updateMapWithObstacles(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data)
    {
        current_map = original_map;
        int map_width = static_cast<int>(original_map.info.width);
        int map_height = static_cast<int>(original_map.info.height);
        unsigned int map_size = original_map.data.size();

        if (map_size != static_cast<unsigned int>(map_width) * static_cast<unsigned int>(map_height)) {
            ROS_ERROR("Map size is not valid.");
            return;
        }
        int scale_factor = 1;
        int translate_x = 0;
        int translate_y = 0;
        double extend = 0.0;
        if (scale_100_3 ==true){
            scale_factor = 100;
            translate_x = -30;
            translate_y = 30;
            extend = 13;
        }
        else{
            scale_factor = 10;
            translate_x = -3;
            translate_y = 3;
            extend = 2.5;
        }
        
        

        for (std::vector<float>::size_type i = 0; i < obstacle_data->data.size(); i += 5)
        {
            // std::cout<<" The original values are: "<< obstacle_data->data[i + 1]<< " "<< obstacle_data->data[i + 2]<< " "<< std::endl;
            // double x_og = obstacle_data->data[i + 1];
            // double y_og = obstacle_data->data[i + 2];
            int id = (obstacle_data->data[i]);
            int x = static_cast<int>(obstacle_data->data[i + 1]*scale_factor)+translate_x;
            int y = static_cast<int>(obstacle_data->data[i + 2]*scale_factor)+translate_y;
            double vel = std::abs(obstacle_data->data[i + 3]);
            // std::cout<< "velocity is                             "<<vel << "                           "<< i << std::endl;
            // std::cout<<" The Scaled values are: "<< x<< " "<< y<< " "<< std::endl;
            if (id < 1000 && vel <= 0.05 && x >= 0 && x < map_width && y >= 0 && y < map_height)
            {
                // std::cout<< "I have an obstical so something has 0 vel" << std::endl;
                int map_index = x + y * map_width;
                
                if (static_cast<unsigned int>(map_index) < map_size) {
                    current_map.data[map_index] = 100; 
                }
                for (double pix = 0.5; pix < extend; pix += 1.0) { // Use double type for pix
                // Update surrounding cells
                    for (int dx = -static_cast<int>(pix); dx <= static_cast<int>(pix); dx++) {
                        for (int dy = -static_cast<int>(pix); dy <= static_cast<int>(pix); dy++) {
                            double new_x = x + dx;
                            double new_y = y + dy;
                            if (new_x >= 0 && new_x < map_width && new_y >= 0 && new_y < map_height) {
                                int index = static_cast<int>(new_x) + static_cast<int>(new_y) * map_width;
                                if (static_cast<unsigned int>(index) < map_size) {
                                    current_map.data[index] = 100;
                                }
                            }
                        }
                    }
                }

















                // for (int pix = 1; pix < extend; pix++){
                //     // Update surrounding cells
                //     for (int dx = -pix; dx <= pix; dx++) {
                //         for (int dy = -pix; dy <= pix; dy++) {
                //             int new_x = x + dx;
                //             int new_y = y + dy;
                //             if (new_x >= 0 && new_x < map_width && new_y >= 0 && new_y < map_height) {
                //                 int index = new_x + new_y * map_width;
                //                 if (static_cast<unsigned int>(index) < map_size) {
                //                     current_map.data[index] = 100;
                //                 }
                //             }
                //         }
                //     }
                // }
            }
            // double backx = static_cast<double>(x+30)/100;
            // double backy = static_cast<double>(y-48)/100;
            // std::cout<<" The Scaled back to original values are: "<< backx<< " "<< backy<< " "<< std::endl;
            // std::cout<<"##########################ERROR######################"<< x_og-backx << " "<< y_og-backy<<std::endl;
            
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
