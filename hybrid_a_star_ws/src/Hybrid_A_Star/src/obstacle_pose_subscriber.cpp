#include "hybrid_a_star/obstacle_pose_subscriber.h"

ObstaclePoseSubscriber2D::ObstaclePoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(
            topic_name, buff_size, &ObstaclePoseSubscriber2D::MessageCallBack, this
    );
}

void ObstaclePoseSubscriber2D::MessageCallBack(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data) {
    buff_mutex_.lock();
    // goal_poses_.emplace_back(goal_pose_ptr);
    // extract x and y 
    // std::cout<<"i am here"<< std::endl;
    // std::cout<< x_y_vals.size() <<std::endl;
    x_y_vals.clear();
    updateMapWithObstacles(obstacle_data);


    buff_mutex_.unlock();
}
bool scale_100_4 = true;
void ObstaclePoseSubscriber2D::updateMapWithObstacles(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data)
    {
        // std::cout<<"i am here now"<< std::endl;
        // buff_mutex_.lock();
        // -3.0419910440369464             2.9681722692357146
        int scale_factor = 1;
        int translate_x = 0;
        int translate_y = 0;
        double extend = 0.0;
        double halfWidth = 0.0;
        double halfHeight = 0.0;
        if (scale_100_4 ==true){
            scale_factor = 100;
            translate_x = -18; //-33;
            translate_y = 35;
            // extend = 13;
            extend = 40;
            halfWidth = 20; 
            halfHeight = 60;
        }
        else{
            scale_factor = 10;
//             x_translation = -3.2964026958248915
// y_translation = 3.539230053680539
            translate_x = -3;
            translate_y = 4;
            extend = 2.5;
        }
        // for (std::vector<float>::size_type i = 0; i < obstacle_data->data.size(); i += 5)
        // {
        //     int id = (obstacle_data->data[i]);
        //     // std::cout<< "id is "<< id <<"for "<< i << std::endl;
        //     int x = static_cast<int>(obstacle_data->data[i + 1]*scale_factor)+translate_x;
        //     int y = static_cast<int>(obstacle_data->data[i + 2]*scale_factor)+translate_y;
        //     double vel = std::abs(obstacle_data->data[i + 3]);
        //     if (id < 1000 && vel <= 0.05) // && x >= 0 && x < map_width && y >= 0 && y < map_height)
        //     {
        //         x_y_vals.push_back(x);
        //         x_y_vals.push_back(y);


        //         for (int pix = 1; pix < extend; pix++){
        //             // Update surrounding cells
        //             for (int dx = -pix; dx <= pix; dx++) {
        //                 for (int dy = -pix; dy <= pix; dy++) {
        //                     int new_x = x + dx;
        //                     int new_y = y + dy;
        //                     x_y_vals.push_back(new_x);
        //                     x_y_vals.push_back(new_y);
        //                 }
        //             }
        //         }
        //     }           
        // }
        

        for (std::vector<float>::size_type i = 0; i < obstacle_data->data.size(); i += 5)
        {

            int id = (obstacle_data->data[i]);
            int x = static_cast<int>(obstacle_data->data[i + 1]*scale_factor)+translate_x;
            int y = static_cast<int>(obstacle_data->data[i + 2]*scale_factor)+translate_y;
            double vel = std::abs(obstacle_data->data[i + 3]);
            if (id < 1000 && vel <= 0.05)
            {
                // std::cout<< "I have an obstical so something has 0 vel" << std::endl;
                x_y_vals.push_back(x);
                x_y_vals.push_back(y);

                for (double dx = -halfWidth; dx <= halfWidth; dx += 1.0) {
                    for (double dy = -halfHeight; dy <= halfHeight; dy += 1.0) {
                        double new_x = x + dx;
                        double new_y = y + dy;
                        x_y_vals.push_back(new_x);
                        x_y_vals.push_back(new_y);
                    }
                }



                // for (double pix = 0.5; pix < extend; pix += 1.0) { // Use double type for pix
                // // Update surrounding cells
                //     for (int dx = -static_cast<int>(pix); dx <= static_cast<int>(pix); dx++) {
                //         for (int dy = -static_cast<int>(pix); dy <= static_cast<int>(pix); dy++) {
                //             double new_x = x + dx;
                //             double new_y = y + dy;
                //             x_y_vals.push_back(new_x);
                //             x_y_vals.push_back(new_y);
                //         }
                //     }
                // }
            }
        }
 
        // buff_mutex_.unlock();
    }

std::vector<double> ObstaclePoseSubscriber2D::getX_Y_Vals() const {
    return x_y_vals;
}
// void ObstaclePoseSubscriber2D::ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff) {
//     buff_mutex_.lock();
//     // if (!goal_poses_.empty()) {
//     //     pose_data_buff.insert(pose_data_buff.end(), goal_poses_.begin(), goal_poses_.end());
//     //     goal_poses_.clear();
//     // }
//     // parse and expand
//     buff_mutex_.unlock();
// }



