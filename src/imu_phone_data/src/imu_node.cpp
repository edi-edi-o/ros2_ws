#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <sstream>
#include <vector>
#include <cstdlib> // for system()
#include <chrono> // for std::chrono::milliseconds
#include <thread> // for std::this_thread::sleep_for
#include <cmath>   // for M_PI
#include <algorithm> // for std::swap

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node") {
        setupNetwork();
        if (!isSocketValid()) {
            RCLCPP_ERROR(this->get_logger(), "Network setup failed");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Network setup completed successfully");

        // Publisher setup for grouped data (Int32 arrays)
        accelerometer_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/imu/accelerometer", 10);
        magnetometer_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/imu/magnetometer", 10);
        gyroscope_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/imu/gyroscope", 10);
        gravity_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/imu/gravity", 10);
        linear_accel_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/imu/linear_acceleration", 10);
        rotation_vector_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/imu/rotation_vector", 10);
        orientation_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/imu/orientation", 10);
        raw_data_pub_ = this->create_publisher<std_msgs::msg::String>("/imu/raw_data", 10);

        // Publisher for effort data
        effort_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/imu/effort", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImuNode::publishData, this));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    bool isSocketValid() const {
        return new_socket_ >= 0;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr accelerometer_pub_, magnetometer_pub_, gyroscope_pub_, gravity_pub_, linear_accel_pub_, rotation_vector_pub_, orientation_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_data_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr effort_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int new_socket_ = -1;

    void setupNetwork() {
        // Create UDP socket
        if ((new_socket_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed: %s", strerror(errno));
            return;
        }

        // Set socket options
        int opt = 1;
        if (setsockopt(new_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Setting socket options failed: %s", strerror(errno));
            close(new_socket_);
            new_socket_ = -1;
            return;
        }

        struct sockaddr_in address;
        memset(&address, 0, sizeof(address)); // Zero out the structure
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(5678);

        // Bind
        if (bind(new_socket_, (const struct sockaddr *)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed: %s", strerror(errno));
            close(new_socket_);
            new_socket_ = -1;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "UDP server listening on port 5678");
    }

    void publishData() {
        if (!isSocketValid()) {
            RCLCPP_ERROR(this->get_logger(), "No active socket for reading data.");
            return;
        }

        char buffer[1024] = {0};
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int n = recvfrom(new_socket_, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client_addr, &addr_len);
        
        if (n < 0) {
            RCLCPP_ERROR(this->get_logger(), "Receive failed: %s", strerror(errno));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received %d bytes", n);

        std::string data(buffer, n);
        
        // Publish raw data for verification
        auto raw_data_msg = std_msgs::msg::String();
        raw_data_msg.data = data;
        raw_data_pub_->publish(raw_data_msg);
        RCLCPP_INFO(this->get_logger(), "Published raw data");

        // Parse and publish sensor data
        std::vector<std::string> values;
        std::istringstream iss(data);
        std::string val;
        while (std::getline(iss, val, ',')) {
            values.push_back(val);
        }

        // Assuming the first 22 fields are used for sensor data
        if (values.size() >= 22) {
            try {
                // Directly publish accelerometer data
                std_msgs::msg::Int32MultiArray accel_array;
                for (size_t i = 1; i <= 3; ++i) {
                    accel_array.data.push_back(std::stoi(values[i]) * 1000000);
                }
                accelerometer_pub_->publish(accel_array);

                // Publish orientation data for TF
                double yaw = std::stod(values[19]) * M_PI / 180.0;  // Convert to radians
                double pitch = std::stod(values[20]) * M_PI / 180.0; // Convert to radians
                double roll = std::stod(values[21]) * M_PI / 180.0;  // Convert to radians

                // Assuming that pitch and roll are swapped in the incoming data
                std::swap(pitch, roll);  // Swap pitch and roll

                // Invert if your sensor data is opposite to RViz conventions
                yaw = -yaw;  // Invert if necessary
                pitch = -pitch; // Assuming + is up in RViz but down in data
                roll = -roll;  // Assuming + is right roll in RViz but left in data

                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw); // Adjust order if needed, here we keep standard convention

                // Prepare the transform
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->now();
                transformStamped.header.frame_id = "world"; // Fixed frame
                transformStamped.child_frame_id = "imu_link"; // Child frame

                transformStamped.transform.translation.x = 0.0;
                transformStamped.transform.translation.y = 0.0;
                transformStamped.transform.translation.z = 0.0;

                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();

                // Broadcast the transform
                tf_broadcaster_->sendTransform(transformStamped);
                RCLCPP_DEBUG(this->get_logger(), "Published TF transform");

                // Publish effort data
                geometry_msgs::msg::WrenchStamped effort_msg;
                effort_msg.header.stamp = this->now();
                effort_msg.header.frame_id = "imu_link"; 

                // Example: using accelerometer data for force (this is just an example, adjust according to actual data)
                effort_msg.wrench.force.x = std::stod(values[1]);
                effort_msg.wrench.force.y = std::stod(values[2]);
                effort_msg.wrench.force.z = std::stod(values[3]);

                effort_pub_->publish(effort_msg);
                RCLCPP_DEBUG(this->get_logger(), "Published effort data");
            } catch (const std::invalid_argument& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert sensor or effort data: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Not enough values for sensor data");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    if (!node->isSocketValid()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to establish network connection");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Network connection established, proceeding to spin node.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}