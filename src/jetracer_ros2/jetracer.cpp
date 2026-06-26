#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#define head1 0xAA
#define head2 0x55
#define sendType_velocity    0x11
#define sendType_params      0x12
#define sendType_coefficient 0x13

using namespace std;
using namespace boost::asio;

class JetracerNode : public rclcpp::Node
{
public:
    JetracerNode() : Node("jetracer"), iosev(), sp(iosev)
    {
        // Declare parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<bool>("publish_odom_transform", true);
        this->declare_parameter<float>("linear_correction", 1.0);
        this->declare_parameter<float>("coefficient_a", -0.016073);
        this->declare_parameter<float>("coefficient_b", 0.176183);
        this->declare_parameter<float>("coefficient_c", -23.428084);
        this->declare_parameter<float>("coefficient_d", 1500.0);
        
        // PID and servo bias params (formerly dynamic reconfigure)
        this->declare_parameter<int>("kp", 350);
        this->declare_parameter<int>("ki", 120);
        this->declare_parameter<int>("kd", 0);
        this->declare_parameter<int>("servo_bias", 0);

        // Get initial parameters
        this->get_parameter("port_name", port_name_);
        this->get_parameter("publish_odom_transform", publish_odom_transform_);
        this->get_parameter("linear_correction", linear_correction_);
        this->get_parameter("coefficient_a", a_);
        this->get_parameter("coefficient_b", b_);
        this->get_parameter("coefficient_c", c_);
        this->get_parameter("coefficient_d", d_);
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);
        this->get_parameter("servo_bias", servo_bias_);

        // Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        lvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lvel", 10);
        rvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rvel", 10);
        lset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lset", 10);
        rset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rset", 10);

        odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&JetracerNode::cmd_callback, this, std::placeholders::_1));

        // Parameter callback
        param_subscriber_ = this->add_on_set_parameters_callback(
            std::bind(&JetracerNode::parametersCallback, this, std::placeholders::_1));

        // Serial Port Setup
        boost::system::error_code ec;
        sp.open(port_name_, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", port_name_.c_str(), ec.message().c_str());
            return;
        }
        sp.set_option(serial_port::baud_rate(115200));   
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));

        // Initial coefficient set
        SetCoefficient(a_, b_, c_, d_);
        SetParams(kp_, ki_, kd_, linear_correction_, servo_bias_);

        cmd_time_ = this->now();

        // Start serial receiving thread
        serial_thread_ = std::thread(&JetracerNode::serial_task, this);

        // Timer for periodic velocity command sending (50 Hz / 0.02s)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&JetracerNode::timer_callback, this));
    }

    ~JetracerNode()
    {
        if (sp.is_open()) {
            sp.close(); // This will interrupt the blocking read
        }
        if (serial_thread_.joinable()) {
            serial_thread_.join(); // Safely wait for thread to exit
        }
    }

private:
    io_service iosev;              
    serial_port sp;
    std::string port_name_;
    
    bool publish_odom_transform_;   
    int kp_;                        
    int ki_;
    int kd_;
    float linear_correction_ = 1.0;
    int servo_bias_ = 0;

    float a_;                        
    float b_;
    float c_;
    float d_;

    double x_ = 0.0;                
    double y_ = 0.0;
    double yaw_ = 0.0;
    rclcpp::Time cmd_time_;
    int startup_frames_discarded_ = 0;  // Discard first N frames — firmware sends
                                        // stale accumulated position before reset

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rset_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread serial_thread_;
    OnSetParametersCallbackHandle::SharedPtr param_subscriber_;

    uint8_t checksum(uint8_t* buf, size_t len)
    {
      uint8_t sum = 0x00;
      for(size_t i=0;i<len;i++)
      {
        sum += *(buf + i);
      }
      return sum;
    }

    void SetParams(int p, int i, int d, float linear_correction, int servo_bias) {
      static uint8_t tmp[15];
      tmp[0]  = head1;
      tmp[1]  = head2;
      tmp[2]  = 0x0F;
      tmp[3]  = sendType_params;
      tmp[4]  = (p>>8)&0xff;
      tmp[5]  = p&0xff;
      tmp[6]  = (i>>8)&0xff;
      tmp[7]  = i&0xff;
      tmp[8]  = (d>>8)&0xff;
      tmp[9]  = d&0xff;
      tmp[10] = (int16_t)((int16_t)(linear_correction*1000)>>8)&0xff;
      tmp[11] = (int16_t)(linear_correction*1000)&0xff;
      tmp[12] = ((int16_t)((int16_t)servo_bias)>>8)&0xff;
      tmp[13] = ((int16_t)servo_bias)&0xff;
      tmp[14] = checksum(tmp,14);
      write(sp, buffer(tmp, 15));
      RCLCPP_INFO(this->get_logger(), "set robot param: p=%d i=%d d=%d linear_correction=%f servo_bias=%d", 
                  p, i, d, linear_correction, servo_bias);
    }

    void SetCoefficient(float a, float b, float c, float d) {
      static uint8_t tmp[21];
      char* p;
      tmp[0]  = head1;
      tmp[1]  = head2;
      tmp[2]  = 0x15;
      tmp[3]  = sendType_coefficient;
      p=(char*)&a;
      tmp[4]  = p[0];
      tmp[5]  = p[1];
      tmp[6]  = p[2];
      tmp[7]  = p[3];
      p=(char*)&b;
      tmp[8]  = p[0];
      tmp[9]  = p[1];
      tmp[10] = p[2];
      tmp[11] = p[3];
      p=(char*)&c;
      tmp[12] = p[0];
      tmp[13] = p[1];
      tmp[14] = p[2];
      tmp[15] = p[3];
      p=(char*)&d;
      tmp[16] = p[0];
      tmp[17] = p[1];
      tmp[18] = p[2];
      tmp[19] = p[3];
      tmp[20] = checksum(tmp,20);
      write(sp, buffer(tmp, 21));
      RCLCPP_INFO(this->get_logger(), "set steering coefficient: a=%f b=%f c=%f d=%f", a, b, c, d);
    }

    void SetVelocity(double x, double y, double yaw)
    {
      static uint8_t tmp[11];
      tmp[0] = head1;
      tmp[1] = head2;
      tmp[2] = 0x0b;
      tmp[3] = sendType_velocity;
      tmp[4] = ((int16_t)(x*1000)>>8) & 0xff;
      tmp[5] = ((int16_t)(x*1000)) & 0xff;
      tmp[6] = ((int16_t)(y*1000)>>8) & 0xff;
      tmp[7] = ((int16_t)(y*1000)) & 0xff;
      tmp[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
      tmp[9] = ((int16_t)(yaw*1000)) & 0xff;
      tmp[10] = checksum(tmp,10);
      write(sp, buffer(tmp, 11));
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      x_ = msg->linear.x;
      y_ = msg->linear.x; // Preserving original code logic
      
      // Convert angular velocity (ω) from Nav2 to steering angle (δ) for firmware.
      // Bicycle model: δ = atan(L * ω / v), where L = wheelbase.
      // In ROS1, TEB had cmd_angle_instead_rotvel: True which sent the steering
      // angle directly. Nav2 controllers always send angular velocity instead,
      // so we must convert here.
      constexpr double WHEELBASE = 0.24;
      constexpr double MIN_TURNING_RADIUS = 0.50;
      // Max physical steering angle: atan(wheelbase / min_turning_radius)
      constexpr double MAX_STEER_ANGLE = 0.448; // atan(0.24/0.50) ≈ 0.448 rad ≈ 25.7°
      
      double v = msg->linear.x;
      double omega = msg->angular.z;
      
      if (std::abs(v) > 0.01) {
          // Normal case: compute steering angle from bicycle model
          yaw_ = std::atan(WHEELBASE * omega / v);
      } else if (std::abs(omega) > 0.01) {
          // Near-zero speed but controller wants turning (e.g., goal approach
          // heading correction). Use max steering in the requested direction
          // instead of clamping to 0 — that was causing overshoot/circling.
          yaw_ = (omega > 0.0) ? MAX_STEER_ANGLE : -MAX_STEER_ANGLE;
      } else {
          // Fully stopped, no turn requested
          yaw_ = 0.0;
      }
      
      // Clamp to physical steering limits — prevents impossible commands to firmware
      yaw_ = std::clamp(yaw_, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
      
      cmd_time_ = this->now();
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        
        bool param_changed = false;

        for (const auto &param : parameters) {
            if (param.get_name() == "kp") {
                kp_ = param.as_int();
                param_changed = true;
            } else if (param.get_name() == "ki") {
                ki_ = param.as_int();
                param_changed = true;
            } else if (param.get_name() == "kd") {
                kd_ = param.as_int();
                param_changed = true;
            } else if (param.get_name() == "servo_bias") {
                servo_bias_ = param.as_int();
                param_changed = true;
            } else if (param.get_name() == "linear_correction") {
                linear_correction_ = param.as_double();
                param_changed = true;
            }
        }

        if (param_changed) {
            SetParams(kp_, ki_, kd_, linear_correction_, servo_bias_);
        }

        return result;
    }

    void timer_callback()
    {
      rclcpp::Time current_time = this->now();
      
      // Stop if no cmd_vel received in 1 second
      if ((current_time - cmd_time_).seconds() > 1.0)
      {
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
      }
      SetVelocity(x_, y_, yaw_);
    }

    void serial_task()
    {
      enum frameState
      {
        State_Head1, State_Head2, State_Size, State_Data, State_CheckSum, State_Handle
      };
      
      frameState state = State_Head1;
      
      uint8_t frame_size=0, frame_sum=0, frame_type=0;
      uint8_t data[50];
      
      double  imu_list[9];
      double  odom_list[6];
      rclcpp::Time now_time, last_time;
      last_time = this->now();
      
      sensor_msgs::msg::Imu imu_msgs;
      geometry_msgs::msg::TransformStamped odom_trans;
      nav_msgs::msg::Odometry odom_msgs;
      std_msgs::msg::Int32 lvel_msgs;
      std_msgs::msg::Int32 rvel_msgs;
      std_msgs::msg::Int32 lset_msgs; 
      std_msgs::msg::Int32 rset_msgs;

      RCLCPP_INFO(this->get_logger(), "start receive message");
      while(rclcpp::ok() && sp.is_open())
      {
        try {
            switch (state)
            {
            case State_Head1:
                frame_sum = 0x00;
                read(sp, buffer(&data[0], 1));
                state = (data[0] == head1 ? State_Head2 : State_Head1);
                break;
                
            case State_Head2:
                read(sp, buffer(&data[1], 1));
                state = (data[1] == head2 ? State_Size : State_Head1);
                break;
                
            case State_Size:
                read(sp, buffer(&data[2], 1));
                frame_size = data[2];
                state = State_Data;
                break;
                
            case State_Data:
                read(sp, buffer(&data[3], frame_size - 4));
                frame_type = data[3];
                state = State_CheckSum;
                break;
                
            case State_CheckSum:
                read(sp, buffer(&data[frame_size -1], 1));
                frame_sum = checksum(data, frame_size - 1);
                state = data[frame_size -1] == frame_sum ? State_Handle : State_Head1;
                break;
                
            case State_Handle: {
                now_time = this->now();
                
                //gyro
                imu_list[0]=((double)((int16_t)(data[4]*256+data[5]))/32768*2000/180*3.1415);
                imu_list[1]=((double)((int16_t)(data[6]*256+data[7]))/32768*2000/180*3.1415);
                imu_list[2]=((double)((int16_t)(data[8]*256+data[9]))/32768*2000/180*3.1415);
                //Acc 
                imu_list[3]=((double)((int16_t)(data[10]*256+data[11]))/32768*2*9.8);
                imu_list[4]=((double)((int16_t)(data[12]*256+data[13]))/32768*2*9.8);
                imu_list[5]=((double)((int16_t)(data[14]*256+data[15]))/32768*2*9.8);
                //Angle 
                imu_list[6]=((double)((int16_t)(data[16]*256+data[17]))/10.0);
                imu_list[7]=((double)((int16_t)(data[18]*256+data[19]))/10.0);
                imu_list[8]=((double)((int16_t)(data[20]*256+data[21]))/10.0);

                //publish the IMU message
                imu_msgs.header.stamp = now_time;
                imu_msgs.header.frame_id = "base_imu_link";
                imu_msgs.angular_velocity.x = imu_list[0];
                imu_msgs.angular_velocity.y = imu_list[1];
                imu_msgs.angular_velocity.z = imu_list[2];
                imu_msgs.linear_acceleration.x = imu_list[3];
                imu_msgs.linear_acceleration.y = imu_list[4];
                imu_msgs.linear_acceleration.z = imu_list[5];
                
                tf2::Quaternion q_imu;
                q_imu.setRPY(0, 0, (imu_list[8]/180.0*3.1415926));
                imu_msgs.orientation.x = q_imu.x();
                imu_msgs.orientation.y = q_imu.y();
                imu_msgs.orientation.z = q_imu.z();
                imu_msgs.orientation.w = q_imu.w();
                
                imu_msgs.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.05};
                imu_msgs.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
                imu_msgs.linear_acceleration_covariance = {1e-2, 0, 0, 0, 0, 0, 0, 0, 0};
                imu_pub_->publish(imu_msgs);
            
                odom_list[0]=((double)((int16_t)(data[22]*256+data[23]))/1000);
                odom_list[1]=((double)((int16_t)(data[24]*256+data[25]))/1000);
                odom_list[2]=((double)((int16_t)(data[26]*256+data[27]))/1000);
                //dx dy dyaw base_frame
                odom_list[3]=((double)((int16_t)(data[28]*256+data[29]))/1000);
                odom_list[4]=((double)((int16_t)(data[30]*256+data[31]))/1000);
                odom_list[5]=((double)((int16_t)(data[32]*256+data[33]))/1000);
            
                tf2::Quaternion q_odom;
                q_odom.setRPY(0, 0, odom_list[2]);

                //publish the transform over tf
                odom_trans.header.stamp = now_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";

                odom_trans.transform.translation.x = odom_list[0];
                odom_trans.transform.translation.y = odom_list[1];
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation.x = q_odom.x();
                odom_trans.transform.rotation.y = q_odom.y();
                odom_trans.transform.rotation.z = q_odom.z();
                odom_trans.transform.rotation.w = q_odom.w();

                if(publish_odom_transform_) {
                    odom_broadcaster_->sendTransform(odom_trans);
                }

                //publish the odometry message
                odom_msgs.header.stamp = now_time;
                odom_msgs.header.frame_id = "odom";

                odom_msgs.pose.pose.position.x = odom_list[0];
                odom_msgs.pose.pose.position.y = odom_list[1];
                odom_msgs.pose.pose.position.z = 0.0;
                odom_msgs.pose.pose.orientation.x = q_odom.x();
                odom_msgs.pose.pose.orientation.y = q_odom.y();
                odom_msgs.pose.pose.orientation.z = q_odom.z();
                odom_msgs.pose.pose.orientation.w = q_odom.w();

                odom_msgs.child_frame_id = "base_footprint";
                
                {
                    double dt = (now_time - last_time).seconds();
                    if (dt > 0.0) {
                        odom_msgs.twist.twist.linear.x = odom_list[3] / dt;
                        odom_msgs.twist.twist.linear.y = odom_list[4] / dt;
                        odom_msgs.twist.twist.angular.z = odom_list[5] / dt;
                    }
                }
                
                odom_msgs.twist.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                                0, 1e-3, 1e-9, 0, 0, 0, 
                                                0, 0, 1e6, 0, 0, 0,
                                                0, 0, 0, 1e6, 0, 0, 
                                                0, 0, 0, 0, 1e6, 0, 
                                                0, 0, 0, 0, 0, 0.1 };
                odom_msgs.pose.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                                0, 1e-3, 1e-9, 0, 0, 0, 
                                                0, 0, 1e6, 0, 0, 0,
                                                0, 0, 0, 1e6, 0, 0, 
                                                0, 0, 0, 0, 1e6, 0, 
                                                0, 0, 0, 0, 0, 1e3 };
                odom_pub_->publish(odom_msgs);
                
                //publish the motor message
                lvel_msgs.data = ((int16_t)(data[34]*256+data[35]));
                rvel_msgs.data = ((int16_t)(data[36]*256+data[37]));
                lset_msgs.data = ((int16_t)(data[38]*256+data[39]));
                rset_msgs.data = ((int16_t)(data[40]*256+data[41]));
                
                lvel_pub_->publish(lvel_msgs);
                rvel_pub_->publish(rvel_msgs);
                lset_pub_->publish(lset_msgs);
                rset_pub_->publish(rset_msgs);

                last_time = now_time;
                state = State_Head1;
                break;
            }
            default:
                state = State_Head1;
                break;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Serial read error: %s", e.what());
            // Small delay to prevent tight loops on errors
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JetracerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
