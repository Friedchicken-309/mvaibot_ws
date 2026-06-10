#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

class mvaibot_485_driver : public rclcpp::Node
{
public:
    mvaibot_485_driver() : Node("mvaibot_485_driver"), serial_fd_(-1)
    {
        //声明并获取参数
        this->declare_parameter("axis_linear.x", 1);
        this->declare_parameter("scale_linear.x", 30.0);
        this->declare_parameter("scale_linear_turbo.x", 100.0);
        this->declare_parameter("axis_angular.yaw", 0);
        this->declare_parameter("enable_button", 8);
        this->declare_parameter("enable_turbo_button", 9);
        this->declare_parameter("button_dir", 0);
        this->declare_parameter("button_limit", 1);
        this->declare_parameter("button_mode", 3);
        this->declare_parameter("publish_rate", 50.0);
        this->declare_parameter("deadzone", 0.05);

        axis_linear_x_ = this->get_parameter("axis_linear.x").as_int();
        scale_linear_x_ = this->get_parameter("scale_linear.x").as_double();
        scale_linear_turbo_x_ = this->get_parameter("scale_linear_turbo.x").as_double();
        axis_angular_yaw_ = this->get_parameter("axis_angular.yaw").as_int();
        enable_button_ = this->get_parameter("enable_button").as_int();
        enable_turbo_button_ = this->get_parameter("enable_turbo_button").as_int();
        button_dir_ = this->get_parameter("button_dir").as_int();
        button_limit_ = this->get_parameter("button_limit").as_int();
        button_mode_ = this->get_parameter("button_mode").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        deadzone_ = this->get_parameter("deadzone").as_double();

        RCLCPP_INFO(this->get_logger(), "参数加载完成: scale_linear=%.2f", scale_linear_x_);

        //打开串口
        const char *serial_port = "/dev/ttyACM0";
        serial_fd_ = open(serial_port, O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", serial_port);
        } else {
            struct termios tty;
            tcgetattr(serial_fd_, &tty);
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~CRTSCTS;
            tty.c_cflag |= CREAD | CLOCAL;
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
            tty.c_oflag &= ~OPOST;
            tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            tcsetattr(serial_fd_, TCSANOW, &tty);
            RCLCPP_INFO(this->get_logger(), "串口已打开: %s @ 9600", serial_port);
        }

        //创建Subscription，订阅/joy话题，在回调函数中进行控制量更新
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&mvaibot_485_driver::joy_callback, this, std::placeholders::_1));

        //创建定时器 50Hz
        int period_ms = static_cast<int>(1000.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&mvaibot_485_driver::timer_callback, this));

    }

    ~mvaibot_485_driver()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    //创建回调函数
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // 检查数据长度
        int max_axis = std::max(axis_linear_x_, axis_angular_yaw_);
        int max_btn = std::max({enable_button_, enable_turbo_button_, button_dir_, button_limit_, button_mode_});
        if ((int)msg->axes.size() <= max_axis || (int)msg->buttons.size() <= max_btn) return;

        // 使能：buttons[8] 按住时运动使能
        enabled_ = (msg->buttons[enable_button_] == 1);

        // 加速：buttons[9] 按住时加速
        turbo_ = (msg->buttons[enable_turbo_button_] == 1);

        // 速度：使能且axes[1]大于死区时才计算
        speed_ = 0;
        float linear_val = msg->axes[axis_linear_x_];

        if (enabled_ && linear_val > deadzone_) {
            double scale = turbo_ ? scale_linear_turbo_x_ : scale_linear_x_;
            speed_ = static_cast<uint8_t>(std::min(linear_val * scale, 100.0));
        }

        // 方向角：axes[0] 映射到 0-180，死区内保持90
        float angular_val = msg->axes[axis_angular_yaw_];
        if (std::fabs(angular_val) < deadzone_) {
            angle_ = 90;
        } else {
            int a = static_cast<int>(std::round(90.0 - angular_val * 90.0));
            angle_ = static_cast<uint8_t>(std::max(0, std::min(a, 180)));
        }

        // 边沿检测：首次收到消息时初始化prev_buttons_
        if (prev_buttons_.empty()) {
            prev_buttons_ = msg->buttons;
            return;
        }

        // 边沿检测：按钮按下时切换（上升沿触发）
        // buttons[0] 切换行驶方向：前进↔后退
        if (msg->buttons[button_dir_] == 1 && prev_buttons_[button_dir_] == 0) {
            dir_ = (dir_ == 2) ? 1 : 2;
            RCLCPP_INFO(this->get_logger(), "方向切换: %s", dir_ == 2 ? "前进" : "后退");
        }
        // buttons[1] 切换限位状态：0→1→2→3→0
        if (msg->buttons[button_limit_] == 1 && prev_buttons_[button_limit_] == 0) {
            limit_ = (limit_ + 1) % 4;
            const char *limit_str[] = {"正常", "前限位", "后限位", "急停"};
            RCLCPP_INFO(this->get_logger(), "限位切换: %s", limit_str[limit_]);
        }
        // buttons[3] 切换控制模式：0→1→2→3→0
        if (msg->buttons[button_mode_] == 1 && prev_buttons_[button_mode_] == 0) {
            mode_ = (mode_ + 1) % 4;
            const char *mode_str[] = {"遥控", "全向移动", "原地转向", "双阿克曼"};
            RCLCPP_INFO(this->get_logger(), "模式切换: %s", mode_str[mode_]);
        }

        prev_buttons_ = msg->buttons;
    }

    void timer_callback()
    {
        uint8_t frame[8];
        frame[0] = 0x33;    // 起始位
        frame[1] = mode_;   // mode
        frame[2] = dir_;    // dir
        frame[3] = speed_;  // speed
        frame[4] = angle_;  // angle
        frame[5] = limit_;  // limit
        frame[6] = 0x00;    // 预留

        // 校验和：前7位相加取低八位
        uint8_t sum = 0;
        for (int i = 0; i < 7; i++) {
            sum += frame[i];
        }
        frame[7] = sum & 0xFF;

        send_frame(frame, 8);
    }

    void send_frame(const uint8_t *data, size_t len)
    {
        if (serial_fd_ < 0) return;

        std::string hex_str;
        for (size_t i = 0; i < len; i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", data[i]);
            hex_str += buf;
        }
        const char *mode_str[] = {"遥控", "全向移动", "原地转向", "双阿克曼"};
        const char *dir_str[] = {"停止", "后退", "前进"};
        const char *limit_str[] = {"正常", "前限位", "后限位", "急停"};
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "发送: [%s] 模式:%s 方向:%s 速度:%d 角度:%d 限位:%s 使能:%d",
                    hex_str.c_str(), mode_str[mode_], dir_str[dir_], speed_, angle_, limit_str[limit_], enabled_);

        ssize_t ret = write(serial_fd_, data, len);
        if (ret < 0) {
            RCLCPP_WARN(this->get_logger(), "串口发送失败");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_fd_ = -1;

    //参数变量
    int axis_linear_x_;
    double scale_linear_x_;
    double scale_linear_turbo_x_;
    int axis_angular_yaw_;
    int enable_button_;
    int enable_turbo_button_;
    int button_dir_;
    int button_limit_;
    int button_mode_;
    double publish_rate_;
    double deadzone_;

    //控制状态
    uint8_t mode_ = 3;     // 0遥控 1全向 2原地转向 3双阿克曼
    uint8_t dir_ = 2;      // 0停止 1后退 2前进
    uint8_t speed_ = 0;    // 0-100
    uint8_t angle_ = 90;   // 0-180
    uint8_t limit_ = 0;    // 0正常 1前限位 2后限位 3急停
    bool enabled_ = false;
    bool turbo_ = false;
    std::vector<int32_t> prev_buttons_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mvaibot_485_driver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}