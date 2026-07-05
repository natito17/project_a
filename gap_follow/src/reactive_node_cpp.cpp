#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// ── Tunable parameters (mirrors Python module-level constants) ────────────────
// See gap_follow/gap_follow/reactive_node.py for full documentation of each.
static constexpr double BUBBLE_RADIUS  = 0.20;  // [m]
static constexpr double MAX_RANGE_CLIP = 3.0;   // [m]
static constexpr int    SMOOTH_WINDOW  = 3;     // must be odd, ≥1
static constexpr double FAST_SPEED     = 3.0;   // [m/s]
static constexpr double MID_SPEED      = 1.5;   // [m/s]
static constexpr double SLOW_SPEED     = 0.5;   // [m/s]
static constexpr double FAST_THRESHOLD = 1.5;   // [m]
static constexpr double MID_THRESHOLD  = 0.5;   // [m]

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_node_cpp")
    {
        // Declare ROS2 parameters with the same defaults as the Python node.
        this->declare_parameter("scan_topic",      std::string("/scan"));
        this->declare_parameter("drive_topic",     std::string("/drive"));
        this->declare_parameter("bubble_radius",   BUBBLE_RADIUS);
        this->declare_parameter("max_range_clip",  MAX_RANGE_CLIP);
        this->declare_parameter("smooth_window",   SMOOTH_WINDOW);
        this->declare_parameter("fast_speed",      FAST_SPEED);
        this->declare_parameter("mid_speed",       MID_SPEED);
        this->declare_parameter("slow_speed",      SLOW_SPEED);
        this->declare_parameter("fast_threshold",  FAST_THRESHOLD);
        this->declare_parameter("mid_threshold",   MID_THRESHOLD);

        bubble_radius_  = this->get_parameter("bubble_radius").as_double();
        max_range_clip_ = this->get_parameter("max_range_clip").as_double();
        smooth_window_  = this->get_parameter("smooth_window").as_int();
        fast_speed_     = this->get_parameter("fast_speed").as_double();
        mid_speed_      = this->get_parameter("mid_speed").as_double();
        slow_speed_     = this->get_parameter("slow_speed").as_double();
        fast_threshold_ = this->get_parameter("fast_threshold").as_double();
        mid_threshold_  = this->get_parameter("mid_threshold").as_double();

        if (smooth_window_ < 1) smooth_window_ = 1;
        if (smooth_window_ % 2 == 0) smooth_window_++;

        const std::string scan_topic  = this->get_parameter("scan_topic").as_string();
        const std::string drive_topic = this->get_parameter("drive_topic").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10,
            std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

        RCLCPP_INFO(this->get_logger(), "ReactiveFollowGap C++ node started.");
    }

private:
    // ── ROS2 handles ─────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

    // ── Runtime parameters ────────────────────────────────────────────────────
    double bubble_radius_;
    double max_range_clip_;
    int    smooth_window_;
    double fast_speed_;
    double mid_speed_;
    double slow_speed_;
    double fast_threshold_;
    double mid_threshold_;

    // ── preprocess_lidar ──────────────────────────────────────────────────────
    // 1. Replace inf/nan with max_range_clip.
    // 2. Clip to [0, max_range_clip].
    // 3. Apply an edge-padded windowed-mean (box) filter.
    // Returns a new vector of the same length.
    std::vector<double> preprocess_lidar(const std::vector<float>& raw)
    {
        const int n = static_cast<int>(raw.size());
        std::vector<double> proc(n);

        // Step 1 & 2: sanitise
        for (int i = 0; i < n; ++i) {
            double v = static_cast<double>(raw[i]);
            if (!std::isfinite(v)) v = max_range_clip_;
            if (v < 0.0)           v = 0.0;
            if (v > max_range_clip_) v = max_range_clip_;
            proc[i] = v;
        }

        // Step 3: windowed mean with edge padding
        // Mirrors the Python: np.pad(mode='edge') + np.convolve(mode='valid')
        const int half_w = smooth_window_ / 2;

        // Build edge-padded array
        std::vector<double> padded(n + 2 * half_w);
        for (int i = 0; i < half_w; ++i)       padded[i]               = proc[0];
        for (int i = 0; i < n; ++i)            padded[half_w + i]       = proc[i];
        for (int i = 0; i < half_w; ++i)       padded[half_w + n + i]   = proc[n - 1];

        const double inv_w = 1.0 / smooth_window_;
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            for (int k = 0; k < smooth_window_; ++k)
                sum += padded[i + k];
            proc[i] = sum * inv_w;
        }

        return proc;
    }

    // ── find_max_gap ──────────────────────────────────────────────────────────
    // Returns (start_i, end_i) of the longest consecutive run of values > 0.
    // Returns (0, -1) if no non-zero element exists.
    std::pair<int,int> find_max_gap(const std::vector<double>& ranges)
    {
        int best_start = 0, best_len = 0;
        int cur_start  = -1;  // -1 means "not in a run"

        for (int i = 0; i < static_cast<int>(ranges.size()); ++i) {
            if (ranges[i] > 0.0) {
                if (cur_start < 0) cur_start = i;
                int run_len = i - cur_start + 1;
                if (run_len > best_len) {
                    best_len  = run_len;
                    best_start = cur_start;
                }
            } else {
                cur_start = -1;
            }
        }

        return {best_start, best_start + best_len - 1};
    }

    // ── find_best_point ───────────────────────────────────────────────────────
    // Center of the gap — stable against corridor oscillation.
    int find_best_point(int start_i, int end_i)
    {
        return (start_i + end_i) / 2;
    }

    // ── choose_speed ──────────────────────────────────────────────────────────
    double choose_speed(double min_range)
    {
        if (min_range > fast_threshold_) return fast_speed_;
        if (min_range > mid_threshold_)  return mid_speed_;
        return slow_speed_;
    }

    // ── lidar_callback ────────────────────────────────────────────────────────
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        const double angle_min       = scan_msg->angle_min;
        const double angle_increment = scan_msg->angle_increment;

        // Step 1: preprocess
        std::vector<double> proc = preprocess_lidar(scan_msg->ranges);
        const int n = static_cast<int>(proc.size());

        // Step 2: safety bubble
        // Find the index of the minimum range.
        int min_idx = static_cast<int>(
            std::min_element(proc.begin(), proc.end()) - proc.begin());
        double min_range = proc[min_idx];

        int bubble_half;
        if (min_range > 0.0) {
            bubble_half = static_cast<int>(
                std::ceil(bubble_radius_ / (min_range * angle_increment)));
        } else {
            bubble_half = n / 4;
        }

        int bubble_start = std::max(0, min_idx - bubble_half);
        int bubble_end   = std::min(n, min_idx + bubble_half + 1);
        for (int i = bubble_start; i < bubble_end; ++i) proc[i] = 0.0;

        // Step 3: find max gap
        auto [start_i, end_i] = find_max_gap(proc);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;

        // Edge case: all beams blocked
        if (end_i < start_i) {
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed          = static_cast<float>(slow_speed_);
            publisher_->publish(drive_msg);
            return;
        }

        // Step 4: best point → steering angle
        int best_idx   = find_best_point(start_i, end_i);
        double best_angle = angle_min + best_idx * angle_increment;

        // Step 5: speed — minimum of non-zero free-space ranges
        double effective_min = std::numeric_limits<double>::infinity();
        for (double v : proc) {
            if (v > 0.0 && v < effective_min) effective_min = v;
        }
        if (!std::isfinite(effective_min)) effective_min = 0.0;
        double speed = choose_speed(effective_min);

        // Step 6: publish
        drive_msg.drive.steering_angle = static_cast<float>(best_angle);
        drive_msg.drive.speed          = static_cast<float>(speed);
        publisher_->publish(drive_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}