#include <ncurses.h>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <map>


/*
To add a new key stroke follow these steps:

1. add letter or number to the map "key_oneshots" if a oneshot is desired
    e.g. key_oneshots['m'] = false;

2. add the functionality of the button press to the thread
    e.g. case 'm':
            // functionality
            key_oneshots['m'] = true;
            break;
*/


class Human: public rclcpp::Node {
    public:
    Human() : Node("human") {
        start_publisher = this->create_publisher<std_msgs::msg::String>("start",10);
        mode_publisher = this->create_publisher<std_msgs::msg::String>("mode",10);
        arm_disarm_publisher = this->create_publisher<std_msgs::msg::String>("arm_disarm",10);
        control_publisher = this->create_publisher<std_msgs::msg::String>("key_controls",10);

        // bullshit ---------
        initscr();
        raw();
        noecho();
        keypad(stdscr, TRUE);
        // -------------------


        input_thread_ = std::thread(&Human::inputLoop, this);

        key_oneshots['s'] = false;
        key_oneshots['p'] = false;
        key_oneshots['o'] = false;
    }

    ~Human() {
        endwin();

        input_running_ = false;
        input_thread_.join();
    }


    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr start_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_disarm_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_publisher;
    std::thread input_thread_;
    std::mutex input_mutex_;
    bool input_running_ = true;
    std::map<char, bool> key_oneshots;

    void inputLoop() {
        while (input_running_) {
            int ch = getch();

            std_msgs::msg::String message;
            switch (ch) {
                case 's':
                    if (!key_oneshots['s']) {
                        message.data = "s";
                        start_publisher->publish(message);
                    }
                    key_oneshots['s'] = true;
                    break;
                case 'q':
                    rclcpp::shutdown();
                    break;
                case 'p':
                    message.data = "p";
                    mode_publisher->publish(message);
                    break;
                case 'o':
                    message.data = "o";
                    mode_publisher->publish(message);
                    break;
                case '[':
                    message.data = "[";
                    mode_publisher->publish(message);
                    break;
                case '=':
                    message.data = "=";
                    mode_publisher->publish(message);
                    break;
                case 'a':
                    message.data = "a";
                    arm_disarm_publisher->publish(message);
                    break;
                case 'd':
                    message.data = "d";
                    arm_disarm_publisher->publish(message);
                    break;
                case 'y':
                    message.data = "y";
                    control_publisher->publish(message);
                    break;
                case 'g':
                    message.data = "g";
                    control_publisher->publish(message);
                    break;
                case 'h':
                    message.data = "h";
                    control_publisher->publish(message);
                    break;
                case 'j':
                    message.data = "j";
                    control_publisher->publish(message);
                    break;
                case 'l':
                    message.data = "l";
                    control_publisher->publish(message);
                    break;
                case ',':
                    message.data = ",";
                    control_publisher->publish(message);
                    break;
                case '.':
                    message.data = ".";
                    control_publisher->publish(message);
                    break;
                case '/':
                    message.data = "/";
                    control_publisher->publish(message);
                    break;
                default:
                    break;

            }
        }
    }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Human>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}