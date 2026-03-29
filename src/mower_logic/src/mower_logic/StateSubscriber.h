//
// Created by clemens on 29.11.24.
//

#ifndef STATESUBSCRIBER_H
#define STATESUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>

template <typename MESSAGE>
class StateSubscriber {
 public:
  explicit StateSubscriber(const std::string& topic);

  void Start(rclcpp::Node::SharedPtr n);

  MESSAGE getMessage();

  bool hasMessage();

  rclcpp::Time getMessageTime();

  void setMessage(const MESSAGE& message);

 private:
  std::string topic_;
  std::mutex message_mutex_{};
  MESSAGE message_{};
  rclcpp::Time last_message_time_{0, 0, RCL_ROS_TIME};
  bool has_message_ = false;
  typename rclcpp::Subscription<MESSAGE>::SharedPtr subscriber_{};
  rclcpp::Node::SharedPtr node_{};
};

template <typename MESSAGE>
StateSubscriber<MESSAGE>::StateSubscriber(const std::string& topic) : topic_{topic} {
}

template <typename MESSAGE>
void StateSubscriber<MESSAGE>::Start(rclcpp::Node::SharedPtr n) {
  node_ = n;
  subscriber_ = n->create_subscription<MESSAGE>(
      topic_, 10,
      [this](const typename MESSAGE::SharedPtr msg) {
        this->setMessage(*msg);
      });
}

template <typename MESSAGE>
MESSAGE StateSubscriber<MESSAGE>::getMessage() {
  std::lock_guard<std::mutex> lk{message_mutex_};
  return message_;
}

template <typename MESSAGE>
bool StateSubscriber<MESSAGE>::hasMessage() {
  std::lock_guard<std::mutex> lk{message_mutex_};
  return has_message_;
}

template <typename MESSAGE>
rclcpp::Time StateSubscriber<MESSAGE>::getMessageTime() {
  std::lock_guard<std::mutex> lk{message_mutex_};
  return last_message_time_;
}

template <typename MESSAGE>
void StateSubscriber<MESSAGE>::setMessage(const MESSAGE& message) {
  std::lock_guard<std::mutex> lk{message_mutex_};
  if (node_) {
    last_message_time_ = node_->get_clock()->now();
  }
  message_ = message;
  has_message_ = true;
}

#endif  // STATESUBSCRIBER_H
