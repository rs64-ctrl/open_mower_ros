#include "PerimeterDocking.h"

#include <mower_msgs/msg/power.hpp>

#include "IdleBehavior.h"
#include "MowingBehavior.h"
#include "mower_msgs/msg/perimeter.hpp"
#include "mower_msgs/srv/perimeter_control_srv.hpp"

#define MIN_SIGNAL 5
#define SEARCH_SPEED 0.1
#define ANGULAR_SPEED 0.5

/* Distance from center to outer coil */
#define COIL_Y_OFFSET 0.11
/* Distance from rear axis to coils */
#define COIL_X_OFFSET 0.40

#define FOLLOW_STATE_FOLLOW 0
#define FOLLOW_STATE_TURN_IN 1
#define FOLLOW_STATE_TURN_OUT 2

extern rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
extern mower_msgs::msg::Status getStatus();
extern mower_msgs::msg::Power getPower();
extern void setGPS(bool enabled);

static rclcpp::Subscription<mower_msgs::msg::Perimeter>::SharedPtr perimeterSubscriber;
static rclcpp::Client<mower_msgs::srv::PerimeterControlSrv>::SharedPtr perimeterClient;

static mower_msgs::msg::Perimeter lastPerimeter;
static int perimeterUpdated = 0;
static int direction;
static double calibrationLeft, calibrationRight, maxCenter;
static int signCenter;

PerimeterSearchBehavior PerimeterSearchBehavior::INSTANCE;
PerimeterDockingBehavior PerimeterDockingBehavior::INSTANCE;
PerimeterUndockingBehavior PerimeterUndockingBehavior::INSTANCE;
PerimeterMoveToGpsBehavior PerimeterMoveToGpsBehavior::INSTANCE;

static void perimeterReceived(const mower_msgs::msg::Perimeter::SharedPtr msg) {
  lastPerimeter = *msg;
  perimeterUpdated = 1;
}

static Behavior* shutdownConnections() {
  auto req = std::make_shared<mower_msgs::srv::PerimeterControlSrv::Request>();
  req->listen_on = 0;
  auto result = perimeterClient->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rosNode->get_logger(), "Perimeter deactivated.");
  } else {
    RCLCPP_ERROR(rosNode->get_logger(), "Failed to deactivate perimeter.");
  }
  perimeterSubscriber.reset();
  perimeterClient.reset();
  return &IdleBehavior::INSTANCE;
}

static int isPerimeterUpdated() {
  if (perimeterUpdated) {
    perimeterUpdated = 0;
    return 1;
  }
  return 0;
}

static float innerSignal() {
  return direction > 0 ? lastPerimeter.left * calibrationLeft : lastPerimeter.right * calibrationRight;
}

static float outerSignal() {
  return direction > 0 ? lastPerimeter.right * calibrationRight : lastPerimeter.left * calibrationLeft;
}

int PerimeterSearchBehavior::configured(const mower_logic::MowerLogicConfig& config) {
  return config.perimeter_signal != 0;
}

std::string PerimeterSearchBehavior::state_name() {
  return "DOCKING";
}

Behavior* PerimeterSearchBehavior::execute() {
  if (!setupConnections()) return shutdownConnections();
  rclcpp::Rate rate(10);
  int tries = 100;
  int toFind = 5;
  while (tries-- && toFind) {
    if (isPerimeterUpdated()) {
      toFind--;
    }
    rate.sleep();
  }

  if (toFind) {
    RCLCPP_ERROR(rosNode->get_logger(), "Failed to activate perimeter");
    return shutdownConnections();
  }

  calibrationLeft = calibrationRight = signCenter =
      lastPerimeter.left < 0 ? 1 : -1;

  if (innerSignal() > -MIN_SIGNAL || outerSignal() > -MIN_SIGNAL) {
    RCLCPP_ERROR(rosNode->get_logger(), "Signal too weak.");
    return shutdownConnections();
  }
  calibrationLeft /= fabs(lastPerimeter.left);
  maxCenter = fabs(lastPerimeter.center);
  calibrationRight /= fabs(lastPerimeter.right);

  geometry_msgs::msg::Twist vel;
  vel.angular.z = 0;
  vel.linear.x = SEARCH_SPEED;
  tries = 10;
  while (tries-- > 0) {
    cmd_vel_pub->publish(vel);
    rate.sleep();
    if (isPerimeterUpdated()) {
      tries = 10;
      if (innerSignal() > 0 || outerSignal() > 0) break;
    }
  }

  if (!tries) {
    RCLCPP_ERROR(rosNode->get_logger(), "Signal timeout");
    return shutdownConnections();
  }

  for (tries = 20; tries-- > 0;) {
    cmd_vel_pub->publish(vel);
    rate.sleep();
  }

  return &PerimeterDockingBehavior::INSTANCE;
}

int PerimeterUndockingBehavior::configured(const mower_logic::MowerLogicConfig& config) {
  return config.perimeter_signal != 0 && config.undock_distance > 1.0;
}

std::string PerimeterUndockingBehavior::state_name() {
  return "UNDOCKING";
}

Behavior* PerimeterUndockingBehavior::execute() {
  if (!setupConnections()) return shutdownConnections();
  rclcpp::Rate rate(10);
  int tries = 100;
  int toFind = 5;
  while (tries-- && toFind) {
    if (isPerimeterUpdated()) {
      toFind--;
    }
    rate.sleep();
  }

  if (toFind) {
    RCLCPP_ERROR(rosNode->get_logger(), "Failed to activate perimeter");
    return shutdownConnections();
  }

  geometry_msgs::msg::Twist vel;
  vel.angular.z = 0;
  vel.linear.x = -SEARCH_SPEED;
  double travelled = 0;
  while (travelled < 0.5) {
    cmd_vel_pub->publish(vel);
    rate.sleep();
    travelled += std::chrono::duration<double>(rate.period()).count() * SEARCH_SPEED;
  }

  calibrationLeft = calibrationRight = signCenter = 1;
  if (innerSignal() < 0) calibrationLeft = calibrationRight = signCenter = -1;

  float maxLeft = lastPerimeter.left * calibrationLeft;
  maxCenter = fabs(lastPerimeter.center);
  float maxRight = lastPerimeter.right * calibrationRight;

  vel.angular.z = direction * ANGULAR_SPEED;
  vel.linear.x = 0;
  tries = 240;
  while (innerSignal() > 0 && --tries) {
    cmd_vel_pub->publish(vel);
    rate.sleep();
    if (isPerimeterUpdated()) {
      float x = lastPerimeter.left * calibrationLeft;
      if (x > maxLeft) maxLeft = x;
      x = fabs(lastPerimeter.center);
      if (x > maxCenter) maxCenter = x;
      x = lastPerimeter.right * calibrationRight;
      if (x > maxRight) maxRight = x;
    }
  }
  if (!tries) {
    RCLCPP_ERROR(rosNode->get_logger(), "Could not turn inwards");
    return shutdownConnections();
  }

  if (maxLeft < MIN_SIGNAL || maxRight < MIN_SIGNAL) {
    RCLCPP_ERROR(rosNode->get_logger(), "Signal too weak.");
    return shutdownConnections();
  }

  direction = -direction;
  calibrationLeft /= maxLeft;
  calibrationRight /= maxRight;
  return &PerimeterMoveToGpsBehavior::INSTANCE;
}

std::string PerimeterDockingBehavior::state_name() {
  return "DOCKING";
}

Behavior* PerimeterDockingBehavior::arrived() {
  if (travelled > config.docking_distance) {
    RCLCPP_WARN(rosNode->get_logger(), "Travelled %.f meters before reaching the station", travelled);
    return &IdleBehavior::INSTANCE;
  }
  if (getPower().v_charge > 5.0) {
    chargeSeen++;
    if (chargeSeen >= 2) {
      chargeSeen = 0;
      return &IdleBehavior::DOCKED_INSTANCE;
    }
  } else {
    chargeSeen = 0;
  }
  return NULL;
}

Behavior* PerimeterFollowBehavior::execute() {
  rclcpp::Rate rate(10);
  geometry_msgs::msg::Twist vel;
  travelled = 0;
  int state = FOLLOW_STATE_FOLLOW;
  double travelTimeSinceUpdate = 0;
  double lastAlpha0 = 0;
  int tries = 0;
  perimeterUpdated = 1;
  Behavior* toReturn;
  double drift = 0;
  double averageInterval = 5;
  while (rclcpp::ok() && !(toReturn = arrived())) {
    if (!isPerimeterUpdated()) {
      cmd_vel_pub->publish(vel);
      rate.sleep();
      if (tries && --tries == 0) {
        RCLCPP_ERROR(rosNode->get_logger(), "Timeout of action %d", state);
        toReturn = &IdleBehavior::INSTANCE;
        break;
      }
      double d = std::chrono::duration<double>(rate.period()).count();
      travelled += d * vel.linear.x;
      travelTimeSinceUpdate += d;
      continue;
    }
    if (innerSignal() < 0) {
      vel.linear.x = 0;
      vel.angular.z = ANGULAR_SPEED * direction;
      if (state != FOLLOW_STATE_TURN_IN) {
        tries = 240;
        state = FOLLOW_STATE_TURN_IN;
      }
    } else if (outerSignal() > 0) {
      vel.linear.x = 0;
      vel.angular.z = -ANGULAR_SPEED * direction;
      if (state != FOLLOW_STATE_TURN_OUT) {
        tries = 240;
        state = FOLLOW_STATE_TURN_OUT;
      }
    } else {
      vel.linear.x = SEARCH_SPEED;
      if (fabs(lastPerimeter.center) > maxCenter) {
        maxCenter = fabs(lastPerimeter.center);
      }
      double c = lastPerimeter.center * signCenter;
      double y0 = -direction * COIL_Y_OFFSET * c / maxCenter;
      double alpha0 = y0 / COIL_X_OFFSET;
      if (state != FOLLOW_STATE_FOLLOW) {
        state = FOLLOW_STATE_FOLLOW;
        tries = 0;
      } else {
        if (travelTimeSinceUpdate > 0) {
          double d0 = -vel.angular.z + (alpha0 - lastAlpha0) / travelTimeSinceUpdate;
          double f = exp(-travelTimeSinceUpdate / averageInterval);
          drift = drift * f + d0 * (1 - f);
        }
      }
      vel.angular.z = drift + alpha0 / 2;
      if (vel.angular.z > ANGULAR_SPEED)
        vel.angular.z = ANGULAR_SPEED;
      else if (vel.angular.z < -ANGULAR_SPEED)
        vel.angular.z = -ANGULAR_SPEED;
      travelTimeSinceUpdate = 0;
      lastAlpha0 = alpha0;
    }
  }
  vel.linear.x = 0;
  vel.angular.z = 0;
  cmd_vel_pub->publish(vel);
  shutdownConnections();
  return toReturn;
}

void PerimeterBase::enter() {
  paused = aborted = false;
}

void PerimeterBase::exit() {
}

void PerimeterBase::reset() {
}

uint8_t PerimeterBase::get_sub_state() {
  return 1;
}

uint8_t PerimeterBase::get_state() {
  return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

bool PerimeterBase::needs_gps() {
  return false;
}

bool PerimeterBase::mower_enabled() {
  return false;
}

void PerimeterBase::command_home() {
}

void PerimeterBase::command_start() {
}

void PerimeterBase::command_s1() {
}

void PerimeterBase::command_s2() {
}

bool PerimeterBase::redirect_joystick() {
  return false;
}

void PerimeterBase::handle_action(std::string action) {
}

int PerimeterBase::setupConnections() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  perimeterSubscriber = rosNode->create_subscription<mower_msgs::msg::Perimeter>(
      "/mower/perimeter", qos, perimeterReceived);
  perimeterClient = rosNode->create_client<mower_msgs::srv::PerimeterControlSrv>("/mower_service/perimeter_listen");
  auto req = std::make_shared<mower_msgs::srv::PerimeterControlSrv::Request>();
  direction = config.perimeter_signal > 0 ? 1 : -1;
  req->listen_on = direction * config.perimeter_signal;
  auto result = perimeterClient->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rosNode->get_logger(), "Perimeter activated");
    return 1;
  }
  RCLCPP_ERROR(rosNode->get_logger(), "Failed to activate perimeter");
  return 0;
}

void PerimeterMoveToGpsBehavior::enter() {
  setGPS(true);
}

std::string PerimeterMoveToGpsBehavior::state_name() {
  return "UNDOCKING";
}

Behavior* PerimeterMoveToGpsBehavior::arrived() {
  return travelled >= config.undock_distance - 0.9 ? &MowingBehavior::INSTANCE : NULL;
}
