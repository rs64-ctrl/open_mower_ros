// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/PolygonRosConverter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Rosbag2 for reading legacy map files
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rclcpp/serialization.hpp>

// Include Messages
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "mower_map/msg/map_area.hpp"

// Include Service Messages
#include "mower_map/srv/add_mowing_area_srv.hpp"
#include "mower_map/srv/clear_map_srv.hpp"
#include "mower_map/srv/clear_nav_point_srv.hpp"
#include "mower_map/srv/get_docking_point_srv.hpp"
#include "mower_map/srv/get_mowing_area_srv.hpp"
#include "mower_map/srv/set_docking_point_srv.hpp"
#include "mower_map/srv/set_nav_point_srv.hpp"

// JSON for map storage
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>
using json = nlohmann::ordered_json;

// Monitoring
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "xbot_msgs/msg/map_size.hpp"

// RPC
#include "xbot_rpc/provider.h"

const std::string MAP_FILE = "map.json";
const std::string LEGACY_MAP_FILE = "map.bag";

// Struct definitions for JSON serialization
struct Point {
  double x;
  double y;
};

typedef std::vector<Point> Polygon;

struct MapArea {
  std::string id;
  std::string name;
  std::string type;
  bool active;
  Polygon outline;
};

struct DockingStation {
  std::string id;
  std::string name;
  bool active;
  Point position;
  double heading;
};

struct MapData {
  std::vector<MapArea> areas;
  std::vector<DockingStation> docking_stations;

  std::vector<MapArea> getMowingAreas() {
    std::vector<MapArea> result;
    for (const auto& area : areas) {
      if (area.type == "mow") result.push_back(area);
    }
    return result;
  }

  void clear() {
    areas.clear();
    docking_stations.clear();
  }

  std::string toJsonString();
};

// JSON serialization for Point — must use ordered_json explicitly
void to_json(json& j, const Point& p) {
  j["x"] = p.x;
  j["y"] = p.y;
}

void from_json(const json& j, Point& p) {
  j.at("x").get_to(p.x);
  j.at("y").get_to(p.y);
}

void to_json(json& j, const MapArea& data) {
  j["id"] = data.id;
  json properties = json::object();
  if (!data.name.empty()) properties["name"] = data.name;
  properties["type"] = data.type;
  if (!data.active) properties["active"] = data.active;
  j["properties"] = properties;
  j["outline"] = data.outline;
}

void from_json(const json& j, MapArea& data) {
  j.at("id").get_to(data.id);
  const auto& properties = j.value("properties", json::object());
  data.name = properties.value("name", "");
  data.type = properties.value("type", "draft");
  data.active = properties.value("active", true);
  j.at("outline").get_to(data.outline);
}

void to_json(json& j, const DockingStation& data) {
  j["id"] = data.id;
  json properties = json::object();
  if (!data.name.empty()) properties["name"] = data.name;
  if (!data.active) properties["active"] = data.active;
  if (!properties.empty()) j["properties"] = properties;
  j["position"] = data.position;
  j["heading"] = data.heading;
}

void from_json(const json& j, DockingStation& data) {
  j.at("id").get_to(data.id);
  const auto& properties = j.value("properties", json::object());
  data.name = properties.value("name", "");
  data.active = properties.value("active", true);
  j.at("position").get_to(data.position);
  j.at("heading").get_to(data.heading);
}

void to_json(json& j, const MapData& data) {
  j["areas"] = data.areas;
  j["docking_stations"] = data.docking_stations;
}

void from_json(const json& j, MapData& data) {
  j.at("areas").get_to(data.areas);
  j.at("docking_stations").get_to(data.docking_stations);
}

std::string MapData::toJsonString() {
  json json_data;
  to_json(json_data, *this);
  return json_data.dump(2);
}

std::string generateNanoId(size_t length = 32) {
  static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  thread_local std::mt19937 rng{std::random_device{}()};
  thread_local std::uniform_int_distribution<> dist(0, sizeof(alphabet) - 2);
  std::string id(length, '\0');
  std::generate_n(id.begin(), length, [&]() { return alphabet[dist(rng)]; });
  return id;
}

/**
 * Convert a geometry_msgs::msg::Polygon to our internal Polygon struct
 */
Polygon geometryPolygonToInternal(const geometry_msgs::msg::Polygon& poly) {
  Polygon result;
  for (const auto& point : poly.points) {
    result.push_back({point.x, point.y});
  }
  return result;
}

/**
 * Convert our internal Polygon struct to geometry_msgs::msg::Polygon
 */
geometry_msgs::msg::Polygon internalPolygonToGeometry(const Polygon& poly) {
  geometry_msgs::msg::Polygon result;
  for (const auto& point : poly) {
    geometry_msgs::msg::Point32 pt;
    pt.x = point.x;
    pt.y = point.y;
    result.points.push_back(pt);
  }
  return result;
}

/**
 * Convert a mower_map::msg::MapArea to our internal MapArea struct
 */
MapArea mowerMapAreaToInternal(const geometry_msgs::msg::Polygon& area, const std::string& type, const std::string& name) {
  MapArea result;
  result.id = generateNanoId();
  result.type = type;
  result.name = name;
  result.active = true;
  result.outline = geometryPolygonToInternal(area);
  return result;
}

/**
 * Convert our internal MapArea struct to mower_map::msg::MapArea
 */
mower_map::msg::MapArea internalMapAreaToMower(const MapArea& area) {
  mower_map::msg::MapArea result;
  result.name = area.name;
  // Leave area empty if it is not active
  if (area.active) {
    result.area = internalPolygonToGeometry(area.outline);
  }
  return result;
}

grid_map::Polygon internalPolygonToGridMap(const Polygon& poly) {
  grid_map::Polygon result;
  for (const auto& point : poly) {
    result.addVertex(grid_map::Position(point.x, point.y));
  }
  return result;
}

/**
 * MowerMapServiceNode: ROS2 node for map management
 */
class MowerMapServiceNode : public rclcpp::Node {
public:
  MowerMapServiceNode() : Node("mower_map_service") {
    // Publishers (all latched via transient local QoS)
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto latched_qos_10 = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();

    json_map_pub_ = this->create_publisher<std_msgs::msg::String>("mower_map_service/json_map", latched_qos);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mower_map_service/map", latched_qos_10);
    map_server_viz_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mower_map_service/map_viz", latched_qos_10);
    map_size_pub_ = this->create_publisher<xbot_msgs::msg::MapSize>("mower_map_service/map_size", latched_qos_10);

    // Load map
    if (!std::filesystem::exists(MAP_FILE) && std::filesystem::exists(LEGACY_MAP_FILE)) {
      RCLCPP_INFO(this->get_logger(), "Found legacy map file, converting to JSON...");
      convertLegacyMapToJson();
    } else {
      readMapFromFile();
    }

    buildMap();

    // Services
    add_area_srv_ = this->create_service<mower_map::srv::AddMowingAreaSrv>(
      "mower_map_service/add_mowing_area",
      std::bind(&MowerMapServiceNode::addMowingArea, this, std::placeholders::_1, std::placeholders::_2));
    get_area_srv_ = this->create_service<mower_map::srv::GetMowingAreaSrv>(
      "mower_map_service/get_mowing_area",
      std::bind(&MowerMapServiceNode::getMowingArea, this, std::placeholders::_1, std::placeholders::_2));
    set_docking_point_srv_ = this->create_service<mower_map::srv::SetDockingPointSrv>(
      "mower_map_service/set_docking_point",
      std::bind(&MowerMapServiceNode::setDockingPoint, this, std::placeholders::_1, std::placeholders::_2));
    get_docking_point_srv_ = this->create_service<mower_map::srv::GetDockingPointSrv>(
      "mower_map_service/get_docking_point",
      std::bind(&MowerMapServiceNode::getDockingPoint, this, std::placeholders::_1, std::placeholders::_2));
    set_nav_point_srv_ = this->create_service<mower_map::srv::SetNavPointSrv>(
      "mower_map_service/set_nav_point",
      std::bind(&MowerMapServiceNode::setNavPoint, this, std::placeholders::_1, std::placeholders::_2));
    clear_nav_point_srv_ = this->create_service<mower_map::srv::ClearNavPointSrv>(
      "mower_map_service/clear_nav_point",
      std::bind(&MowerMapServiceNode::clearNavPoint, this, std::placeholders::_1, std::placeholders::_2));
    clear_map_srv_ = this->create_service<mower_map::srv::ClearMapSrv>(
      "mower_map_service/clear_map",
      std::bind(&MowerMapServiceNode::clearMap, this, std::placeholders::_1, std::placeholders::_2));
  }

  void initRpc(rclcpp::Node::SharedPtr node_ptr) {
    rpc_provider_ = std::make_unique<xbot_rpc::RpcProvider>(node_ptr, "mower_map_service");
    rpc_provider_->addMethod("map.replace",
        [this](const std::string& /*method*/, const nlohmann::basic_json<>& params) -> nlohmann::basic_json<> {
        if (!params.is_array() || params.size() != 1) {
          throw xbot_rpc::RpcException(xbot_rpc::msg::RpcError::ERROR_INVALID_PARAMS, "Missing map parameter");
        }
        try {
          map_data_.areas.clear();
          map_data_.docking_stations.clear();
          json new_data_json = params[0];
          MapData new_data;
          from_json(new_data_json, new_data);
          map_data_ = new_data;
        } catch (const xbot_rpc::RpcException&) {
          throw;
        } catch (const std::exception& e) {
          throw xbot_rpc::RpcException(xbot_rpc::msg::RpcError::ERROR_INVALID_PARAMS, "Invalid map: " + std::string(e.what()));
        }
        saveMapToFile();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu areas via RPC and saved to file", map_data_.areas.size());
        buildMap();
        return "Successfully stored map (" + std::to_string(map_data_.areas.size()) + " areas)";
    });
    rpc_provider_->init();
  }

private:
  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_server_viz_array_pub_;
  rclcpp::Publisher<xbot_msgs::msg::MapSize>::SharedPtr map_size_pub_;

  // Services
  rclcpp::Service<mower_map::srv::AddMowingAreaSrv>::SharedPtr add_area_srv_;
  rclcpp::Service<mower_map::srv::GetMowingAreaSrv>::SharedPtr get_area_srv_;
  rclcpp::Service<mower_map::srv::SetDockingPointSrv>::SharedPtr set_docking_point_srv_;
  rclcpp::Service<mower_map::srv::GetDockingPointSrv>::SharedPtr get_docking_point_srv_;
  rclcpp::Service<mower_map::srv::SetNavPointSrv>::SharedPtr set_nav_point_srv_;
  rclcpp::Service<mower_map::srv::ClearNavPointSrv>::SharedPtr clear_nav_point_srv_;
  rclcpp::Service<mower_map::srv::ClearMapSrv>::SharedPtr clear_map_srv_;

  // RPC provider
  std::unique_ptr<xbot_rpc::RpcProvider> rpc_provider_;

  // MapData instance - the source of truth for map data
  MapData map_data_;

  bool show_fake_obstacle_ = false;
  geometry_msgs::msg::Pose fake_obstacle_pose_;

  // The grid map. This is built from the polygons loaded from the file.
  grid_map::GridMap map_;

  /**
   * Publish map to xbot_monitoring
   */
  void publishMapMonitoring() {
    xbot_msgs::msg::MapSize map_size;
    // NOTE: ROS2 rosidl converts camelCase .msg fields to snake_case in generated C++ code.
    // xbot_msgs/msg/MapSize.msg has mapWidth -> map_width, etc.
    map_size.map_width = map_.getSize().x() * map_.getResolution();
    map_size.map_height = map_.getSize().y() * map_.getResolution();
    auto mapPos = map_.getPosition();
    map_size.map_center_x = mapPos.x();
    map_size.map_center_y = mapPos.y();
    map_size_pub_->publish(map_size);

    std_msgs::msg::String json_map;
    json_map.data = map_data_.toJsonString();
    json_map_pub_->publish(json_map);
  }

  /**
   * Publish map visualizations for rviz.
   */
  void visualizeAreas() {
    auto mapPos = map_.getPosition();

    visualization_msgs::msg::MarkerArray marker_array;

    for (const auto& area : map_data_.areas) {
      if (!area.active) continue;
      if (area.type != "mow" && area.type != "obstacle") continue;

      std_msgs::msg::ColorRGBA color;
      if (area.type == "mow") {
        color.g = 1.0;
      } else if (area.type == "obstacle") {
        color.r = 1.0;
      }
      color.a = 1.0;

      grid_map::Polygon p = internalPolygonToGridMap(area.outline);
      visualization_msgs::msg::Marker marker;
      grid_map::PolygonRosConverter::toLineMarker(p, color, 0.05, 0, marker);

      marker.header.frame_id = "map";
      marker.ns = "mower_map_service";
      marker.id = marker_array.markers.size();
      marker.frame_locked = true;
      marker.pose.orientation.w = 1.0;

      marker_array.markers.push_back(marker);
    }

    // Visualize Docking Point
    if (!map_data_.docking_stations.empty()) {
      const DockingStation& ds = map_data_.docking_stations.front();
      geometry_msgs::msg::Pose docking_pose;
      docking_pose.position.x = ds.position.x;
      docking_pose.position.y = ds.position.y;
      docking_pose.position.z = 0.0;

      double heading = ds.heading;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, heading);
      docking_pose.orientation = tf2::toMsg(q);

      std_msgs::msg::ColorRGBA color;
      color.b = 1.0;
      color.a = 1.0;
      visualization_msgs::msg::Marker marker;

      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.2;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color = color;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.pose = docking_pose;
      RCLCPP_INFO_STREAM(this->get_logger(), "docking pose: " << docking_pose.position.x << ", " << docking_pose.position.y);
      marker.header.frame_id = "map";
      marker.ns = "mower_map_service";
      marker.id = marker_array.markers.size() + 1;
      marker.frame_locked = true;
      marker_array.markers.push_back(marker);
    }

    map_server_viz_array_pub_->publish(marker_array);
  }

  /**
   * Uses the polygons stored in MapData to build the final occupancy grid.
   *
   * First, the map is marked as completely occupied. Then navigation_areas and mowing_areas are marked as free.
   *
   * Then, all obstacles are marked as occupied.
   *
   * Finally, a blur is applied to the map so that it is expensive, but not completely forbidden to drive near boundaries.
   */
  void buildMap() {
    // First, calculate the size of the map by finding the min and max values for x and y.
    float minX = FLT_MAX;
    float maxX = -FLT_MAX;
    float minY = FLT_MAX;
    float maxY = -FLT_MAX;

    // loop through all areas and calculate a size where everything fits
    bool has_valid_area = false;
    for (const auto& area : map_data_.areas) {
      if (!area.active) continue;
      if (area.type != "mow" && area.type != "nav" && area.type != "obstacle") continue;
      for (const auto& point : area.outline) {
        minX = std::min(minX, (float)point.x);
        maxX = std::max(maxX, (float)point.x);
        minY = std::min(minY, (float)point.y);
        maxY = std::max(maxY, (float)point.y);
        has_valid_area = true;
      }
    }

    // Enlarge the map by 1m in all directions.
    // This guarantees that even after blurring, the map has an occupied border.
    maxX += 1.0;
    minX -= 1.0;
    maxY += 1.0;
    minY -= 1.0;

    // Check, if the map was empty. If so, we'd create a huge map. Therefore we build an empty 10x10m map instead.
    if (!has_valid_area) {
      maxX = 5.0;
      minX = -5.0;
      maxY = 5.0;
      minY = -5.0;
    }

    map_ = grid_map::GridMap({"navigation_area"});
    map_.setFrameId("map");
    grid_map::Position origin;
    origin.x() = (maxX + minX) / 2.0;
    origin.y() = (maxY + minY) / 2.0;

    RCLCPP_INFO(this->get_logger(), "Map Position: x=%f, y=%f", origin.x(), origin.y());
    RCLCPP_INFO(this->get_logger(), "Map Size: x=%f, y=%f", (maxX - minX), (maxY - minY));

    map_.setGeometry(grid_map::Length(maxX - minX, maxY - minY), 0.05, origin);
    map_.setTimestamp(this->get_clock()->now().nanoseconds());

    map_.clearAll();
    map_["navigation_area"].setConstant(1.0);

    grid_map::Matrix& data = map_["navigation_area"];
    for (const auto& area : map_data_.areas) {
      if (!area.active) continue;

      double value;
      if (area.type == "mow" || area.type == "nav") {
        value = 0.0;
      } else if (area.type == "obstacle") {
        value = 1.0;
      } else {
        continue;
      }

      grid_map::Polygon poly = internalPolygonToGridMap(area.outline);
      for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        data(index[0], index[1]) = value;
      }
    }

    if (show_fake_obstacle_) {
      grid_map::Polygon poly;
      tf2::Quaternion q;
      tf2::fromMsg(fake_obstacle_pose_.orientation, q);

      tf2::Matrix3x3 m(q);
      double unused1, unused2, yaw;

      m.getRPY(unused1, unused2, yaw);

      Eigen::Vector2d front(cos(yaw), sin(yaw));
      Eigen::Vector2d left(-sin(yaw), cos(yaw));
      Eigen::Vector2d obstacle_pos(fake_obstacle_pose_.position.x, fake_obstacle_pose_.position.y);

      {
        grid_map::Position pos = obstacle_pos + 0.1 * left + 0.25 * front;
        poly.addVertex(pos);
      }
      {
        grid_map::Position pos = obstacle_pos + 0.2 * left - 0.1 * front;
        poly.addVertex(pos);
      }
      {
        grid_map::Position pos = obstacle_pos + 0.6 * left - 0.1 * front;
        poly.addVertex(pos);
      }
      {
        grid_map::Position pos = obstacle_pos + 0.6 * left + 0.7 * front;
        poly.addVertex(pos);
      }

      {
        grid_map::Position pos = obstacle_pos - 0.6 * left + 0.7 * front;
        poly.addVertex(pos);
      }
      {
        grid_map::Position pos = obstacle_pos - 0.6 * left - 0.1 * front;
        poly.addVertex(pos);
      }
      {
        grid_map::Position pos = obstacle_pos - 0.2 * left - 0.1 * front;
        poly.addVertex(pos);
      }
      {
        grid_map::Position pos = obstacle_pos - 0.1 * left + 0.25 * front;
        poly.addVertex(pos);
      }
      for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        data(index[0], index[1]) = 1.0;
      }
    }

    cv::Mat cv_map;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map_, "navigation_area", CV_8UC1, cv_map);

    cv::blur(cv_map, cv_map, cv::Size(5, 5));

    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(cv_map, "navigation_area", map_);

    nav_msgs::msg::OccupancyGrid msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_, "navigation_area", 0.0, 1.0, msg);
    map_pub_->publish(msg);

    publishMapMonitoring();
    visualizeAreas();
  }

  /**
   * Saves the current map data to a JSON file.
   * We don't need to save the grid map, since we can easily build it again after loading.
   */
  void saveMapToFile() {
    std::ofstream file(MAP_FILE);
    if (file.is_open()) {
      file << map_data_.toJsonString();
      file.close();
      RCLCPP_INFO(this->get_logger(), "Map saved to JSON file");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file for writing");
    }
  }

  /**
   * Load the map from a JSON file and build a map.
   */
  void readMapFromFile() {
    std::ifstream json_file(MAP_FILE);
    if (json_file.is_open()) {
      try {
        json loaded_data;
        json_file >> loaded_data;
        json_file.close();

        map_data_ = loaded_data;

        RCLCPP_INFO(this->get_logger(), "Loaded %zu areas from: %s", map_data_.areas.size(), MAP_FILE.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse %s: %s", MAP_FILE.c_str(), e.what());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not open map file: %s", MAP_FILE.c_str());
    }
  }

  void addMowingArea(
    const std::shared_ptr<mower_map::srv::AddMowingAreaSrv::Request> req,
    std::shared_ptr<mower_map::srv::AddMowingAreaSrv::Response> /*res*/)
  {
    RCLCPP_INFO(this->get_logger(), "Got addMowingArea call");

    map_data_.areas.push_back(mowerMapAreaToInternal(req->area.area, req->is_navigation_area ? "nav" : "mow", req->area.name));
    for (const auto& obstacle : req->area.obstacles) {
      map_data_.areas.push_back(mowerMapAreaToInternal(obstacle, "obstacle", ""));
    }

    saveMapToFile();
    buildMap();
  }

  void getMowingArea(
    const std::shared_ptr<mower_map::srv::GetMowingAreaSrv::Request> req,
    std::shared_ptr<mower_map::srv::GetMowingAreaSrv::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Got getMowingArea call with index: %u", req->index);

    auto mowing_areas = map_data_.getMowingAreas();
    if (req->index >= mowing_areas.size()) {
      RCLCPP_ERROR(this->get_logger(), "No mowing area with index: %u", req->index);
      return;
    }

    res->area = internalMapAreaToMower(mowing_areas[req->index]);

    for (const auto& area : map_data_.areas) {
      if (!area.active || area.type != "obstacle") continue;
      res->area.obstacles.push_back(internalPolygonToGeometry(area.outline));
    }
  }

  void setDockingPoint(
    const std::shared_ptr<mower_map::srv::SetDockingPointSrv::Request> req,
    std::shared_ptr<mower_map::srv::SetDockingPointSrv::Response> /*res*/)
  {
    RCLCPP_INFO(this->get_logger(), "Setting Docking Point");

    // Convert quaternion to heading
    tf2::Quaternion q;
    tf2::fromMsg(req->docking_pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double unused1, unused2, heading;
    m.getRPY(unused1, unused2, heading);

    map_data_.docking_stations.clear();
    map_data_.docking_stations.push_back({.id = generateNanoId(),
                                         .name = "Docking Station",
                                         .active = true,
                                         .position = {req->docking_pose.position.x, req->docking_pose.position.y},
                                         .heading = heading});

    saveMapToFile();
    buildMap();
  }

  void getDockingPoint(
    const std::shared_ptr<mower_map::srv::GetDockingPointSrv::Request> /*req*/,
    std::shared_ptr<mower_map::srv::GetDockingPointSrv::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Getting Docking Point");

    if (map_data_.docking_stations.empty()) {
      return;
    }

    const DockingStation& ds = map_data_.docking_stations.front();
    res->docking_pose.position.x = ds.position.x;
    res->docking_pose.position.y = ds.position.y;
    res->docking_pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, ds.heading);
    res->docking_pose.orientation = tf2::toMsg(q);
  }

  void setNavPoint(
    const std::shared_ptr<mower_map::srv::SetNavPointSrv::Request> req,
    std::shared_ptr<mower_map::srv::SetNavPointSrv::Response> /*res*/)
  {
    RCLCPP_INFO(this->get_logger(), "Setting Nav Point");

    fake_obstacle_pose_ = req->nav_pose;

    show_fake_obstacle_ = true;

    buildMap();
  }

  void clearNavPoint(
    const std::shared_ptr<mower_map::srv::ClearNavPointSrv::Request> /*req*/,
    std::shared_ptr<mower_map::srv::ClearNavPointSrv::Response> /*res*/)
  {
    RCLCPP_INFO(this->get_logger(), "Clearing Nav Point");

    if (show_fake_obstacle_) {
      show_fake_obstacle_ = false;

      buildMap();
    }
  }

  void clearMap(
    const std::shared_ptr<mower_map::srv::ClearMapSrv::Request> /*req*/,
    std::shared_ptr<mower_map::srv::ClearMapSrv::Response> /*res*/)
  {
    RCLCPP_INFO(this->get_logger(), "Clearing Map");

    map_data_.clear();

    saveMapToFile();
    buildMap();
  }

  /**
   * Convert a legacy rosbag1 map file to JSON format.
   *
   * Note: rosbag2_cpp can only read rosbag2 format files natively.
   * If the legacy file is a ROS1 .bag file, it must first be converted to rosbag2
   * format using the `rosbags-convert` tool (from the rosbags Python package) or
   * the `ros2 bag convert` command.
   *
   * This method attempts to open the file as a rosbag2 database. If the file is
   * still in ROS1 format, it will log an error with conversion instructions.
   */
  void convertLegacyMapToJson() {
    rosbag2_cpp::Reader reader;
    try {
      // Try opening as rosbag2 (sqlite3 format)
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = LEGACY_MAP_FILE;
      storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";
      reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
        "Error opening legacy map file '%s': %s. "
        "If this is a ROS1 bag file, convert it first using: "
        "rosbags-convert %s",
        LEGACY_MAP_FILE.c_str(), e.what(), LEGACY_MAP_FILE.c_str());
      return;
    }

    // Clear the current map_data
    map_data_.clear();

    // Read all messages and sort by topic
    rclcpp::Serialization<mower_map::msg::MapArea> area_serializer;
    rclcpp::Serialization<geometry_msgs::msg::Pose> pose_serializer;

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      const std::string& topic = bag_msg->topic_name;

      if (topic == "mowing_areas" || topic == "navigation_areas") {
        std::string area_type = (topic == "mowing_areas") ? "mow" : "nav";

        mower_map::msg::MapArea area;
        rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
        area_serializer.deserialize_message(&serialized_msg, &area);

        // Convert main area
        MapArea main_area;
        main_area.id = generateNanoId();
        main_area.name = area.name;
        main_area.type = area_type;
        main_area.active = true;
        main_area.outline = geometryPolygonToInternal(area.area);
        map_data_.areas.push_back(main_area);

        // Convert obstacles as separate areas
        for (const auto& obstacle : area.obstacles) {
          MapArea obs_area;
          obs_area.id = generateNanoId();
          obs_area.name = "";
          obs_area.type = "obstacle";
          obs_area.active = true;
          obs_area.outline = geometryPolygonToInternal(obstacle);
          map_data_.areas.push_back(obs_area);
        }
      } else if (topic == "docking_point") {
        geometry_msgs::msg::Pose pt;
        rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
        pose_serializer.deserialize_message(&serialized_msg, &pt);

        // Convert quaternion to yaw
        tf2::Quaternion q;
        tf2::fromMsg(pt.orientation, q);
        tf2::Matrix3x3 mat(q);
        double unused1, unused2, yaw;
        mat.getRPY(unused1, unused2, yaw);

        // Create docking station
        DockingStation ds;
        ds.id = generateNanoId();
        ds.name = "Docking Station";
        ds.active = true;
        ds.position = {pt.position.x, pt.position.y};
        ds.heading = yaw;
        map_data_.docking_stations.push_back(ds);
      }
    }

    // Save the converted data to JSON file
    saveMapToFile();

    RCLCPP_INFO(this->get_logger(),
      "Successfully converted legacy map to JSON with %zu areas and %zu docking stations",
      map_data_.areas.size(), map_data_.docking_stations.size());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MowerMapServiceNode>();
  node->initRpc(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
