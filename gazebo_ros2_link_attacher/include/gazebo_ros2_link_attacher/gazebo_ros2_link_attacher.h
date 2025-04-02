#ifndef GAZEBO_ROS2_LINK_ATTACHER_HPP
#define GAZEBO_ROS2_LINK_ATTACHER_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include "gazebo_ros2_link_attacher/srv/attach.hpp"

namespace gazebo
{
class GazeboRosLinkAttacher : public WorldPlugin
{
public:
  GazeboRosLinkAttacher();
  ~GazeboRosLinkAttacher();
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override; // must include when inheriting from WorldPlugin

private:
  struct FixedJoint
  {
    std::string model1;
    physics::ModelPtr m1;
    std::string link1;
    physics::LinkPtr l1;
    std::string model2;
    physics::ModelPtr m2;
    std::string link2;
    physics::LinkPtr l2;
    physics::JointPtr joint;
  };

  bool getJoint(const std::string& model1, const std::string& link1,
                const std::string& model2, const std::string& link2,
                FixedJoint &joint);

  bool attach(const std::string& model1, const std::string& link1,
              const std::string& model2, const std::string& link2);

  bool detach(const std::string& model1, const std::string& link1,
              const std::string& model2, const std::string& link2);

  void attachCallback(const std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Request> req,
                      std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Response> res);

  void detachCallback(const std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Request> req,
                      std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Response> res);

  std::vector<FixedJoint> joints_;
  physics::WorldPtr world_;
  physics::PhysicsEnginePtr physics_;
  std::shared_ptr<gazebo_ros::Node> ros_node_;
  rclcpp::Service<gazebo_ros2_link_attacher::srv::Attach>::SharedPtr attach_service_;
  rclcpp::Service<gazebo_ros2_link_attacher::srv::Attach>::SharedPtr detach_service_;
};
}
#endif

