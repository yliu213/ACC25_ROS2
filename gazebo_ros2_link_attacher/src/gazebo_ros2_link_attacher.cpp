#include "gazebo_ros2_link_attacher/gazebo_ros2_link_attacher.h"
#include <gazebo/physics/World.hh>
#include <boost/make_shared.hpp>

GZ_REGISTER_WORLD_PLUGIN(gazebo::GazeboRosLinkAttacher)

namespace gazebo
{
GazeboRosLinkAttacher::GazeboRosLinkAttacher() {}
GazeboRosLinkAttacher::~GazeboRosLinkAttacher() {}

void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  physics_ = world_->Physics();
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  attach_service_ = ros_node_->create_service<gazebo_ros2_link_attacher::srv::Attach>(
    "~/attach",
    std::bind(&GazeboRosLinkAttacher::attachCallback, this, std::placeholders::_1, std::placeholders::_2));

  detach_service_ = ros_node_->create_service<gazebo_ros2_link_attacher::srv::Attach>(
    "~/detach",
    std::bind(&GazeboRosLinkAttacher::detachCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(ros_node_->get_logger(), "gazebo_ros2_link_attacher loaded and ready.");
}

bool GazeboRosLinkAttacher::getJoint(const std::string &model1, const std::string &link1,
                                     const std::string &model2, const std::string &link2,
                                     FixedJoint &joint)
{
  for (auto &j : joints_)
  {
    if (j.model1 == model1 && j.link1 == link1 &&
        j.model2 == model2 && j.link2 == link2)
    {
      joint = j;
      return true;
    }
  }
  return false;
}

bool GazeboRosLinkAttacher::attach(const std::string &model1, const std::string &link1,
                                   const std::string &model2, const std::string &link2)
{
  FixedJoint joint;
  if (getJoint(model1, link1, model2, link2, joint))
  {
    joint.joint->Attach(joint.l1, joint.l2);
    return true;
  }

  joint.model1 = model1;
  joint.model2 = model2;
  joint.link1 = link1;
  joint.link2 = link2;

  auto m1 = world_->ModelByName(model1);
  auto m2 = world_->ModelByName(model2);
  if (!m1 || !m2) return false;

  joint.m1 = boost::dynamic_pointer_cast<physics::Model>(m1);
  joint.m2 = boost::dynamic_pointer_cast<physics::Model>(m2);
  joint.l1 = joint.m1->GetLink(link1);
  joint.l2 = joint.m2->GetLink(link2);

  if (!joint.l1 || !joint.l2) return false;

  joint.joint = physics_->CreateJoint("revolute", joint.m1);
  joint.joint->Attach(joint.l1, joint.l2);
  joint.joint->Load(joint.l1, joint.l2, ignition::math::Pose3d());
  joint.joint->SetUpperLimit(0, 0);
  joint.joint->SetLowerLimit(0, 0);
  joint.joint->Init();

  joints_.push_back(joint);
  return true;
}

bool GazeboRosLinkAttacher::detach(const std::string &model1, const std::string &link1,
                                   const std::string &model2, const std::string &link2)
{
  FixedJoint joint;
  if (getJoint(model1, link1, model2, link2, joint))
  {
    joint.joint->Detach();
    return true;
  }
  return false;
}

void GazeboRosLinkAttacher::attachCallback(const std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Request> req,
                                           std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Response> res)
{
  res->ok = attach(req->model_name_1, req->link_name_1, req->model_name_2, req->link_name_2);
}

void GazeboRosLinkAttacher::detachCallback(const std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Request> req,
                                           std::shared_ptr<gazebo_ros2_link_attacher::srv::Attach::Response> res)
{
  res->ok = detach(req->model_name_1, req->link_name_1, req->model_name_2, req->link_name_2);
}
}
