/*
 * Copyright 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * ...
 */

#include <rmf_task_sequence/events/WaitForConfirmation.hpp>
#include <rmf_task/Request.hpp>
#include <rclcpp/rclcpp.hpp> // ROS2 for communication
#include <std_msgs/msg/string.hpp>
#include <uuid/uuid.h> // For generating UUIDs
#include <chrono>
#include <sstream>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class WaitForConfirmation::Model : public Activity::Model
{
public:

  Model(
    rmf_task::State invariant_initial_state,
    rmf_traffic::Duration initial_wait_duration, // Start with an initial duration
    rmf_traffic::Duration timeout_duration, // Added timeout duration
    const rmf_task::Parameters& parameters,
    std::shared_ptr<rclcpp::Node> node); // ROS2 Node for communication

  std::optional<Estimate> estimate_finish(
    rmf_task::State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const Constraints& constraints,
    const TravelEstimator& travel_estimator) const final;

  rmf_traffic::Duration invariant_duration() const final;

  State invariant_finish_state() const final;

private:
  // Confirmation handling
  void request_confirmation() const;
  void on_confirmation_received(const std::shared_ptr<std_msgs::msg::String> msg);

  // UUID generation
  std::string generate_uuid() const;

  rmf_task::State _invariant_finish_state;
  double _invariant_battery_drain;
  rmf_traffic::Duration _initial_wait_duration;
  rmf_traffic::Duration _timeout_duration;

  mutable bool _confirmation_received; // Track confirmation status
  mutable rmf_traffic::Time _confirmation_request_time; // Time when confirmation was requested
  const std::string _task_uuid; // UUID for the current task instance

  // ROS2 Components
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _confirmation_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _confirmation_pub;
};

//==============================================================================

WaitForConfirmation::Model::Model(
  rmf_task::State invariant_initial_state,
  rmf_traffic::Duration initial_wait_duration,
  rmf_traffic::Duration timeout_duration,
  const rmf_task::Parameters& parameters,
  std::shared_ptr<rclcpp::Node> node)
: _invariant_finish_state(std::move(invariant_initial_state)),
  _initial_wait_duration(initial_wait_duration),
  _timeout_duration(timeout_duration),
  _confirmation_received(false),
  _node(node),
  _task_uuid(generate_uuid()) // Generate UUID once per task instance
{
  if (parameters.ambient_sink())
  {
    // Handle cases where duration is invalid.
    const auto duration =
      _initial_wait_duration.count() < 0 ? rmf_traffic::Duration(0) : _initial_wait_duration;

    _invariant_battery_drain =
      parameters.ambient_sink()->compute_change_in_charge(
      rmf_traffic::time::to_seconds(duration));
  }
  else
  {
    _invariant_battery_drain = 0.0;
  }

  // Initialize ROS2 Publisher and Subscriber
  _confirmation_pub = _node->create_publisher<std_msgs::msg::String>(
    "/request_confirmation", 10);

  _confirmation_sub = _node->create_subscription<std_msgs::msg::String>(
    "/confirmation_received",
    10,
    std::bind(&WaitForConfirmation::Model::on_confirmation_received, this, std::placeholders::_1));

  // Request initial confirmation
  request_confirmation();
}

//==============================================================================

std::string WaitForConfirmation::Model::generate_uuid() const
{
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[37];
  uuid_unparse_lower(uuid, uuid_str);
  return std::string(uuid_str);
}

//==============================================================================

void WaitForConfirmation::Model::request_confirmation() const
{
  std_msgs::msg::String msg;
  msg.data = _task_uuid; // Send the same UUID for this task instance
  _confirmation_pub->publish(msg);
  RCLCPP_INFO(_node->get_logger(), "Confirmation requested with UUID: %s", _task_uuid.c_str());

  // Record the time when the confirmation was requested
  _confirmation_request_time = std::chrono::steady_clock::now();
}

//==============================================================================

void WaitForConfirmation::Model::on_confirmation_received(const std::shared_ptr<std_msgs::msg::String> msg)
{
  if (msg->data == _task_uuid)
  {
    _confirmation_received = true;
    RCLCPP_INFO(_node->get_logger(), "Confirmation received for UUID: %s", msg->data.c_str());
  }
  else
  {
    RCLCPP_WARN(_node->get_logger(), "Received confirmation with unmatched UUID: %s", msg->data.c_str());
  }
}

//==============================================================================

std::optional<rmf_task::Estimate> WaitForConfirmation::Model::estimate_finish(
  rmf_task::State state,
  rmf_traffic::Time earliest_arrival_time,
  const Constraints& constraints,
  const TravelEstimator&) const
{
  // Check if timeout has been reached
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed_time = current_time - _confirmation_request_time;

  if (!_confirmation_received)
  {
    if (elapsed_time > _timeout_duration)
    {
      RCLCPP_ERROR(_node->get_logger(), "Confirmation timeout reached. Failing the task.");
      return std::nullopt; // Fail the task due to timeout
    }

    // Extend wait duration
    state.time(state.time().value() + _initial_wait_duration);

    // Re-request confirmation periodically, e.g., every initial_wait_duration
    request_confirmation();

    // Update battery drain
    if (constraints.drain_battery())
    {
      const double additional_drain =
        _invariant_battery_drain; // Assuming drain per extension
      const double new_battery_soc =
        state.battery_soc().value() - additional_drain;

      if (new_battery_soc < 0.0)
      {
        RCLCPP_ERROR(_node->get_logger(), "Battery depleted while waiting for confirmation.");
        return std::nullopt;
      }

      state.battery_soc(new_battery_soc);
    }

    if (state.battery_soc().value() <= constraints.threshold_soc())
    {
      RCLCPP_ERROR(_node->get_logger(), "Battery SOC below threshold while waiting for confirmation.");
      return std::nullopt;
    }

    return Estimate(state, earliest_arrival_time);
  }

  // Confirmation received, finalize the task without adding additional duration
  RCLCPP_INFO(_node->get_logger(), "Confirmation received. Finalizing the task.");
  // No additional duration is added

  if (constraints.drain_battery())
  {
    const double new_battery_soc =
      state.battery_soc().value() - _invariant_battery_drain;
    if (new_battery_soc < 0.0)
    {
      RCLCPP_ERROR(_node->get_logger(), "Battery depleted upon confirmation.");
      return std::nullopt;
    }
    state.battery_soc(new_battery_soc);
  }

  if (state.battery_soc().value() <= constraints.threshold_soc())
  {
    RCLCPP_ERROR(_node->get_logger(), "Battery SOC below threshold upon confirmation.");
    return std::nullopt;
  }

  return Estimate(state, state.time().value());
}

//==============================================================================

rmf_traffic::Duration WaitForConfirmation::Model::invariant_duration() const
{
  if (_confirmation_received)
    return rmf_traffic::Duration(0); // No duration if confirmed
  else
    return _initial_wait_duration; // Current wait duration
}

//==============================================================================

State WaitForConfirmation::Model::invariant_finish_state() const
{
  return _invariant_finish_state;
}

//==============================================================================
// WaitForConfirmation::Description::Implementation

class WaitForConfirmation::Description::Implementation
{
public:

  rmf_traffic::Duration initial_wait_duration;
  rmf_traffic::Duration timeout_duration;
};

//==============================================================================
// WaitForConfirmation::Description Methods

auto WaitForConfirmation::Description::make(
  rmf_traffic::Duration initial_wait_duration,
  rmf_traffic::Duration timeout_duration)
-> DescriptionPtr
{
  return std::make_shared<Description>(initial_wait_duration, timeout_duration);
}

WaitForConfirmation::Description::Description(
  rmf_traffic::Duration initial_wait_duration_,
  rmf_traffic::Duration timeout_duration_)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation{initial_wait_duration_, timeout_duration_}))
{
  // Do nothing
}

rmf_traffic::Duration WaitForConfirmation::Description::initial_wait_duration() const
{
  return _pimpl->initial_wait_duration;
}

rmf_traffic::Duration WaitForConfirmation::Description::timeout_duration() const
{
  return _pimpl->timeout_duration;
}

auto WaitForConfirmation::Description::initial_wait_duration(
  rmf_traffic::Duration new_initial_wait_duration)
-> Description&
{
  _pimpl->initial_wait_duration = new_initial_wait_duration;
  return *this;
}

auto WaitForConfirmation::Description::timeout_duration(
  rmf_traffic::Duration new_timeout_duration)
-> Description&
{
  _pimpl->timeout_duration = new_timeout_duration;
  return *this;
}

Activity::ConstModelPtr WaitForConfirmation::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  // Assume that a ROS2 node is available; otherwise, pass it appropriately
  auto node = std::make_shared<rclcpp::Node>("wait_for_confirmation_node");

  return std::make_shared<Model>(
    invariant_initial_state,
    _pimpl->initial_wait_duration,
    _pimpl->timeout_duration,
    parameters,
    node);
}

Header WaitForConfirmation::Description::generate_header(
  const State&,
  const Parameters& parameters) const
{
  return Header(
    "Waiting for Confirmation",
    "Waiting until confirmation is received or timeout occurs",
    _pimpl->initial_wait_duration);
}

//==============================================================================
// Factory Methods for WaitForConfirmation

// ConstRequestPtr WaitForConfirmation::make(
//   rmf_traffic::Duration initial_wait_duration,
//   rmf_traffic::Duration timeout_duration,
//   const std::string& id,
//   rmf_traffic::Time earliest_start_time,
//   ConstPriorityPtr priority,
//   bool automatic)
// {
//   auto description = WaitForConfirmation::Description::make(
//     initial_wait_duration,
//     timeout_duration);

//   return std::make_shared<Request>(
//     std::make_shared<rmf_task::Task::Booking>(
//       id,
//       earliest_start_time,
//       std::move(priority),
//       automatic),
//     std::move(description));
// }

// ConstRequestPtr WaitForConfirmation::make(
//   rmf_traffic::Duration initial_wait_duration,
//   rmf_traffic::Duration timeout_duration,
//   const std::string& id,
//   rmf_traffic::Time earliest_start_time,
//   const std::string& requester,
//   rmf_traffic::Time request_time,
//   ConstPriorityPtr priority,
//   bool automatic)
// {
//   auto description = WaitForConfirmation::Description::make(
//     initial_wait_duration,
//     timeout_duration);

//   return std::make_shared<Request>(
//     std::make_shared<rmf_task::Task::Booking>(
//       id,
//       earliest_start_time,
//       std::move(priority),
//       requester,
//       request_time,
//       automatic),
//     std::move(description));
// }

} // namespace events
} // namespace rmf_task_sequence
