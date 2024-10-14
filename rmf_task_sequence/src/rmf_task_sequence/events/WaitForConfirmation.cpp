/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <rmf_task_sequence/events/WaitForConfirmation.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class WaitForConfirmation::Model : public Activity::Model
{
public:

  Model(
    rmf_task::State invariant_initial_state,
    rmf_traffic::Duration initial_wait_duration,
    rmf_traffic::Duration timeout_duration,
    const rmf_task::Parameters& parameters);

  std::optional<Estimate> estimate_finish(
    rmf_task::State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const Constraints& constraints,
    const TravelEstimator& travel_estimator) const final;

  rmf_traffic::Duration invariant_duration() const final;

  State invariant_finish_state() const final;

private:
  rmf_task::State _invariant_finish_state;
  double _invariant_battery_drain;
  rmf_traffic::Duration _initial_wait_duration;
  rmf_traffic::Duration _timeout_duration;
};

//==============================================================================
class WaitForConfirmation::Description::Implementation
{
public:

  rmf_traffic::Duration initial_wait_duration;
  rmf_traffic::Duration timeout_duration;

};

//==============================================================================
auto WaitForConfirmation::Description::make(rmf_traffic::Duration initial_wait_duration, rmf_traffic::Duration timeout_duration)
-> DescriptionPtr
{
  return std::make_shared<Description>(initial_wait_duration, timeout_duration);
}

//==============================================================================
WaitForConfirmation::Description::Description(rmf_traffic::Duration initial_wait_duration_, rmf_traffic::Duration timeout_duration_)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{initial_wait_duration_, timeout_duration_}))
{
  // Do nothing
}

//==============================================================================
rmf_traffic::Duration WaitForConfirmation::Description::initial_wait_duration() const
{
  return _pimpl->initial_wait_duration;
}

//==============================================================================
rmf_traffic::Duration WaitForConfirmation::Description::timeout_duration() const
{
  return _pimpl->timeout_duration;
}

//==============================================================================
auto WaitForConfirmation::Description::initial_wait_duration(rmf_traffic::Duration new_initial_wait_duration)
-> Description&
{
  _pimpl->initial_wait_duration = new_initial_wait_duration;
  return *this;
}

//==============================================================================
auto WaitForConfirmation::Description::timeout_duration(rmf_traffic::Duration new_timeout_duration)
-> Description&
{
  _pimpl->timeout_duration = new_timeout_duration;
  return *this;
}

//==============================================================================
Activity::ConstModelPtr WaitForConfirmation::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  return std::make_shared<Model>(
    invariant_initial_state, _pimpl->initial_wait_duration, _pimpl->timeout_duration, parameters);
}

//==============================================================================
Header WaitForConfirmation::Description::generate_header(
  const State&, const Parameters&) const
{

  return Header(
    "Waiting",
    "Waiting for confirmation",
    _pimpl->initial_wait_duration);
}

//==============================================================================
WaitForConfirmation::Model::Model(
  State invariant_initial_state,
  rmf_traffic::Duration initial_wait_duration,
  rmf_traffic::Duration timeout_duration,
  const Parameters& parameters)
: _invariant_finish_state(std::move(invariant_initial_state)),
  _initial_wait_duration(initial_wait_duration),
  _timeout_duration(timeout_duration)
{
  if (parameters.ambient_sink())
  {
    // Handle cases where _initial_wait_duration is invalid.
    const auto initial_wait_duration =
      _initial_wait_duration.count() < 0 ? rmf_traffic::Duration(0) : _initial_wait_duration;

    _invariant_battery_drain =
      parameters.ambient_sink()->compute_change_in_charge(
      rmf_traffic::time::to_seconds(initial_wait_duration));
  }
  else
  {
    _invariant_battery_drain = 0.0;
  }
}

//==============================================================================
std::optional<Estimate> WaitForConfirmation::Model::estimate_finish(
  State state,
  rmf_traffic::Time earliest_arrival_time,
  const Constraints& constraints,
  const TravelEstimator&) const
{
  state.time(state.time().value() + _initial_wait_duration);

  if (constraints.drain_battery())
  {
    const auto new_battery_soc =
      state.battery_soc().value() - _invariant_battery_drain;
    if (new_battery_soc < 0.0)
    {
      return std::nullopt;
    }
    state.battery_soc(new_battery_soc);
  }

  if (state.battery_soc().value() <= constraints.threshold_soc())
    return std::nullopt;

  return Estimate(state, earliest_arrival_time);
}

//==============================================================================
rmf_traffic::Duration WaitForConfirmation::Model::invariant_duration() const
{
  return _initial_wait_duration;
}

//==============================================================================
State WaitForConfirmation::Model::invariant_finish_state() const
{
  return _invariant_finish_state;
}

} // namespace phases
} // namespace rmf_task_sequence
