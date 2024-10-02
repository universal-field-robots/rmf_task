/*
 * Copyright 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * ...
 */

#ifndef RMF_TASK_SEQUENCE__EVENTS__WAITFORCONFIRMATION_HPP
#define RMF_TASK_SEQUENCE__EVENTS__WAITFORCONFIRMATION_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_task_sequence/Event.hpp> // Assuming WaitFor is part of Event hierarchy
#include <memory>
#include <optional>
#include <string>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
/// A WaitForConfirmation event allows the robot to wait for a confirmation
/// signal. The wait duration is continually extended until the confirmation
/// is received or a timeout occurs.
///
/// The Model of this event manages the logic for requesting confirmation,
/// handling received confirmations, extending wait durations, and enforcing
/// timeout constraints.
class WaitForConfirmation
{
public:

  // Forward declaration of nested classes
  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  class Model; // Defined in the .cpp file
};

//==============================================================================
/// Description class for WaitForConfirmation
///
/// Inherits from Event::Description to encapsulate the parameters and behaviors
/// specific to the WaitForConfirmation event.
class WaitForConfirmation::Description : public Event::Description
{
public:

  /// Type aliases for shared pointers
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;

  /// Factory method to create a Description with initial wait duration and timeout
  ///
  /// \param[in] initial_wait_duration
  ///   The initial duration to wait before re-requesting confirmation.
  /// \param[in] timeout_duration
  ///   The maximum duration to wait for confirmation before failing the task.
  static DescriptionPtr make(
    rmf_traffic::Duration initial_wait_duration,
    rmf_traffic::Duration timeout_duration);

  /// Constructor
  ///
  /// \param[in] initial_wait_duration_
  ///   The initial duration to wait.
  /// \param[in] timeout_duration_
  ///   The timeout duration after which the task fails if confirmation isn't received.
  Description(
    rmf_traffic::Duration initial_wait_duration_,
    rmf_traffic::Duration timeout_duration_);

  /// Get the initial wait duration
  ///
  /// \return The initial wait duration.
  rmf_traffic::Duration initial_wait_duration() const;

  /// Set the initial wait duration
  ///
  /// \param[in] new_initial_wait_duration
  ///   The new initial wait duration.
  /// \return Reference to the Description object.
  Description& initial_wait_duration(rmf_traffic::Duration new_initial_wait_duration);

  /// Get the timeout duration
  ///
  /// \return The timeout duration.
  rmf_traffic::Duration timeout_duration() const;

  /// Set the timeout duration
  ///
  /// \param[in] new_timeout_duration
  ///   The new timeout duration.
  /// \return Reference to the Description object.
  Description& timeout_duration(rmf_traffic::Duration new_timeout_duration);

  /// Override: Create the corresponding Model for this Description
  ///
  /// \param[in] invariant_initial_state
  ///   The initial state before the event begins.
  /// \param[in] parameters
  ///   Task parameters including battery constraints, etc.
  /// \return A shared pointer to the created Model.
  Activity::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const final;

  /// Override: Generate a header for this event
  ///
  /// \param[in] state
  ///   The current state of the task.
  /// \param[in] parameters
  ///   Task parameters.
  /// \return A Header object describing the event.
  Header generate_header(
    const State& state,
    const Parameters& parameters) const final;

private:
  /// Private implementation class for encapsulation
  class Implementation;

  /// Pointer to the implementation (PImpl idiom)
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENTS__WAITFORCONFIRMATION_HPP
