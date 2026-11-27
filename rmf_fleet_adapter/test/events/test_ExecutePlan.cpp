/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <events/ExecutePlan.hpp>

#include <agv/internal_FleetUpdateHandle.hpp>
#include <agv/internal_RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include "../mock/MockRobotCommand.hpp"

#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace events {
namespace test {

namespace {

const std::string lift_name = "test_lift";
const std::string departure_map = "L1";
const std::string destination_map = "L2";

struct EventInfo
{
  std::string name;
  std::string detail;
};

// A lift specific counterpart of phases/MockAdapterFixture: that fixture's
// graph has a single map and no lift, so it cannot reach the lift insertion
// in ExecutePlan::make.
//
// A minimal two level graph where a single lift joins the two levels:
//
//   L1:  0 (outside) ---- 1 (inside the lift)
//                         |  LiftMove (upward only)
//   L2:  3 (outside) ---- 2 (inside the lift)
//
struct LiftFixture
{
  std::shared_ptr<rclcpp::Context> rcl_context;
  std::shared_ptr<agv::test::MockAdapter> adapter;
  std::shared_ptr<agv::FleetUpdateHandle> fleet;
  std::shared_ptr<agv::Node> node;
  rmf_traffic::agv::Graph graph;

  std::shared_ptr<agv::RobotContext> context;
  std::shared_ptr<rmf_fleet_adapter_test::MockRobotCommand> command;

  static std::size_t node_counter;

  LiftFixture()
  {
    rcl_context = std::make_shared<rclcpp::Context>();
    rcl_context->init(0, nullptr);

    adapter = std::make_shared<agv::test::MockAdapter>(
      "test_execute_plan_" + std::to_string(node_counter++),
      rclcpp::NodeOptions().context(rcl_context));

    graph.set_known_lift(
      rmf_traffic::agv::Graph::LiftProperties(
        lift_name, {0.0, 0.0}, 0.0, {2.0, 2.0}));
    const auto lift = graph.find_known_lift(lift_name);

    graph.add_waypoint(departure_map, {5.0, 0.0}).set_charger(true); // 0
    graph.add_waypoint(departure_map, {0.0, 0.0}).set_in_lift(lift); // 1
    graph.add_waypoint(destination_map, {0.0, 0.0}).set_in_lift(lift); // 2
    graph.add_waypoint(destination_map, {5.0, 0.0}); // 3

    // The lanes are shaped the way rmf_fleet_adapter::agv::parse_graph builds
    // them for a lift: entering the car begins a session, riding the car moves
    // it (ExecutePlan::make then skips ahead to the next event, which is the
    // door opening on the lane that leaves the car), and leaving the car opens
    // the doors on its entry and ends the session on its exit. Only the upward
    // ride is modelled, because a downward lane would give the planner a zero
    // length cycle between the two coincident car waypoints.
    using Lane = rmf_traffic::agv::Graph::Lane;
    using Event = Lane::Event;
    const auto d = std::chrono::seconds(4);
    const auto move_d = std::chrono::seconds(1);
    const auto end_d = std::chrono::seconds(0);
    const auto no_event = rmf_utils::clone_ptr<Event>();

    graph.add_lane(
      {0, Event::make(Lane::LiftSessionBegin(lift_name, departure_map, d))},
      {1, no_event});
    graph.add_lane(
      {1, Event::make(Lane::LiftDoorOpen(lift_name, departure_map, d))},
      {0, Event::make(Lane::LiftSessionEnd(lift_name, departure_map, end_d))});
    graph.add_lane(
      {3, Event::make(Lane::LiftSessionBegin(lift_name, destination_map, d))},
      {2, no_event});
    graph.add_lane(
      {2, Event::make(Lane::LiftDoorOpen(lift_name, destination_map, d))},
      {3, Event::make(
          Lane::LiftSessionEnd(lift_name, destination_map, end_d))});
    graph.add_lane(
      {1, Event::make(Lane::LiftMove(lift_name, destination_map, move_d))},
      {2, no_event});

    const rmf_traffic::Profile profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(0.1)
    };

    const rmf_traffic::agv::VehicleTraits traits{
      {0.7, 0.3}, {1.0, 0.45}, profile};

    fleet = adapter->add_fleet("test_fleet", traits, graph);
    node = agv::FleetUpdateHandle::Implementation::get(*fleet).node;
    adapter->start();

    const auto now = rmf_traffic_ros2::convert(adapter->node()->now());
    command = std::make_shared<rmf_fleet_adapter_test::MockRobotCommand>(
      node, graph);

    std::promise<void> robot_added;
    fleet->add_robot(
      command, "test_robot", profile,
      rmf_traffic::agv::Plan::StartSet{{now, 0, 0.0}},
      [this, &robot_added](agv::RobotUpdateHandlePtr updater)
      {
        context = agv::RobotUpdateHandle::Implementation::get(*updater)
        .context.lock();
        command->updater = updater;
        robot_added.set_value();
      });

    robot_added.get_future().wait();
  }

  ~LiftFixture()
  {
    std::weak_ptr<rclcpp::Node> weak_node = node;
    node->stop();
    context.reset();
    command.reset();
    fleet.reset();
    node.reset();
    adapter.reset();

    std::size_t wait_count = 0;
    while (const auto n = weak_node.lock())
    {
      ++wait_count;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (wait_count > 200)
      {
        std::cerr << "Node is not dying during test teardown" << std::endl;
        std::terminate();
      }
    }
  }

  // Plans from `start` to `goal` and returns the events that
  // ExecutePlan::make builds for that plan, in order.
  std::vector<EventInfo> events(
    const rmf_traffic::agv::Plan::Start& start,
    std::size_t goal) const;

  template<typename T>
  T schedule_and_wait(std::function<T()> job) const
  {
    std::promise<T> promise;
    auto future = promise.get_future();
    context->worker().schedule(
      [&promise, job = std::move(job)](const auto&)
      {
        promise.set_value(job());
      });

    return future.get();
  }
};

std::size_t LiftFixture::node_counter = 0;

//==============================================================================
// Depth first, in execution order: each event's name and detail.
void collect_events(
  const rmf_task::Event::ConstStatePtr& state,
  std::vector<EventInfo>& events)
{
  rmf_task::VersionedString::Reader reader;
  events.push_back(
    {*reader.read(state->name()), *reader.read(state->detail())});
  for (const auto& dep : state->dependencies())
  {
    collect_events(dep, events);
  }
}

//==============================================================================
void check_events(
  const std::vector<EventInfo>& actual,
  const std::vector<EventInfo>& expected)
{
  REQUIRE(actual.size() == expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    CAPTURE(i);
    CHECK(actual[i].name == expected[i].name);
    CHECK(actual[i].detail == expected[i].detail);
  }
}

//==============================================================================
std::vector<EventInfo> LiftFixture::events(
  const rmf_traffic::agv::Plan::Start& start,
  std::size_t goal) const
{
  const auto planner = context->planner();
  REQUIRE(planner);

  const auto plan = planner->plan(start, rmf_traffic::agv::Plan::Goal(goal));
  REQUIRE(plan.success());
  REQUIRE_FALSE(plan->get_waypoints().front().graph_index().has_value());

  return schedule_and_wait<std::vector<EventInfo>>(
    [&]()
    {
      auto state = rmf_task::events::SimpleEventState::make(
        0, "test", "", rmf_task::Event::Status::Standby, {},
        context->clock());

      const auto execution = ExecutePlan::make(
        context, context->itinerary().assign_plan_id(), *plan,
        rmf_traffic::agv::Plan::Goal(goal), plan->get_itinerary(),
        rmf_task::Event::AssignID::make(), state, []() {}, []() {},
        std::nullopt);

      std::vector<EventInfo> result;
      if (execution.has_value())
      {
        collect_events(execution->sequence->state(), result);
      }

      return result;
    });
}

} // anonymous namespace

//==============================================================================
SCENARIO_METHOD(LiftFixture, "execute plan lift summon after replan",
  "[events]")
{
  GIVEN("the robot holds a lift session requested from inside")
  {
    const auto session = schedule_and_wait<std::shared_ptr<void>>(
      [&]()
      {
        return context->set_lift_destination(
          lift_name, destination_map, true);
      });
    REQUIRE(session);

    // The replanned start is inside the car, off the graph, and still on the
    // map of the floor the robot boarded from.
    const auto now = rmf_traffic_ros2::convert(adapter->node()->now());
    const rmf_traffic::agv::Plan::Start start(
      now, 1, 0.0, Eigen::Vector2d(0.3, 0.0));

    WHEN("the new plan continues to the floor the lift is riding to")
    {
      // The first summon is the one ExecutePlan::make re-inserts after the
      // replan. It must follow the session (L2), not the reported map (L1).
      // The second is the LiftDoorOpen on the lane that leaves the car.
      THEN("every summon targets the destination floor")
      {
        check_events(
          events(start, 3),
          {
            {"test", ""},
            {"Take [lift:test_lift] to [floor:L2]", ""},
            {"Requesting lift [test_lift] to [L2]", ""},
            {"Move to [graph-wp:1] <0 0 0> through 3 points", ""},
            {"Requesting lift [test_lift] to [L2]", ""},
            {"Move to [graph-wp:3] <5 0 0> through 3 points", ""},
            {"End session with lift [test_lift]", ""},
            {"Move to [graph-wp:3] <5 0 0> through 2 points", ""}
          });
      }
    }

    WHEN("the new plan returns to the floor the robot boarded from")
    {
      // The re-inserted summon still follows the session (L2). Only the
      // LiftDoorOpen on the lane that leaves the car asks for the floor the
      // robot is returning to (L1).
      THEN("the inserted summon follows the session, not the reported map")
      {
        check_events(
          events(start, 0),
          {
            {"test", ""},
            {"Take [lift:test_lift] to [floor:L1]", ""},
            {"Requesting lift [test_lift] to [L2]", ""},
            {"Move to [graph-wp:1] <0 0 0> through 3 points", ""},
            {"Requesting lift [test_lift] to [L1]", ""},
            {"Move to [graph-wp:0] <5 0 0> through 3 points", ""},
            {"End session with lift [test_lift]", ""},
            {"Move to [graph-wp:0] <5 0 0> through 2 points", ""}
          });
      }
    }
  }
}

//==============================================================================
SCENARIO_METHOD(LiftFixture, "position estimate while riding a lift",
  "[events]")
{
  agv::NavParams nav_params{false, nullptr, {}};
  nav_params.find_stacked_vertices(graph);

  const auto starts = [&](const std::string& map, Eigen::Vector2d p)
  {
    return schedule_and_wait<rmf_traffic::agv::Plan::StartSet>(
      [&]()
      {
        return nav_params.compute_plan_starts(
          *context, map, {p.x(), p.y(), 0.0}, context->now());
      });
  };

  const auto describe = [](const rmf_traffic::agv::Plan::StartSet& starts)
  {
    std::stringstream ss;
    for (const auto& s : starts)
    {
      ss << "[wp " << s.waypoint() << " lane "
         << (s.lane().has_value() ? std::to_string(*s.lane()) : "-")
         << " loc " << s.location().has_value() << "]";
    }
    return ss.str();
  };

  const auto require_waypoint = [describe](
    const rmf_traffic::agv::Plan::StartSet& starts,
    std::size_t wp,
    std::optional<Eigen::Vector2d> p)
  {
    CAPTURE(describe(starts));
    REQUIRE(starts.size() == 1);
    CHECK(starts.front().waypoint() == wp);
    CHECK_FALSE(starts.front().lane().has_value());
    REQUIRE(starts.front().location().has_value() == p.has_value());
    if (p.has_value())
    {
      CHECK((*starts.front().location() - *p).norm() < 1e-8);
    }
  };

  // On the car vertex, within the merge radius, and inside the car but away
  // from the car vertex.
  const Eigen::Vector2d on_car(0.0, 0.0);
  const Eigen::Vector2d in_car(0.3, 0.0);

  GIVEN("the robot holds a lift session requested from inside")
  {
    const auto session = schedule_and_wait<std::shared_ptr<void>>(
      [&]()
      {
        return context->set_lift_destination(
          lift_name, destination_map, true);
      });
    REQUIRE(session);

    WHEN("it still reports the boarding floor from inside the car")
    {
      THEN("it is on the car vertex of the destination floor")
      {
        require_waypoint(starts(departure_map, on_car), 2, std::nullopt);
        require_waypoint(starts(departure_map, in_car), 2, in_car);
      }
    }

    WHEN("the lift request phase has also set the boarded level")
    {
      const auto level = std::make_shared<const std::string>(destination_map);
      schedule_and_wait<bool>(
        [&]()
        {
          context->_set_current_boarded_lift_level(level);
          return true;
        });

      THEN("it is on the car vertex of the destination floor")
      {
        require_waypoint(starts(departure_map, on_car), 2, std::nullopt);
        require_waypoint(starts(departure_map, in_car), 2, in_car);
      }
    }

    WHEN("it has localized onto the destination floor")
    {
      THEN("the start set is the same as without a session")
      {
        CHECK(
          describe(starts(destination_map, in_car)) ==
          "[wp 2 lane 2 loc 1][wp 3 lane 3 loc 1]");
      }
    }

    WHEN("it reports a position outside the car")
    {
      THEN("the start set is the same as without a session")
      {
        CHECK(
          describe(starts(departure_map, {5.0, 0.0})) ==
          "[wp 0 lane - loc 0]");
      }
    }
  }

  GIVEN("the robot holds a lift session requested from outside")
  {
    const auto session = schedule_and_wait<std::shared_ptr<void>>(
      [&]()
      {
        return context->set_lift_destination(
          lift_name, destination_map, false);
      });
    REQUIRE(session);

    THEN("the start set does not change")
    {
      // These are the start sets that the code gave before the session was
      // taken into account.
      CHECK(describe(starts(departure_map, on_car)) == "[wp 1 lane - loc 0]");
      CHECK(
        describe(starts(departure_map, in_car)) ==
        "[wp 1 lane 0 loc 1][wp 0 lane 1 loc 1]");
    }
  }

  GIVEN("the robot holds no lift session")
  {
    THEN("the start set does not change")
    {
      // These are the start sets that the code gave before the session was
      // taken into account.
      CHECK(describe(starts(departure_map, on_car)) == "[wp 1 lane - loc 0]");
      CHECK(
        describe(starts(departure_map, in_car)) ==
        "[wp 1 lane 0 loc 1][wp 0 lane 1 loc 1]");
    }
  }
}

//==============================================================================
namespace {

rmf_task::Event::ConstStatePtr find_event(
  const rmf_task::Event::ConstStatePtr& state,
  const std::string& name)
{
  rmf_task::VersionedString::Reader reader;
  if (*reader.read(state->name()) == name)
    return state;

  for (const auto& dep : state->dependencies())
  {
    if (auto found = find_event(dep, name))
      return found;
  }

  return nullptr;
}

struct LocalizeRecord
{
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::pair<std::string, std::optional<std::size_t>>> calls;
};

} // anonymous namespace

//==============================================================================
SCENARIO_METHOD(LiftFixture, "execute plan localizes after a replanned ride",
  "[events]")
{
  using rmf_lift_msgs::msg::LiftState;

  const auto record = std::make_shared<LocalizeRecord>();
  schedule_and_wait<bool>(
    [&]()
    {
      context->set_localization(
        [record](
          agv::EasyFullControl::Destination estimate,
          agv::EasyFullControl::CommandExecution)
        {
          // The localization is never reported as finished, so the ride never
          // reaches the lane driven summon, which localizes on its own.
          std::unique_lock<std::mutex> lock(record->mutex);
          record->calls.emplace_back(estimate.map(), estimate.graph_index());
          record->cv.notify_all();
        });
      return true;
    });

  // Keep the robot from moving inside the car, so the only summon that can
  // finish is the one that ExecutePlan::make re-inserts after the replan.
  command->pause(true);

  const auto publisher = node->create_publisher<LiftState>(
    LiftStateTopicName, rclcpp::SystemDefaultsQoS().reliable());

  // Runs ExecutePlan::make for a replan that starts inside the car on the map
  // of the floor the robot boarded from, then lets the lift arrive at
  // `arrival_floor`. Returns whether the re-inserted summon completed.
  const auto ride = [&](const std::string& arrival_floor) -> bool
  {
    const auto now = rmf_traffic_ros2::convert(adapter->node()->now());
    const rmf_traffic::agv::Plan::Start start(
      now, 1, 0.0, Eigen::Vector2d(0.3, 0.0));
    const auto plan = context->planner()->plan(
      start, rmf_traffic::agv::Plan::Goal(3));
    REQUIRE(plan.success());

    auto execution = schedule_and_wait<std::optional<ExecutePlan>>(
      [&]()
      {
        auto state = rmf_task::events::SimpleEventState::make(
          0, "test", "", rmf_task::Event::Status::Standby, {},
          context->clock());

        return ExecutePlan::make(
          context, context->itinerary().assign_plan_id(), *plan,
          rmf_traffic::agv::Plan::Goal(3), plan->get_itinerary(),
          rmf_task::Event::AssignID::make(), state, []() {}, []() {},
          std::nullopt);
      });
    REQUIRE(execution.has_value());

    const auto summon = find_event(
      execution->sequence->state(),
      "Requesting lift [" + lift_name + "] to [" + arrival_floor + "]");
    REQUIRE(summon);

    bool completed = false;
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline)
    {
      LiftState state;
      state.lift_name = lift_name;
      state.lift_time = node->now();
      state.motion_state = LiftState::MOTION_STOPPED;
      state.destination_floor = arrival_floor;
      state.current_floor = arrival_floor;
      state.session_id = context->requester_id();
      state.door_state = LiftState::DOOR_OPEN;
      state.current_mode = LiftState::MODE_AGV;
      publisher->publish(state);

      std::unique_lock<std::mutex> lock(record->mutex);
      if (!record->calls.empty())
        break;

      if (summon->status() == rmf_task::Event::Status::Completed)
      {
        completed = true;
        break;
      }

      record->cv.wait_for(lock, std::chrono::milliseconds(100));
    }

    // The execution belongs to the worker, so release it there.
    schedule_and_wait<bool>(
      [&]()
      {
        execution.reset();
        return true;
      });

    return completed;
  };

  GIVEN("the robot holds a lift session requested from inside")
  {
    const auto session = schedule_and_wait<std::shared_ptr<void>>(
      [&]()
      {
        return context->set_lift_destination(
          lift_name, destination_map, true);
      });
    REQUIRE(session);

    THEN("the re-inserted summon localizes onto the destination floor")
    {
      ride(destination_map);

      std::unique_lock<std::mutex> lock(record->mutex);
      REQUIRE(record->calls.size() == 1);
      CHECK(record->calls.front().first == destination_map);
      REQUIRE(record->calls.front().second.has_value());
      CHECK(*record->calls.front().second == 2);
    }
  }

  GIVEN("the robot holds a lift session requested from outside")
  {
    const auto session = schedule_and_wait<std::shared_ptr<void>>(
      [&]()
      {
        return context->set_lift_destination(
          lift_name, destination_map, false);
      });
    REQUIRE(session);

    THEN("the re-inserted summon completes without a localization")
    {
      // Without the session guard of #556 the summon follows the map of the
      // first graph waypoint, which is the floor the robot boarded from.
      CHECK(ride(departure_map));

      std::unique_lock<std::mutex> lock(record->mutex);
      CHECK(record->calls.empty());
    }
  }
}

} // namespace test
} // namespace events
} // namespace rmf_fleet_adapter
