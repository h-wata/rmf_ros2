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

#include <agv/RobotContext.hpp>
#include <agv/internal_FleetUpdateHandle.hpp>
#include <agv/internal_RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_fleet_msgs/msg/mutex_group_manual_release.hpp>
#include <rmf_fleet_msgs/msg/mutex_group_states.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include "../mock/MockRobotCommand.hpp"

#include <rmf_utils/catch.hpp>

#include <unistd.h>

namespace rmf_fleet_adapter {
namespace agv {
namespace test {

namespace {

const std::string lift_group = "lift";
const std::string corridor_group = "corridor";

// A graph where one mutex group covers the car vertex of a lift on both
// floors and the lanes out of the car, the way a deployment shares one lift
// between robots. The plan is never computed here, so the floors do not need
// to be connected.
//
//   L1:  4 [corridor]
//        |  [corridor]
//        0 ------------- 1 (car) [lift]
//                    [lift]
//   L2:                  2 (car) [lift] ------------- 3
//                                         [lift]
//
struct MutexFixture
{
  std::shared_ptr<rclcpp::Context> rcl_context;
  std::shared_ptr<MockAdapter> adapter;
  std::shared_ptr<FleetUpdateHandle> fleet;
  std::shared_ptr<Node> node;
  rmf_traffic::agv::Graph graph;

  std::shared_ptr<RobotContext> context;
  std::shared_ptr<rmf_fleet_adapter_test::MockRobotCommand> command;

  rclcpp::Publisher<rmf_fleet_msgs::msg::MutexGroupStates>::SharedPtr
    states_pub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::MutexGroupManualRelease>::SharedPtr
    manual_release_pub;

  static std::size_t node_counter;

  MutexFixture()
  {
    // These tests play the part of the mutex group supervisor, so keep them
    // away from any supervisor, emergency signal or other test running on the
    // default domain of this machine.
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(150 + getpid() % 80);
    rcl_context = std::make_shared<rclcpp::Context>();
    rcl_context->init(0, nullptr, init_options);

    adapter = std::make_shared<MockAdapter>(
      "test_mutex_groups_" + std::to_string(node_counter++),
      rclcpp::NodeOptions().context(rcl_context));

    graph.add_waypoint("L1", {0.0, 0.0}).set_charger(true); // 0
    graph.add_waypoint("L1", {5.0, 0.0}).set_in_mutex_group(lift_group); // 1
    graph.add_waypoint("L2", {5.0, 0.0}).set_in_mutex_group(lift_group); // 2
    graph.add_waypoint("L2", {10.0, 0.0}); // 3
    graph.add_waypoint("L1", {0.0, 5.0}).set_in_mutex_group(corridor_group); // 4

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0).properties().set_in_mutex_group(lift_group); // 1
    graph.add_lane(2, 3).properties().set_in_mutex_group(lift_group); // 2
    graph.add_lane(3, 2); // 3
    graph.add_lane(0, 4).properties().set_in_mutex_group(corridor_group); // 4
    graph.add_lane(4, 0).properties().set_in_mutex_group(corridor_group); // 5

    const rmf_traffic::Profile profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(0.1)
    };

    const rmf_traffic::agv::VehicleTraits traits{
      {0.7, 0.3}, {1.0, 0.45}, profile};

    fleet = adapter->add_fleet("test_fleet", traits, graph);
    node = FleetUpdateHandle::Implementation::get(*fleet).node;
    adapter->start();

    const auto qos = rclcpp::SystemDefaultsQoS()
      .reliable()
      .transient_local()
      .keep_last(100);
    states_pub = node->create_publisher<rmf_fleet_msgs::msg::MutexGroupStates>(
      MutexGroupStatesTopicName, qos);
    manual_release_pub =
      node->create_publisher<rmf_fleet_msgs::msg::MutexGroupManualRelease>(
      MutexGroupManualReleaseTopicName,
      rclcpp::SystemDefaultsQoS().reliable().keep_last(10));

    const auto now = rmf_traffic_ros2::convert(adapter->node()->now());
    command = std::make_shared<rmf_fleet_adapter_test::MockRobotCommand>(
      node, graph);

    std::promise<void> robot_added;
    fleet->add_robot(
      command, "test_robot", profile,
      rmf_traffic::agv::Plan::StartSet{{now, 0, 0.0}},
      [this, &robot_added](RobotUpdateHandlePtr updater)
      {
        context = RobotUpdateHandle::Implementation::get(*updater)
        .context.lock();
        command->updater = updater;
        robot_added.set_value();
      });

    robot_added.get_future().wait();

    // A robot without a task force-releases its mutex groups, so pretend that
    // a task is underway.
    on_worker<bool>([this]()
      {
        context->current_task_id("task_a");
        return true;
      });
  }

  ~MutexFixture()
  {
    std::weak_ptr<rclcpp::Node> weak_node = node;
    node->stop();
    states_pub.reset();
    manual_release_pub.reset();
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

  template<typename T>
  T on_worker(std::function<T()> job) const
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

  bool wait_until(std::function<bool()> condition) const
  {
    for (std::size_t i = 0; i < 100; ++i)
    {
      if (on_worker<bool>(condition))
        return true;

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return false;
  }

  bool locked(const std::string& group) const
  {
    return on_worker<bool>([this, group]()
        {
          return context->locked_mutex_groups().count(group) > 0;
        });
  }

  rmf_traffic::Time now() const
  {
    return on_worker<rmf_traffic::Time>([this]() { return context->now(); });
  }

  void place(rmf_traffic::agv::Plan::StartSet location) const
  {
    on_worker<bool>([this, location]()
      {
        context->set_location(location);
        return true;
      });
  }

  void place_at(std::size_t waypoint) const
  {
    place({rmf_traffic::agv::Plan::Start(now(), waypoint, 0.0)});
  }

  void retain(std::unordered_set<std::string> groups) const
  {
    on_worker<bool>([this, groups]()
      {
        context->retain_mutex_groups(groups, "test");
        return true;
      });
  }

  // Requests the group and then plays the part of the mutex group supervisor
  // by assigning it to this robot.
  void lock(const std::string& group, rmf_traffic::Time claim_time) const
  {
    on_worker<bool>([this, group, claim_time]()
      {
        context->request_mutex_groups({group}, claim_time);
        return true;
      });

    rmf_fleet_msgs::msg::MutexGroupAssignment assignment;
    assignment.group = group;
    assignment.claimant = context->participant_id();
    assignment.claim_time = rmf_traffic_ros2::convert(claim_time);
    rmf_fleet_msgs::msg::MutexGroupStates states;
    states.assignments.push_back(assignment);
    states_pub->publish(states);

    REQUIRE(wait_until([this, group]()
      {
        return context->locked_mutex_groups().count(group) > 0;
      }));
  }

  void lock(const std::string& group) const
  {
    lock(group, now());
  }
};

std::size_t MutexFixture::node_counter = 0;

} // anonymous namespace

//==============================================================================
SCENARIO_METHOD(
  MutexFixture,
  "retain_mutex_groups keeps the groups of the occupied location",
  "[mutex]")
{
  lock(lift_group);
  lock(corridor_group);

  WHEN("The robot is on a car vertex that the remaining plan leaves out")
  {
    place_at(2);
    retain({});

    THEN("The group of the car vertex is kept and the other one is released")
    {
      CHECK(locked(lift_group));
      CHECK_FALSE(locked(corridor_group));
    }

    AND_WHEN("The robot leaves the group")
    {
      place_at(3);
      retain({});

      THEN("The group is released on the next retain")
      {
        CHECK_FALSE(locked(lift_group));
      }
    }
  }

  WHEN("The robot is on a lane out of the car")
  {
    place({rmf_traffic::agv::Plan::Start(
          now(), 3, 0.0, Eigen::Vector2d(7.5, 0.0), 2)});
    retain({});

    THEN("The group of the lane is kept")
    {
      CHECK(locked(lift_group));
      CHECK_FALSE(locked(corridor_group));
    }
  }

  WHEN("The robot still reports the boarding floor while the plan is on the "
    "destination floor")
  {
    // The robot is believed to be on the L1 car vertex, and the plan starts
    // from L2 without the lift group.
    place_at(1);
    retain({corridor_group});

    THEN("The group is kept because the car vertex of every floor is in it")
    {
      CHECK(locked(lift_group));
      CHECK(locked(corridor_group));
    }
  }

  WHEN("The robot is off the groups")
  {
    place_at(0);

    THEN("Only the groups in the plan are kept, as before")
    {
      retain({corridor_group});
      CHECK_FALSE(locked(lift_group));
      CHECK(locked(corridor_group));
    }
  }

  WHEN("The robot is lost")
  {
    place_at(2);
    on_worker<bool>([this]()
      {
        context->set_lost(std::nullopt);
        return true;
      });
    retain({});

    THEN("Nothing is released because the occupied location is unknown")
    {
      CHECK(locked(lift_group));
      CHECK(locked(corridor_group));
    }

    AND_WHEN("The robot is found off the groups")
    {
      place_at(0);
      retain({});

      THEN("Both groups are released")
      {
        CHECK_FALSE(locked(lift_group));
        CHECK_FALSE(locked(corridor_group));
      }
    }
  }

  WHEN("An operator manually releases the group of the occupied location")
  {
    place_at(2);
    rmf_fleet_msgs::msg::MutexGroupManualRelease msg;
    msg.fleet = context->group();
    msg.robot = context->name();
    msg.release_mutex_groups.push_back(lift_group);
    manual_release_pub->publish(msg);

    THEN("The group is released anyway")
    {
      CHECK(wait_until([this]()
        {
          return context->locked_mutex_groups().count(lift_group) == 0;
        }));
      CHECK(locked(corridor_group));
    }
  }
}

} // namespace test
} // namespace agv
} // namespace rmf_fleet_adapter
