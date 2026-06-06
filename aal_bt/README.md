# aal_bt

BehaviorTree.CPP nodes for the Autonomous Adaptation Layer (AAL). Provides the `EnsureConfiguration` decorator, a ROS 2 package-independent BT plugin.

## EnsureConfiguration

A `DecoratorNode` that sends a desired lifecycle configuration to an action server before running its child, then tears down on completion or preemption.

**Ports:**

| Port | Type | Default | Description |
|---|---|---|---|
| `action_name` | `string` | `/reconfigure_system` | `DesiredSystemConfiguration` action server |
| `nodes_to_activate` | `string[]` | — | Nodes that must be active before the child runs |
| `nodes_to_deactivate` | `string[]` | `[]` | Nodes that must be inactive before the child runs |
| `nodes_to_deactivate_on_exit` | `string[]` | `[]` | Teardown override (see below) |

**Teardown behaviour:**

| `nodes_to_deactivate_on_exit` | Effect |
|---|---|
| Not set / empty | Deactivate whatever was in `nodes_to_activate` |
| `none` | Skip teardown entirely |
| Any other value | Deactivate exactly those nodes |

The action server (not this decorator) is responsible for computing the transition sequence and checking current node states.

**BT XML usage** — values are semicolon-separated, no brackets:

```xml
<!-- Default teardown -->
<EnsureConfiguration action_name="/reconfigure_system"
                     nodes_to_activate="identify_node"
                     nodes_to_deactivate="explore_node">
  <YourAction/>
</EnsureConfiguration>

<!-- Multiple nodes -->
<EnsureConfiguration action_name="/reconfigure_system"
                     nodes_to_activate="identify_node;sensor_node"
                     nodes_to_deactivate_on_exit="identify_node">
  <YourAction/>
</EnsureConfiguration>

<!-- No teardown -->
<EnsureConfiguration action_name="/reconfigure_system"
                     nodes_to_activate="identify_node"
                     nodes_to_deactivate_on_exit="none">
  <YourAction/>
</EnsureConfiguration>
```

## Loading the Plugin

```cpp
#include "ament_index_cpp/get_package_share_directory.hpp"

std::string plugin_path =
    ament_index_cpp::get_package_share_directory("aal_bt")
    + "/bt_plugins/" + BT::SharedLibrary::getOSName("aal_bt_nodes");
factory.registerFromPlugin(plugin_path);
```

## Action Interface

The decorator uses `aal_msgs/action/DesiredSystemConfiguration`, whose goal carries an `aal_msgs/msg/DesiredConfiguration`:

```
string[] active_nodes
string[] inactive_nodes
```

## Running Tests

Requires a built workspace:

```bash
cd ~/ros_workspaces/suave_rebetmc_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select aal_bt --event-handlers console_cohesion+
colcon test-result --verbose
```

Tests use an in-process mock action server — no external dependencies required.
