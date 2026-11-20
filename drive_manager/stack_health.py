"""Detect conflicting copies of mission and Nav2 nodes before transitions."""

from collections import Counter


STACK_NODES = frozenset({
    "mission_driver", "command_manager", "web_teleop", "nav2_supervisor",
    "lifecycle_manager_localization", "lifecycle_manager_navigation",
    "map_server", "amcl", "controller_server", "planner_server",
    "smoother_server", "route_server", "behavior_server", "bt_navigator",
    "waypoint_follower", "velocity_smoother", "collision_monitor",
    "docking_server", "front_scan_filter",
})


def duplicate_stack_nodes(names_and_namespaces):
    counts = Counter(
        (name, namespace) for name, namespace in names_and_namespaces
        if name in STACK_NODES
    )
    return sorted(
        f"{namespace.rstrip('/')}/{name} ({count})"
        for (name, namespace), count in counts.items() if count > 1
    )
