General information about this repository, including legal information, build instructions and known issues/limitations, can be found in the [README](../README.md) of the repository root.

# The micro-ROS diagnostic updater package

micro-ROS diagnostic updater is a [ROS 2](http://www.ros2.org/) package that provides convenience functions to implement diagnostic tasks amd updaters based on the ROS Client C-Library (RCLC) for micro-ROS.

An exemplary implementation can be found in: [example/example_updater.c](example_updater.c).

The examples are disabled by default, to build the examples, build with `MICRO_ROS_DIAGNOSTIC_UPDATER_EXAMPLES=ON`

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## How to Build, Test, Install, and Use

After you cloned this repository into your ROS 2 workspace folder, you may build and install it using colcon:
```
colcon build --packages-select micro_ros_diagnostic_updater
```

### Modify the namespace for your output topic

By default, the `micro_ros_diagnostic_updater` will output to `<namespace>/diagnostics_uros`. As this could be an inconvenience for your implementation. We provide a CMake build option `MICRO_ROS_DIAGNOSTIC_UPDATER_DIAGNOSTICS_TOPIC_PREFIX` that can be used to modify the topic while building.

Note

        * The topic will always end on diagnostics_uros
        * It's important that your prefix ends in `/`. Or you'll see `/my/topicdiagnostics_uros`
        * If your prefix begins with `/` the topic will follow the FQDN rules.

To build with this option allowing the final FQDN to take in the namespace of the node.

```
colcon build --packages-select micro_ros_diagnostic_updater --cmake-args -DMICRO_ROS_DIAGNOSTIC_UPDATER_DIAGNOSTICS_TOPIC_PREFIX=my/other/namespaces/
```

To build with this option providing the FQDN
```
colcon build --packages-select micro_ros_diagnostic_updater --cmake-args -DMICRO_ROS_DIAGNOSTIC_UPDATER_DIAGNOSTICS_TOPIC_PREFIX=/this/is/final/
```

### Modify max tasks per updater

By default, the max amount of tasks an updater can have is set to `5`. This number can be too big or too small for some cases. Therefor, we provide a CMake build option `MICRO_ROS_DIAGNOSTIC_UPDATER_MAX_TASKS_PER_UPDATER` to modify such value.

To build with this flag, i.e. with now 8 max tasks per updater.

```
colcon build --packages-select micro_ros_diagnostic_updater --cmake-args -DMICRO_ROS_DIAGNOSTIC_UPDATER_MAX_TASKS_PER_UPDATER=8
```

### Build with examples ###

As mentioned, this package does not build the examples by default, to do so, you can build with

```
colcon build --packages-select micro_ros_diagnostic_updater --cmake-args -DMICRO_ROS_DIAGNOSTIC_UPDATER_EXAMPLES=ON
```

### Publish only on update and Force update ###

The updater won't publish statuses of task who's data is unchanged, this is to reduce the traffic and processing needed by the updater on each iteration. However, due to different reasons, one may want to force the updater to publish everything. This is done with a subscription that is added to the executor passed on the initialization of the updater. The subscriber will be listening for a message of type `std_msgs/msg/Empty`, and the topic is always `<namespace>/diagnostics_uros/force_update`. Keep in mind, the `<namespace>` can be modified as indicated above.

## License

The micro-ROS diagnostics framework packages are open-sourced under the Apache-2.0 license. See the [../LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 micro_ros_diagnostics,
see the file [../3rd-party-licenses.txt](3rd-party-licenses.txt).

## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
