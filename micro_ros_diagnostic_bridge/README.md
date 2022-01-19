General information about this repository, including legal information, build instructions and known issues/limitations, can be found in the [README](../README.md) of the repository root.

# The micro-ROS diagnostic bridge package

micro-ROS diagnostic bridge is a [ROS 2](http://www.ros2.org/) package that provides a bridge to translate micro-ROS diagnostic messages to vanilla ROS 2 diagnostic messages based on a lookup table.

An exemplary lookup table can be found in: [example_table.yaml](example/example_table.yaml), along with an [example launch file](example/launch/example_diagnostic_bridge.launch.py)

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
colcon build --packages-select micro_ros_diagnostic_bridge
```

### Change default input topics or output topic

#### Input

By default, the `micro_ros_diagnostic_bridge` will listen for updates on `<namespace>/diagnostics_uros`. However, it may happen that, as a user, the namespaces where your bridge and updater node run are not the same. For simplicity for the user, the default launch file for the bridge has a launch configuration so that one can pass the remapping needed.

Please read the README of the `micro_ros_diagnostic_updater` for information on how to modify the default topic name.

Note

        The updater will always have the form <prefix>/diagnostics_uros

#### Output

By default, the `micro_ros_diagnostic_bridge` will publish to `<namespace>/diagnostics`. In order for the user to be able to adjust this to their convenience, the default launch file for the bridge has a launch configuration to modify such topic.

## Launching

To launch with remappings

```
ros2 launch micro_ros_diagnostic_bridge diagnostic_bridge.launch.py input_topic:=[/]your/new/input/topicname output_topic:=[/]your/topic/diagnostics
```

Note:

        * There's no need to use the word diagnostics on the output. However, it's recommended. Use it on your convenience.
        * The input topic is always diagnostics_uros. The user may prepend namespaces to it.
        * If your remap begins with a `/` you'll be providing a FQDN which will not respect your node namespace.
        * Your input_topic:= needs to match to your updater output_topic

   

### Build with examples ###

As mentioned, this package does not build the examples by default, to do so, you can build with

```
colcon build --packages-select micro_ros_diagnostic_bridge --cmake-args -DMICRO_ROS_DIAGNOSTIC_BRIDGE_EXAMPLES=ON
```

## License

The micro-ROS diagnostics framework packages are open-sourced under the Apache-2.0 license. See the [../LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [../3rd-party-licenses.txt](3rd-party-licenses.txt).

## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
