General information about this repository, including legal information, build instructions and known issues/limitations, can be found in the [README](../README.md) of the repository root.

# Micro-ROS Diagnostic Messages

The Micro-ROS Diagnostic Messages is a ROS 2 package that provides micro-ROS-specific message types ad service types for diagnostics.
Message and service types are copies of the respective types from ROS 2 common_interfaces/diagnostic_msgswith minor adaptions for micro-ROS.

General information about this repository, including legal information, project context, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## How to Build, Test, Install, and Use

After you cloned this repository into your ROS 2 workspace folder, you may build and install it using colcon:
$ `colcon build --packages-select micro_ros_diagnostic_msgs`

## License

The micro-ROS diagnostics framework packages are open-sourced under the Apache-2.0 license. See the [../LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 micro_ros_diagnostics,
see the file [../3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

Please notice the following issues/limitations:

1. Due to limitations in the Micro-ROS agent, the micro-ROS diagnostics framework can't (yet) publish the default ROS 2 diagnostic messages [diagnostic_msgs](https://github.com/ros2/common_interfaces/tree/master/diagnostic_msgs), but provides simplified versions of the diagnostic messages and services that go without arrays, [MicroROSDiagnosticStatus](msg/MicroROSDiagnosticStatus.msg) and [MicroROSSelfTest](srv/MicroROSSelfTest.srv). These simplified messages and services will have to be translated by the agent or a 3rd party.

## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
