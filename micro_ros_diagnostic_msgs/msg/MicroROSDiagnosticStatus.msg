# This message holds the status of an individual component of the robot.
#
# This message specification is inspired by
# common_interfaces/diagnostic_msgs/msg/DiagnosticStatus.msg
# with the difference that it comprises just one key and value
# instead of an array of KeyValue messages as well as avoiding
# any strings.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level

# A description of the test/component reporting.
uint16 updater_id
# A hardware unique string.
uint16 hardware_id

uint8 number_of_values
micro_ros_diagnostic_msgs/MicroROSDiagnosticKeyValue[] values
