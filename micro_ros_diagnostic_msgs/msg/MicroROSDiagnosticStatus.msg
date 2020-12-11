# This message holds the status of an individual component of the robot.
#
# This message specification is a copy of
# common_interfaces/diagnostic_msgs/msg/DiagnosticStatus.msg
# with the difference that it comprises just one key and value
# instead of an array of KeyValue messages.

# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Level of operation enumerated above.
byte level
# A description of the test/component reporting.
string name
# A description of the status.
string message
# A hardware unique string.
string hardware_id
# What to label this value when viewing.
string key
# A value to track over time.
string value
