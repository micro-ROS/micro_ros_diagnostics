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

# Possible value types
byte VALUE_BOOL=1
byte VALUE_INT=2
byte VALUE_DOUBLE=3
byte VALUE_LOOKUP=10

# Level of operation enumerated above.
byte level

# A description of the test/component reporting.
uint8 updater_id
# A hardware unique string.
uint8 hardware_id

# What key, value is transmitted.
uint8 key

byte value_type
bool bool_value
int64 int_value
float64 double_value
int64 value_id
