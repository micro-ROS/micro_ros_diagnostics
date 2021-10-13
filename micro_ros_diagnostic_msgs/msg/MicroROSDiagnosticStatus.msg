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
uint16 updater_id
# A hardware unique string.
uint16 hardware_id

# What key, value is transmitted.
uint16 key

byte value_type
bool bool_value
int32 int_value
float32 double_value
uint16 value_id
