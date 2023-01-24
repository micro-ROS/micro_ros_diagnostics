# Possible levels of operations.
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

# Possible value types
byte VALUE_BOOL=1
byte VALUE_INT=2
byte VALUE_FLOAT=3
byte VALUE_LOOKUP=10

# Level of operation enumerated above.
byte level

# What key, value is transmitted.
uint16 key

byte value_type
bool bool_value
int32 int_value
float32 double_value
uint16 value_id
