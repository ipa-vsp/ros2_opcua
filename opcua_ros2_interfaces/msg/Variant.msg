uint8 TYPE_NULL             = 0
uint8 TYPE_BOOL             = 1
uint8 TYPE_INT8             = 2
uint8 TYPE_UINT8            = 3
uint8 TYPE_INT16            = 4
uint8 TYPE_UINT16           = 5
uint8 TYPE_INT32            = 6
uint8 TYPE_UINT32           = 7
uint8 TYPE_INT64            = 8
uint8 TYPE_UINT64           = 9
uint8 TYPE_FLOAT            = 10
uint8 TYPE_DOUBLE           = 11
uint8 TYPE_STRING           = 12
uint8 TYPE_DATETIME         = 13
uint8 TYPE_GUID             = 14
uint8 TYPE_BYTESTRING       = 15
uint8 TYPE_NODEID           = 16
uint8 TYPE_QUALIFIED_NAME   = 17
uint8 TYPE_LOCALIZED_TEXT   = 18
uint8 TYPE_STATUS_CODE      = 19
uint8 TYPE_EXTENSION_OBJECT = 20

uint8 type

# Scalar fields — exactly one is meaningful based on `type`.
bool    bool_value
int64   int_value          # covers int8..int64 (narrowed by `type`)
uint64  uint_value         # covers uint8..uint64
float64 double_value       # covers float32/float64
string  string_value
builtin_interfaces/Time       datetime_value
unique_identifier_msgs/UUID   guid_value
uint8[]                       bytestring_value
NodeId                        nodeid_value
QualifiedName                 qualified_name_value
LocalizedText                 localized_text_value
StatusCode                    status_code_value

# Array support — when is_array is true the repeated fields below are used
# instead of the scalar fields above.
bool    is_array
int32[] array_dimensions   # multi-dimensional arrays; empty = 1-D

bool[]           bool_array
int64[]          int_array
uint64[]         uint_array
float64[]        double_array
string[]         string_array
builtin_interfaces/Time[]      datetime_array
unique_identifier_msgs/UUID[]  guid_array
uint8[]                        bytestring_array  # concatenated; not segmented
NodeId[]                       nodeid_array
QualifiedName[]                qualified_name_array
LocalizedText[]                localized_text_array
StatusCode[]                   status_code_array

# ExtensionObject — structured type whose layout is defined by extension_data_type.
# Decoded/encoded by generated codecs from opcua_ros2_idl.
NodeId  extension_data_type
uint8[] extension_body     # binary-encoded body (UA binary encoding)
