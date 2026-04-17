uint8 TYPE_NUMERIC = 0
uint8 TYPE_STRING  = 1
uint8 TYPE_GUID    = 2
uint8 TYPE_OPAQUE  = 3

uint16 namespace_index
uint8  identifier_type
uint32 numeric_id
string string_id
unique_identifier_msgs/UUID guid_id
uint8[] opaque_id
