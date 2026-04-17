# Naming conventions for `opcua_ros2_interfaces`

## Field names

OPC UA specifies field names in camelCase (e.g. `browseName`, `nodeId`,
`securityMode`). ROS 2 IDL requires `snake_case` for wire field names. We
follow ROS 2 style on the wire:

| OPC UA spec     | This package   |
|-----------------|----------------|
| `browseName`    | `browse_name`  |
| `nodeId`        | `node_id`      |
| `displayName`   | `display_name` |
| `securityMode`  | `security_mode`|
| `applicationUri`| `application_uri` |

## Numeric constants

Constants that enumerate OPC UA values (e.g. `SecurityMode`, `NodeClass`,
`ApplicationType`) are expressed as `uint8` constants inside the message
that owns them. Names are `SCREAMING_SNAKE_CASE` with a short prefix:

```
uint8 SECURITY_MODE_NONE             = 1
uint8 SECURITY_MODE_SIGN             = 2
uint8 SECURITY_MODE_SIGN_AND_ENCRYPT = 3
```

The numeric values **match the OPC UA specification** exactly, not a
zero-based enumeration, to allow direct casting in C++.

## Variant field design

`Variant.msg` uses a parallel-field layout (one scalar field per type +
matching array fields) rather than nested messages. Rationale:

- Avoids the nested-message size tax for common scalar cases.
- Maps cleanly onto `std::variant<bool, int64_t, …>` in `opcua_ros2_core`.
- The `is_array` flag selects between scalar and array interpretations;
  only the field matching `type` is meaningful in either mode.

See `ARCHITECTURE.md §2.3` for the full discussion.

## No external dependencies

This package depends **only** on `builtin_interfaces` and
`unique_identifier_msgs`. Downstream packages that need `open62541pp` types
must depend on `opcua_ros2_vendor` separately.
