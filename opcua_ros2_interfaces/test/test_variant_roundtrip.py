# Copyright 2026 Vishnuprasad Prachandabhanu / Fraunhofer IPA.
# SPDX-License-Identifier: Apache-2.0
"""Round-trip unit tests for Variant.msg covering every TYPE_* tag."""

from builtin_interfaces.msg import Time
from opcua_ros2_interfaces.msg import (
    LocalizedText,
    NodeId,
    QualifiedName,
    StatusCode,
    Variant,
)
import pytest
import rclpy.serialization
from unique_identifier_msgs.msg import UUID


def _roundtrip(msg):
    """Serialize then deserialize a ROS 2 message and return the copy."""
    raw = rclpy.serialization.serialize_message(msg)
    return rclpy.serialization.deserialize_message(raw, type(msg))


def _v(**kwargs):
    v = Variant()
    for k, val in kwargs.items():
        setattr(v, k, val)
    return v


def test_type_constants_are_ordered():
    """Verify every TYPE_* constant equals its spec value."""
    assert Variant.TYPE_NULL == 0
    assert Variant.TYPE_BOOL == 1
    assert Variant.TYPE_INT8 == 2
    assert Variant.TYPE_UINT8 == 3
    assert Variant.TYPE_INT16 == 4
    assert Variant.TYPE_UINT16 == 5
    assert Variant.TYPE_INT32 == 6
    assert Variant.TYPE_UINT32 == 7
    assert Variant.TYPE_INT64 == 8
    assert Variant.TYPE_UINT64 == 9
    assert Variant.TYPE_FLOAT == 10
    assert Variant.TYPE_DOUBLE == 11
    assert Variant.TYPE_STRING == 12
    assert Variant.TYPE_DATETIME == 13
    assert Variant.TYPE_GUID == 14
    assert Variant.TYPE_BYTESTRING == 15
    assert Variant.TYPE_NODEID == 16
    assert Variant.TYPE_QUALIFIED_NAME == 17
    assert Variant.TYPE_LOCALIZED_TEXT == 18
    assert Variant.TYPE_STATUS_CODE == 19
    assert Variant.TYPE_EXTENSION_OBJECT == 20


@pytest.mark.parametrize('type_tag,field,value', [
    (Variant.TYPE_BOOL,   'bool_value',   True),
    (Variant.TYPE_INT8,   'int_value',    -128),
    (Variant.TYPE_UINT8,  'uint_value',   255),
    (Variant.TYPE_INT16,  'int_value',    -32768),
    (Variant.TYPE_UINT16, 'uint_value',   65535),
    (Variant.TYPE_INT32,  'int_value',    -2147483648),
    (Variant.TYPE_UINT32, 'uint_value',   4294967295),
    (Variant.TYPE_INT64,  'int_value',    -(2**63)),
    (Variant.TYPE_UINT64, 'uint_value',   2**64 - 1),
    (Variant.TYPE_FLOAT,  'double_value', 3.14),
    (Variant.TYPE_DOUBLE, 'double_value', 2.718281828),
    (Variant.TYPE_STRING, 'string_value', 'hello OPC UA'),
])
def test_scalar_roundtrip(type_tag, field, value):
    """Each scalar type tag survives a serialize-deserialize cycle."""
    v = _v(type=type_tag, **{field: value})
    v2 = _roundtrip(v)
    assert v2.type == type_tag
    if isinstance(value, float):
        assert getattr(v2, field) == pytest.approx(value, rel=1e-6)
    else:
        assert getattr(v2, field) == value


def test_datetime_roundtrip():
    """Variant with datetime value preserves sec and nanosec."""
    t = Time(sec=1_700_000_000, nanosec=123456789)
    v2 = _roundtrip(_v(type=Variant.TYPE_DATETIME, datetime_value=t))
    assert v2.datetime_value.sec == t.sec
    assert v2.datetime_value.nanosec == t.nanosec


def test_guid_roundtrip():
    """Variant with GUID value preserves all 16 bytes."""
    uid = UUID(uuid=list(range(16)))
    v2 = _roundtrip(_v(type=Variant.TYPE_GUID, guid_value=uid))
    assert list(v2.guid_value.uuid) == list(uid.uuid)


def test_bytestring_roundtrip():
    """Variant with bytestring value preserves byte content."""
    bs = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    v2 = _roundtrip(_v(type=Variant.TYPE_BYTESTRING, bytestring_value=list(bs)))
    assert bytes(v2.bytestring_value) == bs


def test_nodeid_roundtrip():
    """Variant with nodeid value preserves namespace index and numeric id."""
    nid = NodeId(namespace_index=3, identifier_type=NodeId.TYPE_NUMERIC, numeric_id=1001)
    v2 = _roundtrip(_v(type=Variant.TYPE_NODEID, nodeid_value=nid))
    assert v2.nodeid_value.namespace_index == 3
    assert v2.nodeid_value.numeric_id == 1001


def test_qualified_name_roundtrip():
    """Variant with qualified name value preserves name string."""
    qn = QualifiedName(namespace_index=1, name='Temperature')
    v2 = _roundtrip(_v(type=Variant.TYPE_QUALIFIED_NAME, qualified_name_value=qn))
    assert v2.qualified_name_value.name == 'Temperature'


def test_localized_text_roundtrip():
    """Variant with localized text value preserves locale and text."""
    lt = LocalizedText(locale='en-US', text='Emergency Stop')
    v2 = _roundtrip(_v(type=Variant.TYPE_LOCALIZED_TEXT, localized_text_value=lt))
    assert v2.localized_text_value.text == 'Emergency Stop'


def test_status_code_roundtrip():
    """Variant with status code value preserves value and name."""
    sc = StatusCode(value=0x80350000, name='BadNodeIdUnknown')
    v2 = _roundtrip(_v(type=Variant.TYPE_STATUS_CODE, status_code_value=sc))
    assert v2.status_code_value.value == 0x80350000
    assert v2.status_code_value.name == 'BadNodeIdUnknown'


def test_bool_array_roundtrip():
    """Bool array variant round-trips correctly."""
    v2 = _roundtrip(_v(type=Variant.TYPE_BOOL, is_array=True, bool_array=[True, False, True]))
    assert v2.is_array is True
    assert list(v2.bool_array) == [True, False, True]


def test_int_array_roundtrip():
    """Int array variant round-trips correctly."""
    v2 = _roundtrip(_v(type=Variant.TYPE_INT32, is_array=True, int_array=[-1, 0, 2147483647]))
    assert list(v2.int_array) == [-1, 0, 2147483647]


def test_double_array_roundtrip():
    """Double array variant round-trips correctly."""
    vals = [1.1, 2.2, 3.3]
    v2 = _roundtrip(_v(type=Variant.TYPE_DOUBLE, is_array=True, double_array=vals))
    assert list(v2.double_array) == pytest.approx(vals)


def test_string_array_roundtrip():
    """String array variant round-trips correctly."""
    v2 = _roundtrip(_v(
        type=Variant.TYPE_STRING, is_array=True,
        string_array=['alpha', 'beta', 'gamma'],
    ))
    assert list(v2.string_array) == ['alpha', 'beta', 'gamma']


def test_null_variant_default():
    """Default-constructed variant is TYPE_NULL and not an array."""
    v = Variant()
    assert v.type == Variant.TYPE_NULL
    assert v.is_array is False
