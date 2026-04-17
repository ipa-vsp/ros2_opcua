# Copyright 2026 Vishnuprasad Prachandabhanu / Fraunhofer IPA.
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for NodeId.msg round-trips and edge cases."""

from opcua_ros2_interfaces.msg import NodeId
import rclpy.serialization
from unique_identifier_msgs.msg import UUID


def _rt(msg):
    """Serialize then deserialize msg and return the copy."""
    raw = rclpy.serialization.serialize_message(msg)
    return rclpy.serialization.deserialize_message(raw, type(msg))


def test_numeric_nodeid():
    """Numeric NodeId preserves namespace index and numeric id."""
    nid = NodeId(
        namespace_index=0,
        identifier_type=NodeId.TYPE_NUMERIC,
        numeric_id=84,  # UA_NS0ID_ROOTFOLDER
    )
    nid2 = _rt(nid)
    assert nid2.namespace_index == 0
    assert nid2.identifier_type == NodeId.TYPE_NUMERIC
    assert nid2.numeric_id == 84


def test_string_nodeid():
    """String NodeId preserves namespace index and string id."""
    nid = NodeId(
        namespace_index=2,
        identifier_type=NodeId.TYPE_STRING,
        string_id='MyDevice.Temperature',
    )
    nid2 = _rt(nid)
    assert nid2.namespace_index == 2
    assert nid2.string_id == 'MyDevice.Temperature'


def test_guid_nodeid():
    """GUID NodeId preserves all 16 UUID bytes."""
    uid = UUID(uuid=list(range(16)))
    nid = NodeId(
        namespace_index=1,
        identifier_type=NodeId.TYPE_GUID,
        guid_id=uid,
    )
    nid2 = _rt(nid)
    assert nid2.identifier_type == NodeId.TYPE_GUID
    assert list(nid2.guid_id.uuid) == list(range(16))


def test_opaque_nodeid():
    """Opaque NodeId preserves raw byte sequence."""
    nid = NodeId(
        namespace_index=3,
        identifier_type=NodeId.TYPE_OPAQUE,
        opaque_id=[0xFF, 0x00, 0xAB],
    )
    nid2 = _rt(nid)
    assert list(nid2.opaque_id) == [0xFF, 0x00, 0xAB]


def test_type_constants():
    """TYPE_* constants match OPC UA identifier type enum values."""
    assert NodeId.TYPE_NUMERIC == 0
    assert NodeId.TYPE_STRING == 1
    assert NodeId.TYPE_GUID == 2
    assert NodeId.TYPE_OPAQUE == 3


def test_zero_namespace():
    """Namespace index 0 (OPC UA base namespace) serializes correctly."""
    nid = NodeId(namespace_index=0, identifier_type=NodeId.TYPE_NUMERIC, numeric_id=0)
    nid2 = _rt(nid)
    assert nid2.namespace_index == 0
    assert nid2.numeric_id == 0


def test_max_namespace():
    """Maximum namespace index (65535) serializes correctly."""
    nid = NodeId(
        namespace_index=65535,
        identifier_type=NodeId.TYPE_STRING,
        string_id='max',
    )
    nid2 = _rt(nid)
    assert nid2.namespace_index == 65535
