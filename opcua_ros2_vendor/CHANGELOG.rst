^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package opcua_ros2_vendor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-04-17)
------------------
* Initial vendored build of open62541pp pinned to
  6b83f185e0fbc78f98647ff4afd848b8a17cbc60.
* OpenSSL-backed encryption (UA_ENABLE_ENCRYPTION=OPENSSL).
* Subscriptions, events, method calls, discovery, and JSON encoding enabled.
* Upstream examples and tests disabled.
* Ships the open62541pp_datatype_headers-extras CMake helper.
