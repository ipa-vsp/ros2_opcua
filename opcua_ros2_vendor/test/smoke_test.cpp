// Copyright 2026 Vishnuprasad Prachandabhanu / Fraunhofer IPA.
// SPDX-License-Identifier: Apache-2.0
//
// Vendor-package smoke test: instantiates an open62541pp Client, attempts to
// connect to an unreachable endpoint, and asserts the library exits cleanly
// with a failure StatusCode (i.e. no crash, no leak, no undefined symbol).

#include <gtest/gtest.h>

#include <open62541pp/client.hpp>

TEST(Open62541ppVendor, ClientFailsToConnectCleanly)
{
  opcua::Client client;

  // 127.0.0.1:65535 is a non-listening address on any sane host. The call must
  // return a bad status rather than throw or hang indefinitely.
  try {
    client.connect("opc.tcp://127.0.0.1:65535");
    FAIL() << "connect() unexpectedly succeeded against an unreachable endpoint";
  } catch (const opcua::BadStatus & e) {
    // open62541pp signals connection failures as BadStatus-derived exceptions.
    // Any non-good status is acceptable for the smoke test.
    EXPECT_NE(e.code(), UA_STATUSCODE_GOOD);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
