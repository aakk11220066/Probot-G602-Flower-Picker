// Copyright 2011 The Closure Library Authors. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS-IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

goog.provide('goog.storage.ExpiringStorageTest');
goog.setTestOnly('goog.storage.ExpiringStorageTest');

goog.require('goog.storage.ExpiringStorage');
goog.require('goog.storage.storage_test');
goog.require('goog.testing.MockClock');
goog.require('goog.testing.jsunit');
goog.require('goog.testing.storage.FakeMechanism');

function testBasicOperations() {
  var mechanism = new goog.testing.storage.FakeMechanism();
  var storage = new goog.storage.ExpiringStorage(mechanism);
  goog.storage.storage_test.runBasicTests(storage);
}

function testExpiration() {
  var mechanism = new goog.testing.storage.FakeMechanism();
  var clock = new goog.testing.MockClock(true);
  var storage = new goog.storage.ExpiringStorage(mechanism);

  // No expiration.
  storage.set('first', 'one second', 1000);
  storage.set('second', 'permanent');
  storage.set('third', 'two seconds', 2000);
  storage.set('fourth', 'permanent');
  clock.tick(100);
  assertEquals('one second', storage.get('first'));
  assertEquals('permanent', storage.get('second'));
  assertEquals('two seconds', storage.get('third'));
  assertEquals('permanent', storage.get('fourth'));

  // A key has expired.
  clock.tick(1000);
  assertUndefined(storage.get('first'));
  assertEquals('permanent', storage.get('second'));
  assertEquals('two seconds', storage.get('third'));
  assertEquals('permanent', storage.get('fourth'));
  assertNull(mechanism.get('first'));

  // Add an already expired key.
  storage.set('fourth', 'one second again', 1000);
  assertNull(mechanism.get('fourth'));
  assertUndefined(storage.get('fourth'));

  // Another key has expired.
  clock.tick(1000);
  assertEquals('permanent', storage.get('second'));
  assertUndefined(storage.get('third'));
  assertNull(mechanism.get('third'));

  // Clean up.
  storage.remove('second');
  assertNull(mechanism.get('second'));
  assertUndefined(storage.get('second'));
  clock.uninstall();
}

function testClockSkew() {
  var mechanism = new goog.testing.storage.FakeMechanism();
  var storage = new goog.storage.ExpiringStorage(mechanism);
  var clock = new goog.testing.MockClock(true);

  // Simulate clock skew.
  clock.tick(100);
  storage.set('first', 'one second', 1000);
  clock.reset();
  assertUndefined(storage.get('first'));
  assertNull(mechanism.get('first'));

  clock.uninstall();
}
