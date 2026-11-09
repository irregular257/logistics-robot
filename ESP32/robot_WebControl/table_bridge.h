#pragma once

// Set BOTH this and LOG_EXTERNAL_TABLE_ENABLED (STM32) to 1 after configuring
// the two STA MAC addresses and arming the physically aligned external table.
#ifndef TABLE_NOW_ENABLED
#define TABLE_NOW_ENABLED 0
#endif

#if TABLE_NOW_ENABLED
#include <turntable_protocol.h>
#include <turntable_radio.h>

// External table's STA MAC, NOT its AP MAC. Both radios use channel 1 so the
// Robot-Car browser hotspot can stay active alongside ESP-NOW.
#ifndef TABLE_NOW_PEER_MAC_BYTES
#define TABLE_NOW_PEER_MAC_BYTES {0, 0, 0, 0, 0, 0}
#endif
static const uint8_t TABLE_NOW_PEER_MAC[6] = TABLE_NOW_PEER_MAC_BYTES;
static const uint8_t TABLE_NOW_CHANNEL = 1;

class TableBridge {
 public:
  enum Fault : uint8_t {
    NO_FAULT = 0, NOT_CONFIGURED = 1, RADIO_INIT = 2, LINK_LOST = 3,
    COMMAND_TIMEOUT = 4, COMMAND_REJECTED = 5, TABLE_REBOOTED = 6,
    RECEIVE_OVERFLOW = 7, STM_LINK_LOST = 8, CANCELLED = 9,
    TABLE_FAULT = 10, SESSION_CHANGED = 11
  };

  bool begin() {
    Serial.print("ESP-NOW car STA MAC: "); Serial.println(WiFi.macAddress());
    carBoot_ = nonzeroRandom();
    uint8_t any = 0;
    for (uint8_t i = 0; i < 6; ++i) any |= TABLE_NOW_PEER_MAC[i];
    if (!any || (TABLE_NOW_PEER_MAC[0] & 1U)) {
      fault_ = NOT_CONFIGURED;
      Serial.println("Set TABLE_NOW_PEER_MAC before enabling the external table");
      return false;
    }
    initialized_ = radio_.begin(TABLE_NOW_PEER_MAC, TABLE_NOW_CHANNEL);
    if (!initialized_) fault_ = RADIO_INIT;
    return initialized_;
  }

  void onUartSession() {
    // An STM reset may reuse its tick-seeded task ID. Invalidate the old run,
    // then allow one START message from the new UART session to create a fresh
    // random radio run. Reconnection itself never sends START or grants HELD.
    if (active_) fail(STM_LINK_LOST);
    freshStartAllowed_ = true;
  }

  // Runs only from the main loop/UART parser; callbacks merely enqueue packets.
  void onRequest(const WebFrame &frame) {
    if (frame.command != WEB_CMD_TABLE_REQUEST || frame.length != 9U) return;
    uint8_t op = frame.payload[0];
    uint32_t task = Web_Get32(frame.payload + 1);
    uint32_t window = Web_Get32(frame.payload + 5);
    if (op < 1U || op > 4U || !task) return;
    if (op == 1U && (!active_ || task != taskId_ || freshStartAllowed_)) {
      // Only a new operator-started STM task can initiate a new radio run.
      // An already-running run is never replaced by another START.
      if (active_ && !aborting_ && state_ != table_now::DONE) {
        fail(SESSION_CHANGED); return;
      }
      taskId_ = task; run_ = nonzeroRandom(); active_ = true;
      freshStartAllowed_ = false;
      starting_ = true; startAt_ = millis(); pending_ = false;
      holdApproved_ = false;
      aborting_ = false; fault_ = initialized_ ? NO_FAULT : fault_;
      state_ = table_now::IDLE; picked_ = 0; window_ = 0;
      lastOp_ = 1U; lastWindow_ = 0;
      // Do not reuse an old task's ready/held status while START is pending.
      return;
    }
    if (!active_ || task != taskId_) return;
    if (op == 4U) { fail(CANCELLED); return; }
    if (fault_ || aborting_ || starting_) return;
    if (op == lastOp_ && window == lastWindow_) return;
    if (pending_) return; // Stop-and-wait: retry the original command verbatim.
    if (op == 2U) {
      if (state_ != table_now::READY || window != window_) return;
      queue(table_now::HOLD, window);
    } else if (op == 3U) {
      if (state_ != table_now::HELD || window != window_) return;
      queue(table_now::PICK_DONE, window);
    } else return;
    lastOp_ = op; lastWindow_ = window;
  }

  void tick(bool stmOnline) {
    if (!initialized_) return;
    // Enforce elapsed deadlines BEFORE an old queued ACK can clear pending_.
    uint32_t now = millis();
    if (active_ && !aborting_) {
      if (!stmOnline) fail(STM_LINK_LOST);
      else if (pending_ && (uint32_t)(now - commandAt_) >= 600U) fail(COMMAND_TIMEOUT);
      else if (starting_ && (uint32_t)(now - startAt_) >= 600U) fail(LINK_LOST);
      else if (!starting_ && (uint32_t)(now - statusAt_) >= 600U) fail(LINK_LOST);
    }
    table_now::Packet packet;
    uint32_t receivedAt;
    while (radio_.receive(packet, &receivedAt)) {
      if ((uint32_t)(millis() - receivedAt) < 300U) accept(packet, receivedAt);
    }
    if (radio_.overflowed() && active_) fail(RECEIVE_OVERFLOW);
    now = millis();
    if (!active_) return;
    if (!stmOnline) fail(STM_LINK_LOST);
    if (!aborting_ && starting_) {
      if (seen_ && (uint32_t)(now - statusAt_) < 300U) {
        if (peerState_ != table_now::IDLE && peerState_ != table_now::DONE) fail(COMMAND_REJECTED);
        else if (!pending_) queue(table_now::START, 0);
      }
      if ((uint32_t)(now - startAt_) >= 600U) fail(LINK_LOST);
    } else if (!aborting_ && (uint32_t)(now - statusAt_) >= 600U) fail(LINK_LOST);

    if (aborting_) {
      // ABORT remains repeatable even after its ACK, until a new explicit task.
      if (seen_ && (uint32_t)(now - retryAt_) >= 100U && radio_.send(command_)) retryAt_ = now;
      return;
    }
    if (pending_) {
      if ((uint32_t)(now - commandAt_) >= 600U) { fail(COMMAND_TIMEOUT); return; }
      if (!sent_ || (uint32_t)(now - retryAt_) >= 40U) {
        if (radio_.send(command_)) { sent_ = true; retryAt_ = now; }
      }
    }
    if ((uint32_t)(now - heartbeatAt_) >= 100U) {
      table_now::Packet heartbeat = table_now::makePacket(table_now::HEARTBEAT);
      heartbeat.tableBoot = tableBoot_; heartbeat.carBoot = carBoot_; heartbeat.run = run_;
      if (radio_.send(heartbeat)) heartbeatAt_ = now;
    }
  }

  bool active() const { return active_; }

  void fillStatus(uint8_t *out) const {
    // A cached HELD never survives a fault or a pending HOLD/START command.
    out[0] = fault_ ? table_now::FAULT : state_;
    out[1] = picked_; out[2] = fault_;
    Web_Put32(out + 3, taskId_); Web_Put32(out + 7, window_);
    Web_Put32(out + 11, tableBoot_);
  }

 private:
  table_now::Radio radio_;
  table_now::Packet command_ = {};
  uint32_t carBoot_ = 0, tableBoot_ = 0, run_ = 0, taskId_ = 0;
  uint32_t sequence_ = 0, peerSequence_ = 0, statusAt_ = 0;
  uint32_t commandAt_ = 0, retryAt_ = 0, heartbeatAt_ = 0, startAt_ = 0;
  uint32_t window_ = 0, lastWindow_ = 0;
  uint32_t holdWindow_ = 0;
  uint8_t state_ = table_now::IDLE, peerState_ = table_now::IDLE;
  uint8_t picked_ = 0, fault_ = NO_FAULT, lastOp_ = 0;
  bool initialized_ = false, seen_ = false, active_ = false, starting_ = false;
  bool pending_ = false, sent_ = false, aborting_ = false;
  bool holdApproved_ = false;
  bool freshStartAllowed_ = false;

  static uint32_t nonzeroRandom() {
    uint32_t value; do { value = esp_random(); } while (!value); return value;
  }
  void queue(table_now::Type type, uint32_t window) {
    command_ = table_now::makePacket(type);
    command_.tableBoot = tableBoot_; command_.carBoot = carBoot_; command_.run = run_;
    if (!++sequence_) ++sequence_;
    command_.sequence = sequence_; command_.window = window;
    pending_ = true; sent_ = false; commandAt_ = millis(); retryAt_ = 0;
  }
  void fail(uint8_t reason) {
    if (!fault_) fault_ = reason;
    state_ = table_now::FAULT; starting_ = false; holdApproved_ = false;
    if (aborting_) return;
    aborting_ = true; pending_ = false;
    queue(table_now::ABORT, window_); pending_ = false;
    retryAt_ = millis() - 100U;
    Serial.printf("External table fault %u; grasp inhibited\n", (unsigned)fault_);
  }
  void accept(const table_now::Packet &packet, uint32_t receivedAt) {
    if (!table_now::validPacket(packet) || packet.type != table_now::STATUS || !packet.tableBoot) return;
    if (seen_ && packet.tableBoot == tableBoot_ &&
        !table_now::newer32(packet.sequence, peerSequence_)) return;
    if (seen_ && packet.tableBoot != tableBoot_ && active_) {
      // Changing the boot ID invalidates all prior window and hold tokens.
      tableBoot_ = packet.tableBoot; peerSequence_ = packet.sequence; statusAt_ = receivedAt;
      peerState_ = packet.state; fail(TABLE_REBOOTED); return;
    }
    tableBoot_ = packet.tableBoot; peerSequence_ = packet.sequence;
    statusAt_ = receivedAt; seen_ = true; peerState_ = packet.state;
    if (!active_ || aborting_) return;
    bool ours = packet.carBoot == carBoot_ && packet.run == run_;
    if (!ours) {
      if (!starting_) fail(SESSION_CHANGED);
      return;
    }
    if (packet.state == table_now::FAULT) { fail(TABLE_FAULT); return; }
    // Status is execution evidence, not ESP-NOW MAC send completion.
    if (pending_ && packet.ack == command_.sequence) {
      if (packet.result == table_now::OK) {
        if (command_.type == table_now::START) starting_ = false;
        if (command_.type == table_now::HOLD) {
          if (packet.state != table_now::HELD || packet.window != command_.window) {
            fail(COMMAND_REJECTED); return;
          }
          holdApproved_ = true; holdWindow_ = command_.window;
        } else if (command_.type == table_now::PICK_DONE) holdApproved_ = false;
        pending_ = false;
      } else if (command_.type == table_now::HOLD &&
                 (packet.result == table_now::STALE_WINDOW || packet.result == table_now::BAD_STATE)) {
        // A late request loses this window; keep scanning, never grasp mid-move.
        pending_ = false;
      } else { fail(COMMAND_REJECTED); return; }
    }
    if (starting_) return;
    state_ = packet.state; picked_ = packet.picked; window_ = packet.window;
    // HELD is exposed only after the matching HOLD received its application ACK.
    if (state_ == table_now::HELD && (!holdApproved_ || window_ != holdWindow_))
      state_ = table_now::READY;
  }
};

#else
// No ESP-NOW dependency when disabled; existing car behavior and host tests stay intact.
class TableBridge {
 public:
  bool begin() { return false; }
  void onUartSession() {}
  void onRequest(const WebFrame &) {}
  void tick(bool) {}
  bool active() const { return false; }
  void fillStatus(uint8_t *out) const { memset(out, 0, 15); out[0] = 6; out[2] = 1; }
};
#endif
