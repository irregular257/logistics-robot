/* ESP32-WROOM / DevKit, Arduino IDE, no third-party libraries required.
 * GPIO17 TX -> STM32 PA1 (UART4 RX); GPIO16 RX <- PA0 (UART4 TX).
 * Flash the matching robot STM32 project as well. Common GND, 3.3 V UART.
 */
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <esp_system.h>
#include <Preferences.h>
#include "web_protocol.h"
#include "control_state.h"
#include "uart_parser.h"
#include "web_page.h"

const char *AP_SSID = "Robot-Car";
const char *AP_PASSWORD = "robot12345"; // At least 8 characters.
constexpr int STM_RX_PIN = 16;
constexpr int STM_TX_PIN = 17;
HardwareSerial StmSerial(2);
WebServer server(80);
BrowserControl control;
UartParser parser;
Preferences servoPreferences;

uint32_t uartSession = 0;
uint16_t uartSequence = 0;
bool helloAccepted = false, statusReceived = false, haveStatusSequence = false;
uint16_t statusSequence = 0;
uint32_t statusAt = 0, helloAt = 0, heartbeatAt = 0;
uint8_t stmMode = WEB_MODE_IDLE, stmFault = 0, stmClaw = 2;
uint8_t lastFault = 0;
uint32_t lastFaultAt = 0;
bool liftEnabled = false;
bool autoSupported = false;
bool gimbalSupported = false;
int16_t gimbalAngle = -1;
int16_t clawAngle = -1;
bool traySupported = false;
int16_t trayAngle = -1;
bool servoPresetsSupported = false, presetStorageReady = false;
uint16_t servoPresets[4] = {10, 70, 54, 170};
uint16_t savedPresets[4] = {10, 70, 54, 170};
uint16_t queuedPresets[4];
bool savedPresetsValid = false, presetsPending = false, presetSaveRequested = false;
bool presetRestoreDone = false;
uint16_t presetApplySequence = 0;
uint32_t presetApplyAt = 0, presetRestoreAt = 0;
uint8_t presetRestoreAttempts = 0;
uint8_t autoState = 0, autoDrop = 0, autoError = 0;
int16_t autoXcm = 30, autoYcm = 30;
int8_t stmJog[4] = {0, 0, 0, 0};
bool ackReceived = false;
uint8_t lastAckCommand = 0, lastAckResult = 0;
uint16_t lastAckSequence = 0;
uint32_t ackAt = 0;
uint16_t operationSequence = 0;
uint8_t operationCommand = 0;
bool taskSupported = false, stmTaskReady = false, taskSubmitted = false, taskPending = false;
uint8_t taskPlan[WEB_TASK_PLAN_SIZE] = {}, queuedTaskPlan[WEB_TASK_PLAN_SIZE] = {};
uint16_t taskApplySequence = 0;
uint32_t taskApplyAt = 0;

bool freshStatus() {
  return helloAccepted && statusReceived && (uint32_t)(millis() - statusAt) < 600U;
}
uint32_t randomNonzero() {
  uint32_t value; do { value = esp_random(); } while (!value); return value;
}
uint16_t sendFrame(uint8_t command, const uint8_t *payload = nullptr, uint8_t length = 0) {
  WebFrame frame = {};
  frame.session = uartSession; frame.sequence = ++uartSequence;
  frame.command = command; frame.length = length;
  if (command != WEB_CMD_HELLO && command != WEB_CMD_HEARTBEAT) {
    operationSequence = frame.sequence; operationCommand = command; ackReceived = false;
  }
  if (length) memcpy(frame.payload, payload, length);
  uint8_t wire[WEB_MAX_FRAME]; uint8_t size = Web_Encode(&frame, wire);
  StmSerial.write(wire, size); return frame.sequence;
}
void stopAndRevoke() {
  // Revoke before sending: a delayed MODE/JOG with the old token is rejected.
  taskPending = false;
  control.clear(); sendFrame(WEB_CMD_STOP);
}
void beginSession() {
  uartSession = randomNonzero(); uartSequence = 0;
  helloAccepted = false; statusReceived = false; haveStatusSequence = false;
  stmMode = WEB_MODE_IDLE; stmFault = 0; liftEnabled = false; ackReceived = false;
  operationCommand = 0; operationSequence = 0; autoSupported = false;
  taskSupported = stmTaskReady = taskSubmitted = taskPending = false;
  memset(taskPlan, 0, sizeof(taskPlan));
  gimbalSupported = false; gimbalAngle = -1;
  traySupported = false; trayAngle = -1;
  servoPresetsSupported = false; clawAngle = -1; presetsPending = false;
  presetSaveRequested = false; presetRestoreDone = false; presetRestoreAttempts = 0;
  presetRestoreAt = millis();
  parser.reset(); statusAt = helloAt = millis(); sendFrame(WEB_CMD_HELLO);
}
bool newer16(uint16_t value, uint16_t previous) {
  return (int16_t)(value - previous) > 0;
}
void receiveFrame(const WebFrame &frame) {
  if (frame.session != uartSession) return;
  if (frame.command == WEB_CMD_ACK && frame.length == 4) {
    uint8_t command = frame.payload[0], result = frame.payload[1];
    if (command == WEB_CMD_HELLO && result == WEB_RESULT_OK)
      helloAccepted = true;
    // Heartbeats are transport traffic, not the last operator action.
    if (command == operationCommand && frame.sequence == operationSequence) {
      ackReceived = true; lastAckCommand = command; lastAckResult = result;
      lastAckSequence = frame.sequence; ackAt = millis();
    }
    if (command == WEB_CMD_TASK && taskPending && frame.sequence == taskApplySequence) {
      taskPending = false;
      taskSubmitted = result == WEB_RESULT_OK && (uint32_t)(millis() - taskApplyAt) < 1500U;
      if (taskSubmitted) memcpy(taskPlan, queuedTaskPlan, sizeof(taskPlan));
    }
    if (command == WEB_CMD_SERVO_PRESETS && presetsPending && frame.sequence == presetApplySequence) {
      presetsPending = false;
      if (result == WEB_RESULT_OK) {
        presetRestoreDone = true;
        if (presetSaveRequested && presetStorageReady) {
          if (servoPreferences.putBytes("angles", queuedPresets, sizeof(queuedPresets)) == sizeof(queuedPresets)) {
            memcpy(savedPresets, queuedPresets, sizeof(savedPresets)); savedPresetsValid = true;
          } else { presetStorageReady = false; savedPresetsValid = false; }
        }
      }
      presetSaveRequested = false;
    }
    if (command == WEB_CMD_MODE && control.armPending && frame.sequence == control.armSequence) {
      bool ok = result == WEB_RESULT_OK && frame.payload[2] == WEB_MODE_MANUAL &&
                frame.payload[3] == 0 && freshStatus();
      if (control.acceptArmAck(frame.sequence, ok, millis()) && !ok) stopAndRevoke();
    }
  } else if (frame.command == WEB_CMD_STATUS && (frame.length == 8 || frame.length == 16 || frame.length == 18 || frame.length == WEB_SERVO_STATUS_SIZE || frame.length == WEB_TRAY_STATUS_SIZE || frame.length == WEB_TASK_STATUS_SIZE)) {
    if (haveStatusSequence && !newer16(frame.sequence, statusSequence)) return;
    haveStatusSequence = true; statusSequence = frame.sequence;
    statusReceived = true; statusAt = millis();
    stmMode = frame.payload[0];
    if (frame.payload[1] && frame.payload[1] != stmFault) { lastFault = frame.payload[1]; lastFaultAt = millis(); }
    stmFault = frame.payload[1]; stmClaw = frame.payload[2];
    liftEnabled = frame.payload[3] != 0;
    for (int i = 0; i < 4; ++i) stmJog[i] = (int8_t)frame.payload[4 + i];
    autoSupported = frame.length >= 16 && frame.payload[11] == 1;
    if (autoSupported) {
      autoState=frame.payload[8]; autoDrop=frame.payload[9]; autoError=frame.payload[10];
      autoXcm=(int16_t)Web_Get16(frame.payload+12);
      autoYcm=(int16_t)Web_Get16(frame.payload+14);
    }
    gimbalSupported = frame.length >= 18;
    servoPresetsSupported = frame.length >= WEB_SERVO_STATUS_SIZE;
    uint16_t angle = gimbalSupported ? Web_Get16(frame.payload + 16) : 0xFFFFU;
    uint16_t minimum = servoPresetsSupported ? 0U : 54U, maximum = servoPresetsSupported ? 270U : 229U;
    gimbalAngle = angle >= minimum && angle <= maximum ? (int16_t)angle : -1;
    clawAngle = -1;
    if (servoPresetsSupported) {
      uint16_t target = Web_Get16(frame.payload + 18);
      if (target <= 270U) clawAngle = (int16_t)target;
      uint16_t received[4]; bool valid = true;
      for (int i = 0; i < 4; ++i) { received[i] = Web_Get16(frame.payload + 20 + 2 * i); if (received[i] > 270U) valid = false; }
      if (valid) memcpy(servoPresets, received, sizeof(servoPresets));
      else servoPresetsSupported = false;
    }
    taskSupported = frame.length == WEB_TASK_STATUS_SIZE && frame.payload[30] == WEB_TASK_PROTOCOL_VERSION;
    stmTaskReady = taskSupported && frame.payload[31] == 1U;
    if (!taskSupported) { taskSubmitted = false; taskPending = false; }
    traySupported = frame.length >= WEB_TRAY_STATUS_SIZE;
    uint16_t trayTarget = traySupported ? Web_Get16(frame.payload + 28) : WEB_TRAY_UNKNOWN_ANGLE;
    trayAngle = trayTarget <= WEB_TRAY_MAX_ANGLE ? (int16_t)trayTarget : -1;
    if (control.armed && (stmMode != WEB_MODE_MANUAL || stmFault)) stopAndRevoke();
  }
}
void serviceUart() {
  WebFrame frame;
  int budget = 256;
  while (budget-- && StmSerial.available()) {
    if (parser.feed((uint8_t)StmSerial.read(), millis(), frame)) {
      receiveFrame(frame);
      while (parser.take(frame)) receiveFrame(frame);
    }
  }
}
bool parseInteger(const String &value, int32_t minimum, int32_t maximum, int32_t &out) {
  if (!value.length() || value.length() > 11) return false;
  size_t pos = 0; bool negative = value[0] == '-';
  if (negative) ++pos;
  if (pos == value.length()) return false;
  int64_t result = 0;
  for (; pos < value.length(); ++pos) {
    char ch = value[pos]; if (ch < '0' || ch > '9') return false;
    result = result * 10 + (ch - '0'); if (result > 2147483648LL) return false;
  }
  if (negative) result = -result;
  if (result < minimum || result > maximum) return false;
  out = (int32_t)result; return true;
}
bool parseSequence(const String &value, uint32_t &out) {
  if (!value.length() || value.length() > 10) return false;
  uint64_t result = 0;
  for (size_t i = 0; i < value.length(); ++i) {
    char ch = value[i]; if (ch < '0' || ch > '9') return false;
    result = result * 10 + (ch - '0'); if (result > UINT32_MAX) return false;
  }
  out = (uint32_t)result; return out != 0;
}
uint64_t requestToken() {
  String value = server.hasArg("token") ? server.arg("token") : server.header("X-Token");
  if (value.length() != 16) return 0;
  uint64_t token = 0;
  for (size_t i = 0; i < 16; ++i) {
    char ch = value[i]; uint8_t digit;
    if (ch >= '0' && ch <= '9') digit = ch - '0';
    else if (ch >= 'a' && ch <= 'f') digit = ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F') digit = ch - 'A' + 10;
    else return 0;
    token = (token << 4) | digit;
  }
  return token;
}
void replyError(int code, const char *message) {
  server.send(code, "application/json; charset=utf-8", String("{\"ok\":false,\"error\":\"") + message + "\"}");
}
void replyQueued(uint16_t sequence) {
  server.send(202, "application/json", String("{\"ok\":true,\"queued\":true,\"uartSeq\":") + sequence + "}");
}
bool ownerRequired(bool ordered) {
  if (control.expired(millis())) stopAndRevoke();
  uint64_t token = requestToken();
  if (!control.owns(token)) { replyError(403, "请先申请控制权"); return false; }
  if (ordered) {
    uint32_t sequence;
    if (!parseSequence(server.arg("seq"), sequence)) { replyError(400, "seq无效"); return false; }
    if (!control.accept(token, sequence, millis())) { replyError(409, "过期或重复的操作"); return false; }
  } else control.lastSeen = millis();
  return true;
}
bool linkRequired() {
  if (!freshStatus()) { replyError(503, "STM32未连接，请检查UART4和主控固件"); return false; }
  return true;
}
bool motorsStopped() {
  return !control.moving && !stmJog[0] && !stmJog[1] && !stmJog[2] && !stmJog[3];
}
bool presetsSaved() {
  return servoPresetsSupported && savedPresetsValid && !presetsPending &&
         memcmp(savedPresets, servoPresets, sizeof(savedPresets)) == 0;
}
uint16_t queuePresets(const uint16_t *values, bool save) {
  uint8_t payload[8];
  memcpy(queuedPresets, values, sizeof(queuedPresets));
  for (int i = 0; i < 4; ++i) Web_Put16(payload + 2 * i, values[i]);
  presetsPending = true; presetSaveRequested = save; presetApplyAt = millis();
  presetApplySequence = sendFrame(WEB_CMD_SERVO_PRESETS, payload, sizeof(payload));
  return presetApplySequence;
}
void servicePresets() {
  uint32_t now = millis();
  if (presetsPending && (uint32_t)(now - presetApplyAt) >= 1200U) {
    presetsPending = false; presetSaveRequested = false; // Never persist an unacknowledged setting.
  }
  // Restore configuration only before the first browser takes ownership; this command never moves a servo.
  if (!savedPresetsValid || presetRestoreDone || presetsPending || !freshStatus() ||
      !servoPresetsSupported || stmMode != WEB_MODE_IDLE || stmFault || control.token || !motorsStopped()) return;
  if (presetsSaved()) { presetRestoreDone = true; return; }
  if (presetRestoreAttempts >= 3 || (uint32_t)(now - presetRestoreAt) > 10000U) return;
  if (presetRestoreAttempts && (uint32_t)(now - presetApplyAt) < 1500U) return;
  ++presetRestoreAttempts; queuePresets(savedPresets, false);
}
String jsonQuote(const String &value) {
  String output = "\"";
  for (size_t i = 0; i < value.length(); ++i) {
    char ch = value[i];
    if (ch == '\\' || ch == '"') output += '\\';
    if ((uint8_t)ch >= 0x20) output += ch;
  }
  return output + '"';
}
void handleStatus() {
  String json; json.reserve(650);
  bool link = freshStatus(); bool claimed = control.token && !control.expired(millis());
  json = String("{\"link\":") + (link ? "true" : "false") + ",\"stmOnline\":" + (link ? "true" : "false");
  json += String(",\"claimed\":") + (claimed ? "true" : "false") + ",\"ownerActive\":" + (claimed ? "true" : "false");
  json += String(",\"mine\":") + (claimed && control.owns(requestToken()) ? "true" : "false");
  json += String(",\"armed\":") + (control.armed ? "true" : "false");
  json += String(",\"pending\":") + (control.armPending ? "true" : "false");
  json += String(",\"mode\":") + stmMode + ",\"fault\":" + stmFault + ",\"claw\":" + stmClaw;
  json += String(",\"lastFault\":") + lastFault + ",\"lastFaultAgeMs\":" + (lastFault ? (int32_t)(millis() - lastFaultAt) : -1);
  json += String(",\"liftEnabled\":") + (liftEnabled ? "true" : "false");
  json += String(",\"gimbalSupported\":") + (gimbalSupported ? "true" : "false");
  json += String(",\"gimbalAngle\":") + gimbalAngle;
  json += String(",\"clawAngle\":") + clawAngle;
  json += String(",\"traySupported\":") + (traySupported ? "true" : "false");
  json += String(",\"trayAngle\":") + trayAngle;
  json += String(",\"servoPresetsSupported\":") + (servoPresetsSupported ? "true" : "false");
  json += String(",\"presetsPending\":") + (presetsPending ? "true" : "false");
  json += String(",\"presetsSaved\":") + (presetsSaved() ? "true" : "false");
  json += String(",\"presetStorageReady\":") + (presetStorageReady ? "true" : "false");
  json += String(",\"presets\":{\"clawClose\":") + servoPresets[0] + ",\"clawOpen\":" + servoPresets[1] + ",\"gimbalIn\":" + servoPresets[2] + ",\"gimbalOut\":" + servoPresets[3] + "}";
  json += String(",\"x\":") + stmJog[0] + ",\"y\":" + stmJog[1] + ",\"turn\":" + stmJog[2] + ",\"lift\":" + stmJog[3];
  json += String(",\"lastAckCommand\":") + lastAckCommand + ",\"lastAckCmd\":" + lastAckCommand;
  json += String(",\"lastAckSequence\":") + lastAckSequence + ",\"lastAckSeq\":" + lastAckSequence + ",\"lastAckResult\":" + lastAckResult;
  json += String(",\"ackValid\":") + (ackReceived ? "true" : "false");
  json += String(",\"ackAgeMs\":") + (ackReceived ? (int32_t)(millis() - ackAt) : -1);
  json += String(",\"statusAgeMs\":") + (statusReceived ? (int32_t)(millis() - statusAt) : -1);
  json += String(",\"taskSupported\":") + (taskSupported ? "true" : "false");
  json += String(",\"taskPending\":") + (taskPending ? "true" : "false");
  json += String(",\"taskAccepted\":") + (taskSubmitted && stmTaskReady && !taskPending ? "true" : "false");
  json += ",\"taskPlan\":[";
  for (int i = 0; i < WEB_TASK_PLAN_SIZE; ++i) { if (i) json += ','; json += String(taskPlan[i]); }
  json += ']';
  json += String(",\"autoSupported\":") + (autoSupported ? "true" : "false");
  json += String(",\"autoState\":") + autoState + ",\"autoDrop\":" + autoDrop + ",\"autoError\":" + autoError;
  json += String(",\"autoXcm\":") + autoXcm + ",\"autoYcm\":" + autoYcm + "}";
  server.sendHeader("Cache-Control", "no-store"); server.send(200, "application/json; charset=utf-8", json);
}
void handleClaim() {
  if (!linkRequired()) return;
  if (control.expired(millis())) stopAndRevoke();
  if (control.token) { replyError(409, "已有浏览器控制，请先释放控制权"); return; }
  if (presetsPending) { replyError(409, "正在恢复已保存的舵机角度，请稍后申请"); return; }
  uint64_t token = ((uint64_t)randomNonzero() << 32) | randomNonzero();
  control.claim(token, millis());
  presetRestoreDone = true; // A user session must never be overwritten by a later startup restore.
  char text[17]; snprintf(text, sizeof(text), "%08lx%08lx", (unsigned long)(token >> 32), (unsigned long)token);
  server.send(200, "application/json", String("{\"ok\":true,\"token\":\"") + text + "\"}");
}
void handleMode() {
  if (!ownerRequired(true) || !linkRequired()) return;
  int32_t value;
  if (!parseInteger(server.arg("value"), 0, 1, value)) { replyError(400, "模式无效"); return; }
  if (!value) { stopAndRevoke(); server.send(200, "application/json", "{\"ok\":true}"); return; }
  if (control.armPending) { replyError(409, "等待主控确认"); return; }
  if (stmFault) { replyError(409, "主控故障，请等待链路恢复后重新申请"); return; }
  control.armed = false; control.moving = false;
  uint8_t mode = WEB_MODE_MANUAL;
  control.armSequence = sendFrame(WEB_CMD_MODE, &mode, 1);
  control.armPending = true; control.armSentAt = millis(); replyQueued(control.armSequence);
}
void handleJog() {
  if (!ownerRequired(true) || !linkRequired()) return;
  if (!control.armed || control.armPending || stmMode != WEB_MODE_MANUAL) { replyError(409, "请先启用手动控制"); return; }
  const char *fields[] = {"x", "y", "turn", "lift"};
  int32_t values[4]; uint8_t payload[4];
  for (int i = 0; i < 4; ++i) {
    if (!parseInteger(server.arg(fields[i]), i == 3 ? -1 : -50, i == 3 ? 1 : 50, values[i])) {
      replyError(400, "运动参数无效"); return;
    }
    payload[i] = (uint8_t)(int8_t)values[i];
  }
  bool chassis = values[0] || values[1] || values[2];
  if (values[3] && chassis) { replyError(400, "底盘和升降不能同时运动"); return; }
  if (stmFault) { replyError(409, "主控故障，请停止并检查设备状态"); return; }
  if (values[3] && !liftEnabled) { replyError(409, "主控尚未开放升降，请更新固件并检查驱动器反馈"); return; }
  control.moving = chassis || values[3]; control.lastJog = millis();
  replyQueued(sendFrame(WEB_CMD_JOG, payload, 4));
}
void handleClaw() {
  if (!ownerRequired(true) || !linkRequired()) return;
  if (!control.armed || control.armPending || stmMode != WEB_MODE_MANUAL) { replyError(409, "请先启用手动控制"); return; }
  if (stmFault) { replyError(409, "主控故障，请先检查设备状态"); return; }
  if (!motorsStopped()) { replyError(409, "请等待底盘和升降停止后操作夹爪"); return; }
  int32_t value;
  if (!parseInteger(server.arg("value"), 0, 1, value)) { replyError(400, "夹爪参数无效"); return; }
  uint8_t payload = (uint8_t)value; replyQueued(sendFrame(WEB_CMD_CLAW, &payload, 1));
}
void handleGimbal() {
  if (!ownerRequired(true) || !linkRequired()) return;
  if (!gimbalSupported) { replyError(409, "请更新 STM32 云台控制固件"); return; }
  if (!control.armed || control.armPending || stmMode != WEB_MODE_MANUAL) { replyError(409, "请先启用手动控制"); return; }
  if (stmFault) { replyError(409, "主控故障，请先检查设备状态"); return; }
  if (!motorsStopped()) { replyError(409, "请等待底盘和升降停止后操作云台"); return; }
  int32_t value;
  if (!parseInteger(server.arg("angle"), servoPresetsSupported ? 0 : 54, servoPresetsSupported ? 270 : 229, value)) {
    replyError(400, "云台角度无效"); return;
  }
  uint8_t payload[2]; Web_Put16(payload, (uint16_t)value);
  replyQueued(sendFrame(WEB_CMD_GIMBAL, payload, sizeof(payload)));
}
void handleClawAngle() {
  if (!ownerRequired(true) || !linkRequired()) return;
  if (!servoPresetsSupported) { replyError(409, "请同时更新 STM32 和 ESP32 角度调试固件"); return; }
  if (!control.armed || control.armPending || stmMode != WEB_MODE_MANUAL || stmFault) { replyError(409, "请启用手动并确认无故障"); return; }
  if (!motorsStopped() || presetsPending) { replyError(409, "请等待底盘、升降及预设操作停止"); return; }
  int32_t value;
  if (!parseInteger(server.arg("angle"), 0, 270, value)) { replyError(400, "夹爪角度须为 0° 至 270° 的整数"); return; }
  uint8_t payload[2]; Web_Put16(payload, (uint16_t)value);
  replyQueued(sendFrame(WEB_CMD_CLAW_ANGLE, payload, sizeof(payload)));
}
void handleTrayAngle() {
  if (!ownerRequired(true) || !linkRequired()) return;
  if (!traySupported) { replyError(409, "请同时更新 STM32 和 ESP32 转盘控制固件"); return; }
  if (!control.armed || control.armPending || stmMode != WEB_MODE_MANUAL) { replyError(409, "请先启用手动控制"); return; }
  if (stmFault) { replyError(409, "主控故障，请先检查设备状态"); return; }
  if (!motorsStopped()) { replyError(409, "请等待底盘和升降停止后操作转盘"); return; }
  int32_t value;
  if (!parseInteger(server.arg("angle"), WEB_TRAY_MIN_ANGLE, WEB_TRAY_MAX_ANGLE, value)) {
    replyError(400, "转盘角度无效"); return;
  }
  uint8_t payload[2]; Web_Put16(payload, (uint16_t)value);
  replyQueued(sendFrame(WEB_CMD_TRAY_ANGLE, payload, sizeof(payload)));
}
void handleServoPresets() {
  if (!ownerRequired(true) || !linkRequired()) return;
  if (!servoPresetsSupported) { replyError(409, "请同时更新 STM32 和 ESP32 角度预设固件"); return; }
  if (!presetStorageReady) { replyError(503, "ESP32 保存空间不可用，无法保存预设"); return; }
  if (control.armPending || stmMode == WEB_MODE_AUTO || stmFault || !motorsStopped() || presetsPending) { replyError(409, "请停止运动，等待主控状态后保存预设"); return; }
  const char *fields[] = {"clawClose", "clawOpen", "gimbalIn", "gimbalOut"};
  uint16_t values[4]; int32_t value;
  for (int i = 0; i < 4; ++i) {
    if (!parseInteger(server.arg(fields[i]), 0, 270, value)) { replyError(400, "预设角度须为 0° 至 270° 的整数"); return; }
    values[i] = (uint16_t)value;
  }
  replyQueued(queuePresets(values, true));
}
void serviceTask() {
  if (taskPending && (uint32_t)(millis() - taskApplyAt) >= 1500U) {
    taskPending = false; taskSubmitted = false;
  }
}
bool parseTaskPlan(uint8_t *plan) {
  const char *fields[] = {"color1", "shape1", "color2", "shape2", "color3", "shape3"};
  for (int i = 0; i < WEB_TASK_PLAN_SIZE; ++i) {
    int32_t value;
    if (!parseInteger(server.arg(fields[i]), 1, i % 2 ? 6 : 7, value) ||
        (!(i % 2) && value != 1 && value != 2 && value != 3 && value != 4 && value != 7)) return false;
    plan[i] = (uint8_t)value;
  }
  for (int i = 0; i < WEB_TASK_PLAN_SIZE; i += 2) {
    uint8_t color = plan[i], shape = plan[i + 1];
    if (shape == 1) { if (color != 4) return false; }
    else if (shape == 3) { if (color == 7) return false; }
    else if (shape == 4) { if (color != 1 && color != 2 && color != 7) return false; }
    else if (shape != 5 && shape != 6) return false;
  }
  return true;
}
void handleTask() {
  if (!ownerRequired(true) || !linkRequired()) return;
  serviceTask();
  if (!taskSupported) { replyError(409, "请更新 STM32 三件物料任务固件"); return; }
  if (stmFault || stmMode == WEB_MODE_AUTO || control.armPending || !motorsStopped() || presetsPending || taskPending) {
    replyError(409, "请先停止运动，并等待当前指令确认后发送任务"); return;
  }
  uint8_t plan[WEB_TASK_PLAN_SIZE];
  if (!parseTaskPlan(plan)) { replyError(400, "请为三件物料选择有效的颜色和形状"); return; }
  memcpy(queuedTaskPlan, plan, sizeof(plan));
  taskSubmitted = false; taskPending = true; taskApplyAt = millis();
  taskApplySequence = sendFrame(WEB_CMD_TASK, queuedTaskPlan, sizeof(queuedTaskPlan));
  replyQueued(taskApplySequence);
}
void handleStart() {
  if (!ownerRequired(true) || !linkRequired()) return;
  serviceTask();
  if (!autoSupported || !taskSupported) { replyError(409,"请更新 STM32 三件物料任务固件"); return; }
  int32_t positioned;
  if (!parseInteger(server.arg("positioned"),1,1,positioned)) { replyError(400,"请确认起点、车头方向和升降搬运高度"); return; }
  uint8_t plan[WEB_TASK_PLAN_SIZE];
  if (!parseTaskPlan(plan)) { replyError(400, "请为三件物料选择有效的颜色和形状"); return; }
  if (!taskSubmitted || !stmTaskReady || taskPending || memcmp(plan, taskPlan, sizeof(plan)) != 0) {
    replyError(409, "请先发送当前抓取顺序，并等待主控确认"); return;
  }
  if (stmFault || stmMode==WEB_MODE_AUTO || control.armPending || !motorsStopped() || presetsPending) {
    replyError(409,"请先停止手动运动，检查设备状态"); return;
  }
  control.armed=false;
  uint8_t payload[1U + WEB_TASK_PLAN_SIZE] = {1U};
  memcpy(payload + 1, plan, sizeof(plan));
  replyQueued(sendFrame(WEB_CMD_START,payload,sizeof(payload)));
}
void handlePause() {
  if (!ownerRequired(true) || !linkRequired()) return;
  control.armed=false; control.moving=false;
  replyQueued(sendFrame(WEB_CMD_PAUSE));
}
void setup() {
  Serial.begin(115200);
  presetStorageReady = servoPreferences.begin("robot-servos", false);
  if (presetStorageReady && servoPreferences.getBytesLength("angles") == sizeof(savedPresets)) {
    savedPresetsValid = servoPreferences.getBytes("angles", savedPresets, sizeof(savedPresets)) == sizeof(savedPresets);
    for (int i = 0; i < 4; ++i) if (savedPresets[i] > 270U) savedPresetsValid = false;
  }
  StmSerial.setRxBufferSize(512); StmSerial.begin(115200, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);
  WiFi.mode(WIFI_AP); WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, 1, false, 4)) {
    Serial.println("Failed to start AP"); return;
  }
  const char *headers[] = {"X-Token"}; server.collectHeaders(headers, 1);
  server.on("/", HTTP_GET, []() { server.sendHeader("Cache-Control", "no-store"); server.send_P(200, "text/html; charset=utf-8", WEB_PAGE); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/claim", HTTP_POST, handleClaim);
  server.on("/api/ping", HTTP_POST, []() { if (ownerRequired(false)) server.send(200, "application/json", "{\"ok\":true}"); });
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/jog", HTTP_POST, handleJog);
  server.on("/api/claw", HTTP_POST, handleClaw);
  server.on("/api/gimbal", HTTP_POST, handleGimbal);
  server.on("/api/claw-angle", HTTP_POST, handleClawAngle);
  server.on("/api/tray-angle", HTTP_POST, handleTrayAngle);
  server.on("/api/servo-presets", HTTP_POST, handleServoPresets);
  server.on("/api/task", HTTP_POST, handleTask);
  server.on("/api/stop", HTTP_POST, []() { stopAndRevoke(); server.send(200, "application/json", "{\"ok\":true,\"stopped\":true}"); });
  server.on("/api/release", HTTP_POST, []() { if (ownerRequired(false)) { stopAndRevoke(); server.send(200, "application/json", "{\"ok\":true}"); } });
  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/pause", HTTP_POST, handlePause);
  server.onNotFound([]() { replyError(404, "接口不存在"); });
  server.begin(); beginSession();
  Serial.println("Connect to Robot-Car, open http://192.168.4.1/");
}
void loop() {
  serviceUart();
  uint32_t now = millis();
  if (control.expired(now) || control.jogExpired(now) || control.armExpired(now)) stopAndRevoke();
  if (helloAccepted && (uint32_t)(now - statusAt) >= 600U) {
    stopAndRevoke(); beginSession();
  }
  if (!helloAccepted && (uint32_t)(now - helloAt) >= 400U) {
    helloAt = now; sendFrame(WEB_CMD_HELLO);
  } else if (helloAccepted && (uint32_t)(now - heartbeatAt) >= 100U) {
    heartbeatAt = now; sendFrame(WEB_CMD_HEARTBEAT);
  }
  serviceTask(); servicePresets(); server.handleClient(); serviceUart(); delay(1);
}
