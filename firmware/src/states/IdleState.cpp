#include "states/IdleState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void IdleState::enter() {
  lastUpdateMs_ = 0;
  lastActivityMs_ = millis();
  ledController.setSolid(Config::LED_IDLE_COLOR);
}

bool IdleState::handleCalibrationRequest() {
  if (inputController.takeCalibrationRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
    return true;
  }
  return false;
}

void IdleState::runMotionPipeline(float dt, unsigned long now) {
  float raw[9] = {};
  if (!sensorController.readRaw(raw)) {
    return;  // Skip frame on sensor read failure
  }

  float motion[6] = {};
  motionController.compute(raw, sensorController.baseline(), dt, motion);

  if (motionController.hasMotionActivity()) {
    lastActivityMs_ = now;
  }

  const uint16_t buttonBits = inputController.buttonBits();
  const bool hidReportSent = hidController.sendReports(motion, buttonBits);
  if (telemetryController.enabled()) {
    telemetryController.publish(motion, buttonBits, hidReportSent);
  }
}

void IdleState::handleSleepTransition(unsigned long now) {
  const unsigned long inactiveMs = now - lastActivityMs_;
  if (inactiveMs >= Config::IDLE_SLEEP_TIMEOUT_MS) {
    stateMachine.changeState(&StateMachine::sleepState);
  }
}

void IdleState::update() {
  inputController.update();

  if (handleCalibrationRequest()) {
    return;
  }

  const unsigned long now = millis();
  if (inputController.takeActivity()) {
    lastActivityMs_ = now;
  }

  const float dt = (lastUpdateMs_ == 0) ? 0.01
                                        : ((now - lastUpdateMs_) / 1000.0);
  lastUpdateMs_ = now;
  runMotionPipeline(dt, now);
  handleSleepTransition(now);
}

void IdleState::exit() {}
