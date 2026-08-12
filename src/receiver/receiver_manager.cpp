#include "receiver/receiver_manager.h"

uint16_t linkScore(const LinkQuality& lq, uint8_t priority) {
  if (!lq.valid || lq.failsafe) {
    return 0;
  }
  uint32_t score = (uint32_t)lq.lq_percent * 6 + (uint32_t)lq.rssi_percent * 3;
  if (priority == 0) {
    score += 100;
  }
  if (score > 1000) {
    score = 1000;
  }
  return (uint16_t)score;
}

ReceiverManager::ReceiverManager(ReceiverPort& port_a, ReceiverPort& port_b)
    : state_(SelectionState::NO_LINK),
      active_index_(-1),
      active_since_ms_(0),
      challenger_index_(-1),
      challenger_better_since_ms_(0),
      switch_count_(0),
      last_switch_ms_(0) {
  ports_[0] = &port_a;
  ports_[1] = &port_b;
  rcFrameInit(empty_frame_);
  linkQualityInit(empty_lq_);
}

void ReceiverManager::begin(const SelectionConfig& cfg) {
  cfg_ = cfg;
  state_ = SelectionState::NO_LINK;
  active_index_ = -1;
  active_since_ms_ = 0;
  challenger_index_ = -1;
  challenger_better_since_ms_ = 0;
  switch_count_ = 0;
  last_switch_ms_ = 0;
}

void ReceiverManager::setConfig(const SelectionConfig& cfg) {
  cfg_ = cfg;
}

void ReceiverManager::doSwitch(int8_t idx, uint32_t now_ms) {
  active_index_ = idx;
  active_since_ms_ = now_ms;
  switch_count_++;
  last_switch_ms_ = now_ms;
  state_ = SelectionState::SWITCHING;
}

void ReceiverManager::update(uint32_t now_ms) {
  ports_[0]->update(now_ms);
  ports_[1]->update(now_ms);

  bool alive[RECEIVER_PORT_COUNT];
  alive[0] = ports_[0]->isAlive(now_ms, cfg_.link_timeout_ms);
  alive[1] = ports_[1]->isAlive(now_ms, cfg_.link_timeout_ms);

  if (active_index_ < 0) {
    int8_t best = -1;
    uint16_t best_score = 0;
    for (uint8_t i = 0; i < RECEIVER_PORT_COUNT; ++i) {
      if (!alive[i]) continue;
      uint16_t s = linkScore(ports_[i]->getLinkQuality(), ports_[i]->priority());
      if (best < 0 || s > best_score) {
        best = (int8_t)i;
        best_score = s;
      }
    }
    if (best < 0) {
      state_ = SelectionState::NO_LINK;
      return;
    }
    active_index_ = best;
    active_since_ms_ = now_ms;
    challenger_index_ = -1;
    challenger_better_since_ms_ = 0;
    state_ = SelectionState::LOCKED;
    return;
  }

  if (!alive[active_index_]) {
    int8_t other = (int8_t)(1 - active_index_);
    challenger_index_ = -1;
    challenger_better_since_ms_ = 0;
    if (alive[other]) {
      // Dead active: switch immediately, ignoring min_active_time_ms.
      doSwitch(other, now_ms);
    } else {
      state_ = SelectionState::NO_LINK;
      active_index_ = -1;
    }
    return;
  }

  int8_t other = (int8_t)(1 - active_index_);
  if (!alive[other]) {
    challenger_index_ = -1;
    challenger_better_since_ms_ = 0;
    state_ = SelectionState::LOCKED;
    return;
  }

  uint16_t active_score = linkScore(ports_[active_index_]->getLinkQuality(),
                                     ports_[active_index_]->priority());
  uint16_t other_score = linkScore(ports_[other]->getLinkQuality(), ports_[other]->priority());

  uint32_t margin_needed = ((uint32_t)active_score * cfg_.hysteresis_percent) / 100;
  bool better = (uint32_t)other_score >= (uint32_t)active_score + margin_needed;

  if (!better) {
    challenger_index_ = -1;
    challenger_better_since_ms_ = 0;
    state_ = SelectionState::LOCKED;
    return;
  }

  if (challenger_index_ != other) {
    challenger_index_ = other;
    challenger_better_since_ms_ = now_ms;
  }
  state_ = SelectionState::EVALUATING;

  bool switch_delay_elapsed = (now_ms - challenger_better_since_ms_) >= cfg_.switch_delay_ms;
  bool min_active_elapsed = (now_ms - active_since_ms_) >= cfg_.min_active_time_ms;

  if (switch_delay_elapsed && min_active_elapsed) {
    doSwitch(other, now_ms);
    challenger_index_ = -1;
    challenger_better_since_ms_ = 0;
  }
}

int8_t ReceiverManager::activeIndex() const {
  return active_index_;
}

const RCFrame& ReceiverManager::activeFrame() const {
  if (active_index_ < 0) return empty_frame_;
  return ports_[active_index_]->getFrame();
}

const LinkQuality& ReceiverManager::activeLink() const {
  if (active_index_ < 0) return empty_lq_;
  return ports_[active_index_]->getLinkQuality();
}

bool ReceiverManager::hasValidLink() const {
  return active_index_ >= 0;
}

ReceiverPort* ReceiverManager::activePort() {
  if (active_index_ < 0) return nullptr;
  return ports_[active_index_];
}

SelectionState ReceiverManager::state() const {
  return state_;
}

uint32_t ReceiverManager::switchCount() const {
  return switch_count_;
}

uint32_t ReceiverManager::lastSwitchMs() const {
  return last_switch_ms_;
}

uint16_t ReceiverManager::scoreOf(uint8_t index) const {
  if (index >= RECEIVER_PORT_COUNT) return 0;
  return linkScore(ports_[index]->getLinkQuality(), ports_[index]->priority());
}
