#pragma once
#include <stdint.h>
#include "receiver/receiver_port.h"
#include "config/config_types.h"
#include "protocols/protocol_types.h"

enum class SelectionState : uint8_t { NO_LINK = 0, LOCKED = 1, EVALUATING = 2, SWITCHING = 3 };

// Weighting rationale: link quality (freshness/frame integrity) counts 2x as much as
// raw RSSI because dropped frames hurt control more directly than a modest signal
// deficit. The flat +100 bonus for the priority-0 (primary) port only breaks
// near-ties — it can never overcome a large, genuine quality gap on the other port
// (max lq+rssi contribution is 900). A dead or failsafed link always scores 0.
// score = lq_percent*6 + rssi_percent*3 + (priority == 0 ? 100 : 0), capped at 1000.
uint16_t linkScore(const LinkQuality& lq, uint8_t priority);

class ReceiverManager {
 public:
  ReceiverManager(ReceiverPort& port_a, ReceiverPort& port_b);

  void begin(const SelectionConfig& cfg);
  void setConfig(const SelectionConfig& cfg);
  void update(uint32_t now_ms);

  int8_t activeIndex() const;
  const RCFrame& activeFrame() const;
  const LinkQuality& activeLink() const;
  bool hasValidLink() const;
  ReceiverPort* activePort();
  SelectionState state() const;
  uint32_t switchCount() const;
  uint32_t lastSwitchMs() const;
  uint16_t scoreOf(uint8_t index) const;

 private:
  void doSwitch(int8_t idx, uint32_t now_ms);

  ReceiverPort* ports_[RECEIVER_PORT_COUNT];
  SelectionConfig cfg_;
  SelectionState state_;

  int8_t active_index_;
  uint32_t active_since_ms_;

  int8_t challenger_index_;
  uint32_t challenger_better_since_ms_;

  uint32_t switch_count_;
  uint32_t last_switch_ms_;

  RCFrame empty_frame_;
  LinkQuality empty_lq_;
};
