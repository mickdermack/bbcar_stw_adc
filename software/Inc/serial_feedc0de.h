#pragma once

#ifdef __cplusplus
#include <cstdint>
#include "protocol.h"
class SerialFeedc0de
{
public:
  void send_feedback(const Feedback& in);
  bool tx_done();
private:
  Feedback feedback;
};

#endif
