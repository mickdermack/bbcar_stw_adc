#include "serial_esp.h"
#include "can.h"

#include "serial_feedc0de.h"

void SerialFeedc0de::send_feedback(const Feedback& in)
{
  char test = 0;
  DEBUG_U8(4, test);

  feedback = in;

  feedback.start = Feedback::VALID_HEADER;
  feedback.checksum = calculateChecksum(feedback);

  if (serial_esp_tx_ready())
    serial_esp_tx((uint8_t*)&feedback, sizeof(feedback));
}
