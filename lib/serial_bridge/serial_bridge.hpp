#pragma once

#include "SerialTransfer.h"
#include "packets.h"

class SerialBridge {
  public:
    SerialBridge() : transfer_() {}

    void begin(Stream& _port) {
      transfer_.begin(_port);
    }

    void send(const SensorPacket& pkt) {
      uint16_t sendSize = 0;
      sendSize = transfer_.txObj(pkt, sendSize);
      transfer_.sendData(sendSize, SENSOR);
    }

    void send(const WatchdogPacket& pkt) {
      uint16_t sendSize = 0;
      sendSize = transfer_.txObj(pkt, sendSize);
      transfer_.sendData(sendSize, WATCHDOG);
    }

  private:
    SerialTransfer transfer_;
};
