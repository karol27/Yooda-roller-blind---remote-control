#pragma once

#include "esphome.h"

namespace esphome {
namespace yooda_protocol {

/// Enum przedstawiający możliwe komendy odczytane z pilota.
enum class YoodaCommand : uint8_t {
  UP,
  DOWN,
  STOP,
  UNKNOWN
};

class YoodaProtocol : public Component, public remote_receiver::RemoteReceiverListener {
 public:
  // Funkcja wywoływana przez ESPHome przy inicjalizacji komponentu.
  void setup() override;

  // Funkcja wywoływana automatycznie dla każdego odebranego sygnału (lista impulsów).
  void on_receive(const remote_receiver::RemoteReceiveData &data) override;

 protected:
  /// Główna funkcja dekodująca sekwencję impulsów na ID pilota, numer rolety i komendę.
  /// Zwraca true, jeśli udało się poprawnie zdekodować ramkę Yooda.
  bool decode_yooda_frame(const std::vector<remote_receiver::RemoteReceiveDataItem> &items,
                          uint32_t &pilot_id, uint8_t &roller_id, YoodaCommand &command);

  /// Funkcja pomocnicza do interpretacji pojedynczego bitu (0/1) na bazie długości impulsów.
  /// Zwraca -1, jeśli nie udało się jednoznacznie ustalić bitu.
  int parse_bit(const remote_receiver::RemoteReceiveDataItem &high, 
                const remote_receiver::RemoteReceiveDataItem &low);

  /// Funkcja pomocnicza: zamienia 8-bitowy kod rozkazu na enum YoodaCommand.
  YoodaCommand command_from_byte(uint8_t cmd);
};

}  // namespace yooda_protocol
}  // namespace esphome
