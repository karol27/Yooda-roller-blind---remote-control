#include "yooda_protocol.h"

namespace esphome {
namespace yooda_protocol {

// Zakładane czasy nominalne w mikrosekundach (µs).
// Ponieważ 1 ms = 1000 µs, np. 0.32 ms to 320 µs, 6.28 ms to 6280 µs itp.
static const uint32_t SYNC_PULSE_MIN_US = 4500;  // dolna tolerancja impulsu synchronizacji
static const uint32_t SYNC_PULSE_MAX_US = 7000;  // górna tolerancja impulsu synchronizacji
// bit "0": 0.32 ms (high) + 0.76 ms (low)
// bit "1": 0.72 ms (high) + 0.36 ms (low)
static const uint32_t BIT0_HIGH_MIN_US = 200; 
static const uint32_t BIT0_HIGH_MAX_US = 450;   // np. tolerancja ok. 0.32 ms +/- 
static const uint32_t BIT0_LOW_MIN_US  = 600; 
static const uint32_t BIT0_LOW_MAX_US  = 950;   // np. tolerancja ok. 0.76 ms +/- 

static const uint32_t BIT1_HIGH_MIN_US = 550; 
static const uint32_t BIT1_HIGH_MAX_US = 900;   // np. tolerancja ok. 0.72 ms +/- 
static const uint32_t BIT1_LOW_MIN_US  = 200; 
static const uint32_t BIT1_LOW_MAX_US  = 550;   // np. tolerancja ok. 0.36 ms +/- 

// ----------------------------------------------------------------------------
// ESPHome lifecycle
// ----------------------------------------------------------------------------

void YoodaProtocol::setup() {
  // W razie potrzeby można tu zainicjalizować zmienne, logi itp.
  ESP_LOGI("yooda_protocol", "YoodaProtocol setup completed");
}

void YoodaProtocol::on_receive(const remote_receiver::RemoteReceiveData &data) {
  // Wektor przechowuje poszczególne impulsy (high/low) zarejestrowane przez remote_receiver
  const auto &items = data.items;

  uint32_t pilot_id = 0;
  uint8_t roller_id = 0;
  YoodaCommand command = YoodaCommand::UNKNOWN;

  // Dekodujemy ramkę
  if (this->decode_yooda_frame(items, pilot_id, roller_id, command)) {
    // Udało się zdekodować
    const char *cmd_str = "UNKNOWN";
    switch (command) {
      case YoodaCommand::UP:    cmd_str = "UP"; break;
      case YoodaCommand::DOWN:  cmd_str = "DOWN"; break;
      case YoodaCommand::STOP:  cmd_str = "STOP"; break;
      default: break;
    }

    ESP_LOGD("yooda_protocol", 
      "Odebrano poprawny kod YOODA -> Pilot ID: 0x%07X, Roleta: %u, Komenda: %s", 
      pilot_id, roller_id, cmd_str);

    // Tutaj można np. wywołać callback, wysłać zdarzenie do Home Assistant itp.
    // Np. publish state, ustawić switch, zaktualizować encję itp.
  }
}

// ----------------------------------------------------------------------------
// Dekodowanie ramki
// ----------------------------------------------------------------------------

bool YoodaProtocol::decode_yooda_frame(const std::vector<remote_receiver::RemoteReceiveDataItem> &items,
                                       uint32_t &pilot_id, uint8_t &roller_id, YoodaCommand &command) {
  // Minimalna liczba impulsów do dekodowania: 
  // - 1 impuls synchro (2 "półimpulsy": high+low) 
  // - 40 bitów, każdy z nich to 2 "półimpulsy" (high + low)
  // Czyli oczekujemy co najmniej 1*2 + 40*2 = 82 impulsów w wektorze
  if (items.size() < 82) {
    return false;
  }

  // Sprawdzamy impuls synchronizacji na początku:
  // sync = ~6.28 ms = 6280 µs (z tolerancją)
  uint32_t first_pulse_length = items[0].duration;  // "high"
  uint32_t second_pulse_length = items[1].duration; // "low"

  if ((first_pulse_length + second_pulse_length) < SYNC_PULSE_MIN_US ||
      (first_pulse_length + second_pulse_length) > SYNC_PULSE_MAX_US) {
    // brak prawidłowego impulsu synchronizacji na początku
    return false;
  }

  // Dalej od pozycji 2 zaczynają się bity (40 bitów po 2 impulsy).
  int bit_index = 0;
  uint64_t raw_data = 0;  // tu przechowamy wszystkie 40 bitów

  // Chcemy odczytać 40 bitów = 80 impulsów (każdy bit to "high" i "low")
  for (int i = 0; i < 40; i++) {
    int idx = 2 + i * 2; 
    int bit_val = parse_bit(items[idx], items[idx + 1]);
    if (bit_val < 0) {
      // nie udało się zinterpretować bitu -> błąd
      return false;
    }
    // Doklejamy bit do raw_data od lewej do prawej (starszy -> młodszy)
    raw_data = (raw_data << 1) | bit_val;
    bit_index++;
  }

  // Mamy 40 bitów w raw_data.
  // Zgodnie ze specyfikacją:
  //  - 28 bitów ID pilota
  //  - 4 bity numer rolety
  //  - 8 bitów komendy
  // Bity od najstarszego do najmłodszego (bit 39 -> bit 0).

  // Najpierw wyciągamy ID (28 bitów)
  uint64_t mask_28 = ((uint64_t)1 << 28) - 1;  // np. 0x0FFFFFFF
  uint64_t id_28 = (raw_data >> (40 - 28)) & mask_28; // pobieramy górne 28 bitów
  pilot_id = static_cast<uint32_t>(id_28);

  // Kolejne 4 bity to ID rolet
  uint64_t mask_4 = ((uint64_t)1 << 4) - 1; // 0xF
  uint64_t roller_4 = (raw_data >> (40 - 28 - 4)) & mask_4;
  roller_id = static_cast<uint8_t>(roller_4);

  // Ostatnie 8 bitów to komenda
  uint64_t mask_8 = 0xFF;  // 8 bitów
  uint64_t cmd_8 = raw_data & mask_8; 
  command = command_from_byte(static_cast<uint8_t>(cmd_8));

  return true;
}

// ----------------------------------------------------------------------------
// Interpretacja pojedynczego bitu
// ----------------------------------------------------------------------------

int YoodaProtocol::parse_bit(const remote_receiver::RemoteReceiveDataItem &high, 
                             const remote_receiver::RemoteReceiveDataItem &low) {
  uint32_t high_us = high.duration;
  uint32_t low_us  = low.duration;

  // Sprawdzamy, czy parametry pasują do bitu "0"
  bool bit0_high_ok = (high_us >= BIT0_HIGH_MIN_US && high_us <= BIT0_HIGH_MAX_US);
  bool bit0_low_ok  = (low_us  >= BIT0_LOW_MIN_US  && low_us  <= BIT0_LOW_MAX_US);

  if (bit0_high_ok && bit0_low_ok) {
    return 0;
  }

  // Sprawdzamy, czy parametry pasują do bitu "1"
  bool bit1_high_ok = (high_us >= BIT1_HIGH_MIN_US && high_us <= BIT1_HIGH_MAX_US);
  bool bit1_low_ok  = (low_us  >= BIT1_LOW_MIN_US  && low_us  <= BIT1_LOW_MAX_US);

  if (bit1_high_ok && bit1_low_ok) {
    return 1;
  }

  // Jeśli ani "0" ani "1" nie pasuje, zwracamy -1 (błąd)
  return -1;
}

// ----------------------------------------------------------------------------
// Rozpoznawanie komendy
// ----------------------------------------------------------------------------

YoodaCommand YoodaProtocol::command_from_byte(uint8_t cmd) {
  // Zgodnie z informacją:
  //  UP   = 0001 0001 (0x11)
  //  STOP = 0101 0101 (0x55)
  //  DOWN = 0011 0011 (0x33)
  switch (cmd) {
    case 0x11: return YoodaCommand::UP;
    case 0x33: return YoodaCommand::DOWN;
    case 0x55: return YoodaCommand::STOP;
    default:   return YoodaCommand::UNKNOWN;
  }
}

}  // namespace yooda_protocol
}  // namespace esphome
