#include "esphome.h"

class YoodaProtocol : public Component, public RemoteReceiverCallBack {
public:
  void setup() override {
    // Zarejestruj dekoder
    remote_receiver->register_callback(this);
  ESP_LOGD("YoodaProtocol", "Dekoder został zarejestrowany.");
  }

  void on_receive(const RemoteReceiveData &data) override {
    // Minimalna liczba impulsów (synchronizacja + co najmniej 5 bitów)
    if (data.raw.len < 6) {
    ESP_LOGW("YoodaProtocol", "Odebrano mniej niż 5 bitów – sygnał zbyt krótki.");
    return;
    }

    // const int SYNC_PULSE = 6280; // 6.28 ms              my measures
    const int SYNC_HIGH = 4760; // 4.76 ms
    const int SYNC_LOW = 1520;  // 1.52 ms                  1566
    const int BIT_0_LOW = 320;  // 0.32 ms                  360
    const int BIT_0_HIGH = 760; // 0.76 ms                  753
    const int BIT_1_LOW = 720;  // 0.72 ms                  730
    const int BIT_1_HIGH = 360; // 0.36 ms                  389
    const int REPEAT_DELAY = 10; // 10 ms przerwy między transmisjami
    
    // Sprawdź pierwszy impuls jako synchronizacyjny
    // if (!data.matches_at(0, SYNC_PULSE, tolerance::TOLERANCE_PERCENT)) return;
    // Sprawdź stan wysoki i niski dla synchronizacji
    if (!data.matches_at(0, SYNC_HIGH, tolerance::TOLERANCE_PERCENT) || 
      !data.matches_at(1, SYNC_LOW, tolerance::TOLERANCE_PERCENT)) {
    ESP_LOGW("YoodaProtocol", "Błąd synchronizacji – niepoprawny sygnał synchronizacji.");
    return;
  }
    
    
    // Odbieramy kod bit po bicie
    uint64_t full_code = 0;
    int received_bits = 0;

    for (int i = 1; i < data.raw.len / 2 && received_bits < 40; i++) {
    int idx = (i - 1) * 2 + 1;  // Indeks impulsów w danych
    if (data.matches_at(idx, BIT_0_LOW, tolerance::TOLERANCE_PERCENT) &&
      data.matches_at(idx + 1, BIT_0_HIGH, tolerance::TOLERANCE_PERCENT)) {
      full_code = (full_code << 1);  // Bit 0
      received_bits++;
    } else if (data.matches_at(idx, BIT_1_LOW, tolerance::TOLERANCE_PERCENT) &&
           data.matches_at(idx + 1, BIT_1_HIGH, tolerance::TOLERANCE_PERCENT)) {
      full_code = (full_code << 1) | 1;  // Bit 1
      received_bits++;
    } else {
      break;  // Niezgodny sygnał
    }
    }

    // Loguj, jeśli kod jest niekompletny, ale zawiera więcej niż 5 bitów
    if (received_bits > 5 && received_bits < 40) {
    ESP_LOGW("YoodaProtocol", "Odebrano %d bitów, ale sygnał jest niekompletny.", received_bits);
    return;
    }

    // Jeśli kod jest kompletny
    if (received_bits == 40) {
    ESP_LOGD("YoodaProtocol", "Pełny kod: 0x%010llX", full_code);

    // Podział na części
    uint32_t pilot_code = (full_code >> 12) & 0x0FFFFFFF;  // Pierwsze 28 bitów
    uint8_t roller_number = (full_code >> 8) & 0x0F;       // Kolejne 4 bity
    uint8_t command = full_code & 0xFF;                    // Ostatnie 8 bitów

    // Wyświetlenie poszczególnych części
    ESP_LOGD("YoodaProtocol", "Kod pilota: 0x%08X", pilot_code);
    ESP_LOGD("YoodaProtocol", "Numer rolety: %u", roller_number);
    ESP_LOGD("YoodaProtocol", "Komenda: 0x%02X", command);

    // Obsługa komendy
    if (command == 0x11) {
      ESP_LOGD("YoodaProtocol", "Komenda: UP");
      // Wykonaj akcję "up"
    } else if (command == 0x55) {
      ESP_LOGD("YoodaProtocol", "Komenda: STOP");
      // Wykonaj akcję "stop"
    } else if (command == 0x33) {
      ESP_LOGD("YoodaProtocol", "Komenda: DOWN");
      // Wykonaj akcję "down"
    } else {
      ESP_LOGD("YoodaProtocol", "Nieznana komenda");
    }
    } else {
    ESP_LOGW("YoodaProtocol", "Odebrano zbyt mało bitów, aby rozpoznać kod.");
    }
    
    // Zaktualizowanie zmiennych globalnych w ESPHome
    id(pilot_code) = pilot_code;
    id(roller_number) = roller_number;
    id(command) = command;
    
  }
  
};


// // implementacja w esphome:
// external_components:
  // - source: custom
    
// remote_receiver:
  // pin: GPIOxx # Twój pin odbiornika
  // dump: custom

// logger:
  // level: DEBUG
  
// // /path/to/project/
// // ├── my_device.yaml
// // ├── custom_components/
// // │   └── yooda_protocol/
// // │       └── yooda_protocol.cpp

// // Dodanie zmiennnych globalnychq
// globals:
  // - id: pilot_code
    // type: int
    // initial_value: '0'
  // - id: roller_number
    // type: int
    // initial_value: '0'
  // - id: command
    // type: int
    // initial_value: '0'

class YoodaTransmitter : public Component {
public:   
  void send_code(uint32_t pilot_code, uint8_t roller_number, const std::string& command, int repeat_count) {
    
    // Mapowanie komend na wartości binarne
    uint8_t command_code;
    if (command == "UP") {
      command_code = 0x11;  // Kod dla UP
    } else if (command == "DOWN") {
      command_code = 0x33; // Kod dla DOWN
    } else if (command == "STOP") {
      command_code = 0x55; // Kod dla STOP
    } else {
      ESP_LOGW("YoodaTransmitter", "Nieznana komenda: %s", command.c_str());
      return; // Nieznana komenda, wyjdź z funkcji
    }
    
    // Składanie pełnego kodu
    uint64_t full_code = ((uint64_t)pilot_code << 12) | ((uint64_t)roller_number << 8) | command;

    // Konfiguracja parametrów czasowych protokołu
    const int SYNC_HIGH = 4760; // 4.76 ms
    const int SYNC_LOW = 1520;  // 1.52 ms
    const int BIT_0_LOW = 320;  // 0.32 ms
    const int BIT_0_HIGH = 760; // 0.76 ms
    const int BIT_1_LOW = 720;  // 0.72 ms
    const int BIT_1_HIGH = 360; // 0.36 ms
    const int REPEAT_DELAY = 10; // 10 ms przerwy między transmisjami

    for (int repeat = 0; repeat < repeat_count; repeat++) { // Wysłanie sygnału `repeat_count` razy
      // Wysłanie sygnału synchronizacji
      remote_transmitter->send_raw(SYNC_HIGH, true); // Stan wysoki
      remote_transmitter->send_raw(SYNC_LOW, false); // Stan niski

      // Wysłanie 40 bitów
      for (int i = 39; i >= 0; i--) {
        bool bit = (full_code >> i) & 0x1;
        if (bit) {
          remote_transmitter->send_raw(BIT_1_LOW, true);
          remote_transmitter->send_raw(BIT_1_HIGH, false);
        } else {
          remote_transmitter->send_raw(BIT_0_LOW, true);
          remote_transmitter->send_raw(BIT_0_HIGH, false);
        }
      }

      // Przerwa między powtórzeniami, ale nie po ostatnim
      if (repeat < repeat_count - 1) {
        delay(REPEAT_DELAY); // Czekaj 10 ms
      }
    }
    ESP_LOGD("YoodaTransmitter", "Wysłano komendę: %s", command.c_str());
  }
};