// Frame format: [timestamp, setpoint, speed, Kp, Ki, Kd, control_m, delimiter]
//   - Timestamp: 32 bits
//   - All others: 16 bits
//   - Delimiter: 8 bits (0xAA)

#include <Arduino.h>

#define SETPOINT_PIN  A0
#define SPEED_PIN     A1
#define KP_PIN        A2
#define KI_PIN        A3
#define KD_PIN        A4
#define CONTROL_M_PIN A5
#define CONTROL_C_PIN 2

#define SAMPLE_RATE_HZ 1000
volatile bool sample_flag = false;

inline uint16_t readADC(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

void sendUARTFrame(uint16_t setpoint, uint16_t speed,
                   uint16_t kp, uint16_t ki, uint16_t kd,
                   uint16_t control_m);

void setup() {
  Serial.begin(230400);
  pinMode(CONTROL_C_PIN, OUTPUT);

  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC enable, prescaler 128

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = (16 * 1000000 / 8 / SAMPLE_RATE_HZ) - 1;
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  sample_flag = true;
}

void loop() {
  if (sample_flag) {
    sample_flag = false;

    uint16_t setpoint   = readADC(0);
    uint16_t speed      = readADC(1);
    uint16_t kp         = readADC(2);
    uint16_t ki         = readADC(3);
    uint16_t kd         = readADC(4);
    uint16_t control_m  = readADC(5);

    sendUARTFrame(setpoint, speed, kp, ki, kd, control_m);
  }
}

void sendUARTFrame(uint16_t setpoint, uint16_t speed,
                   uint16_t kp, uint16_t ki, uint16_t kd,
                   uint16_t control_m) {
  uint32_t timestamp = micros();
  Serial.write((uint8_t*)&timestamp, 4);
  Serial.write((uint8_t*)&setpoint, 2);
  Serial.write((uint8_t*)&speed, 2);
  Serial.write((uint8_t*)&kp, 2);
  Serial.write((uint8_t*)&ki, 2);
  Serial.write((uint8_t*)&kd, 2);
  Serial.write((uint8_t*)&control_m, 2);
  Serial.write(0xAA);
}