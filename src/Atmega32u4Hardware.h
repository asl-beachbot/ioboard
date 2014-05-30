#ifndef _ATMEGA32U4_HARDWARE_H
#define _ATMEGA32U4_HARDWARE_H

extern "C" {
  #include <avr/io.h>
  #include "Descriptors.h"
  #include "avr_time.h"
  #include <LUFA/Drivers/USB/USB.h>
}


class Atmega32u4Hardware {
  public:
    Atmega32u4Hardware() {}

    void init() {
      avr_time_init();
      USB_Init();

      DDRB |= 0xFF;	      // Use all B Pins for output (B56 PWM velocity, B7 PWM linear motor, B0 enable engines, B1-2 relays)
      DDRD |= 0xFF;           // Use all D Pins for digital output (D0123467 rake)
      DDRC |= 0b01000000;     // Use C6 for PWM linear motor

      ICR3 |= 0xFFFF;         // 16-Bit PWM
      ICR1 |= 0xFFFF;         // 16-Bit PWM

      TCCR1A = 0b10101010;    // Use OC1A, OC1B, OC1C for Fast PWM
      TCCR1B = 0b00011001;    // Fast PWM, no prescaling
      TCCR3A = 0b10000010;    // Use OC3A for Fast PWM
      TCCR3B = 0b00011001;    // Fast PWM, no prescaling

      OCR1A = 0x8000;         // Initialise all PWM signals
      OCR1B = 0x8000;
      OCR1C = 0x8000;
      OCR3A = 0x8000;

      PORTD = 0x00;           // Initialise all Pins with 0
      PORTB = 0x00;
      PORTC = 0x00;

      ADCSRA = 0b10001111;    // Init ADC-Interrupts
      ADMUX  = 0x0;

      sei(); 
      ADCSRA |= 1<<ADSC;
    }

    int read() {
      return CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    }

    void write(uint8_t* data, int length) {
      CDC_Device_SendData(&VirtualSerial_CDC_Interface, (char *)data, (uint16_t)length);
    }

    unsigned long time() {
      return avr_time_now();
    }

    static USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface;
};

void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

#endif
