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

      DDRB |= 0b01100000;     // Use B5-6 for 16-Bit PWM
      ICR1 = 0xFFFF;
      TCCR1A = 0b10100000;
      TCCR1B = 0b00010001;
      OCR1A = 0x8000;
      OCR1B = 0x8000;
      DDRD = 0xFF;            // Use all D Pins for digital output
      PORTD = 0x00;           // Initialise all D Pins with 0

      // Init Registers
      ADCSRA |= (1<<ADPS2);
      ADMUX  |= (1<<REFS1);
      ADMUX  |= (1<<REFS0);
      ADCSRA = 0;
      ADCSRA |= 1<<ADIE;
      ADCSRA |= 1<<ADEN;

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
