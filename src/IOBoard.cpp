#include "IOBoardRos.h"
#include "Atmega32u4Hardware.h"

#include "io_to_board.h"
#include "io_from_board.h"

// Include C headers (ie, non C++ headers) in this block
extern "C"
{
  #include <util/delay.h>
  #include <LUFA/Drivers/USB/USB.h>
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include <avr/sfr_defs.h>
}

// Needed for AVR to use virtual functions
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}

// Initialize ROS
ros::NodeHandle nh;
io_from_board out_msg;
ros::Publisher io_board_out("io_from_board", &out_msg);

// Variables for Actors
uint8_t servoMaske = 0;
uint8_t changedRakeBits = 0;
uint8_t relais_flags = 0;
uint32_t lastTimeMessage = 0;

// Variables for ADC Interrupts
uint32_t lastTimeInterruptRight = 0UL;
uint32_t lastTimeInterruptLeft = 0UL;

// Variables for Calibration of Odometry
uint8_t swap_status_motors = 0;
uint16_t idleLeft = 0;
uint16_t idleRight = 0;
uint16_t calibrationCountLeft = 0;
uint16_t calibrationCountRight = 0;
uint16_t calibrationSumLeft = 0;
uint16_t calibrationSumRight = 0;

void ioboard_cb(const io_to_board& io)
{
  // Store Time of last recieved message
  lastTimeMessage = avr_time_now();

  // Set digital output pins B0, B1, B2 and B3 as desired by message
  if ((io.status_charger   & 0x1) == 1) { PORTB |= 0x2; } else { PORTB &= 0xFD; }

  // Enable/Disable engines as desired by message
  if ((io.status_motors & 0x1) == 1)
  {
    PORTB |= 0x1;
    swap_status_motors = 1;
  }
  else
  {
    PORTB &= 0xFE;
  }
  
  // Set PWM outputs for motor velocity as desired by message
  if(idleRight != 0 && idleLeft != 0)
  {  
    if ((io.motor_left & 0xFF00) == 0x00)
    {
      OCR1A = 0x0100;
    }
    else if ((io.motor_left & 0xFF00) == 0xFF00)
    {

      OCR1A = 0xFF00;
    }
      else
    {
      OCR1A = io.motor_left;
    }
          
          
    if ((io.motor_right & 0xFF00) == 0x00)
    {
      OCR1B = 0x0100;
    }
    else if ((io.motor_right & 0xFF00) == 0xFF00)
    {
      OCR1B = 0xFF00;
    }
    else
    {
      OCR1B = io.motor_right;
    }

  }
  nh.loginfo("Setting values");
}


ISR(ADC_vect) {
  uint8_t temp_ADCL = ADCL;
  uint8_t temp_ADCH = ADCH;  
  uint16_t valueADC =  (temp_ADCH << 8) | temp_ADCL;

  switch (ADMUX) {
    case 0x0:
      if(idleLeft == 0 && swap_status_motors == 1)
      {
        if(calibrationCountLeft >= 80) { idleLeft = (uint16_t) (((float) calibrationSumLeft) / ((float) calibrationCountLeft) + 0.5); }
        else { calibrationSumLeft += valueADC; calibrationCountLeft++; }
      }

      else if (idleLeft != 0)
      {
        // 1028.3688 := 725mm (circumference of wheel) / 60 (seconds in a minute) / 47 (gear) * 4000 (maximum rpm)
        out_msg.deltaUmLeft += (int32_t) ((((double) valueADC) / ((double) idleLeft) * 1028.3688 - 1028.3688) * (double) (avr_time_now() - lastTimeInterruptLeft));
        lastTimeInterruptLeft = avr_time_now();
      }

      ADMUX = 0x1;
      break;


    case 0x1:
      if(idleRight == 0 && swap_status_motors == 1)
      {
        if(calibrationCountRight >= 80) { idleRight = (uint16_t) (((float) calibrationSumRight) / ((float) calibrationCountRight) + 0.5); }
        else { calibrationSumRight += valueADC; calibrationCountRight++; }
      }

      else if (idleRight != 0)
      {
        // 1028.3688 := 725mm (circumference of wheel) / 60 (seconds in a minute) / 47 (gear) * 4000 (maximum rpm)
        out_msg.deltaUmRight += (int32_t) ((((double) valueADC) / ((double) idleRight) * 1028.3688 - 1028.3688) * (double) (avr_time_now() - lastTimeInterruptRight));
        lastTimeInterruptRight = avr_time_now();
      }

      ADMUX = 0x4;
      break;


    case 0x4:
      // 30.3834 := 3970mV * 64.2kO / 8.2kO / 1023
      out_msg.mVBattery = (uint16_t) ((float) valueADC * 30.3834);
      ADMUX = 0x0;
      break;


    default:
      break;
  };

  ADCSRA |= (1 << ADSC);
}


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

int main()
{
  // Initialize ADC Interrupts
  lastTimeInterruptLeft = avr_time_now();
  lastTimeInterruptRight = avr_time_now();
  out_msg.deltaUmLeft = 0;
  out_msg.deltaUmRight = 0;
 

  // Initialize ROS
  nh.initNode(); 
  nh.advertise(io_board_out);
  ros::Subscriber<io_to_board> sub("to_ioboard", &ioboard_cb);
  nh.subscribe(sub);


  // Do timed/repeated stuff
  uint32_t lastTimeOdometry = 0UL;
  uint32_t lastTimeRake = 0UL;
  // PC 7 as output pin
  sbi(DDRC, PC7);
  // sbi(TCCR4B, CS42);    // set timer4 prescale factor to 64
  sbi(TCCR4B, CS41);
  sbi(TCCR4B, CS40);
  sbi(TCCR4D, WGM40);   // put timer 4 in phase- and frequency-correct PWM mode 
  sbi(TCCR4A, PWM4A);   // enable PWM mode for comparator OCR4A
  sbi(TCCR4A, COM4A1);   // enable PWM mode for comparator OCR4A
  sbi(TCCR4C, PWM4D);   // enable PWM mode for comparator OCR4D
  // sbi(TCCR4A, COM4A1);
  // sbi(TCCR4A, PWM4A);
  // TCCR4A = (1 << COM4A1) | (0 << COM4A0) | (1 << FOC4A) | (1 << PWM4A);
  OCR4C = 40;
  OCR4A = 0;
  unsigned char a = 0;

  while(1)
  {
    // Stop engines and raise rake, if last recieved message is older then 2s
    // %TODO raise rake
    if ((lastTimeMessage != 0) && (avr_time_now() - lastTimeMessage > 1000))
    { 
      lastTimeMessage = 0;

      OCR1A = 0x8000;
      OCR1B = 0x8000;
    }

    // Publish odometry all 40ms
    if (avr_time_now() - lastTimeOdometry > 40)
    {
      // set D6 to input
      out_msg.timestamp = avr_time_now();
      io_board_out.publish(&out_msg);
      out_msg.deltaUmLeft = 0;
      out_msg.deltaUmRight = 0;      
      lastTimeOdometry = avr_time_now();
      a++;
      if(a > 40) {
        a = 0;
      }
      OCR4A = a;
    }


    // Turn on TX LED
    DDRB |= (1 << DDB0);

    DDRD |= (0 << PD7);
    // Activate pull up resistor
    PORTD |= (1 << PD7);
    unsigned char i;
    i = PIND;
    if(i & (1 << PD7)) {
      PORTB |= (1 << PB0);
    } else {
      PORTB &= (0 << PB0);
    }


    nh.spinOnce();

    // LUFA functions that need to be called frequently to keep USB alive
    CDC_Device_USBTask(&Atmega32u4Hardware::VirtualSerial_CDC_Interface);
    USB_USBTask();
  }
  return 0;
}
