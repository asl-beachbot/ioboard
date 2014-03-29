#include "ros.h"
#include "std_msgs/String.h"
#include "io_to_board.h"
#include "io_from_board.h"
#include "Atmega32u4Hardware.h"
#define puls_oben 1200
#define puls_unten 1600

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

ros::NodeHandle nh;

// std_msgs::String info_msg;

io_from_board out_msg;

// typedef io_to_board IOMessageToBoard;

ros::Publisher io_board_out("io_from_board", &out_msg);
// ros::Publisher io_board_info("io_info", &info_msg);

uint8_t servoMaske = 0;

bool cb(0);

void ioboard_cb(const io_to_board& io)
{
  if (io.rake_flags != servoMaske)
  {
    servoMaske = (io.rake_flags | 0x80);
    cli();

    PORTD |= ~servoMaske;                // alle 0 in io.rake_flags stehen für ein gehobenes Rechenteil
    _delay_us(puls_oben);
    PORTD &= servoMaske;
    _delay_us(20000 - puls_oben);
    PORTD |= ~servoMaske;
    _delay_us(puls_oben);
    PORTD &= servoMaske;
    _delay_us(20000 - puls_oben);
    PORTD |= ~servoMaske;
    _delay_us(puls_oben);
    PORTD &= servoMaske;
    _delay_us(20000 - puls_oben);
    PORTD |= ~servoMaske; 
    _delay_us(puls_oben);
    PORTD &= servoMaske;

    sei();
    servoMaske = (io.rake_flags & 0x7F);        // oberstes Bit von PORTD darf hier nicht geändert werden
    cli();

    PORTD |= servoMaske;                    // alle 1 in io.rake_flags stehen für ein gesenktes Rechenteil
    _delay_us(puls_unten);
    PORTD &= ~servoMaske;
    _delay_us(20000 - puls_unten);
    PORTD |= servoMaske;
    _delay_us(puls_unten);
    PORTD &= ~servoMaske;
    _delay_us(20000 - puls_unten);
    PORTD |= servoMaske;
    _delay_us(puls_unten);
    PORTD &= ~servoMaske;
    _delay_us(20000 - puls_unten);
    PORTD |= servoMaske;
    _delay_us(puls_unten);
    PORTD &= ~servoMaske;

    sei();
  }


  if ((io.status & 0x1) == 1)
  {
    PORTD |= 0x80;
  }
  else
  {
    PORTD &= 0x7F;
  }
  
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
    OCR1AH = (io.motor_left & 0xFF00) >> 8;
    OCR1AL = (io.motor_left & 0x00FF);
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
    OCR1BH = (io.motor_right & 0xFF00) >> 8;
    OCR1BL = (io.motor_right & 0x00FF);
  }
}


volatile int16_t temp_output[2] = {-1, -1};

uint8_t temp_ADCL = 0;
uint8_t temp_ADCH = 0;
uint8_t last_controlled_motor = 2; // 2 = left, 3 = rigth

ISR(ADC_vect) {
//  ADCSRA |= (1 << ADSC);
//  while(bit_is_set(ADCSRA, ADSC));
  temp_ADCL = ADCL;
  temp_ADCH = ADCH;
  out_msg.velocity =  (temp_ADCH << 8) | temp_ADCL;
  //bswap
  //out_msg.velocity = __builtin_bswap16(out_msg.velocity);
  switch (ADMUX) {
    case 0xC0:
      ADMUX = 0xC1;
      out_msg.status = 0x02;
      break;
    case 0xC1:
      out_msg.status = 0x03;
      ADMUX = 0xC0;
      break;
    default:
      break;
  };
  ADCSRA |= (1 << ADSC);

  // if(!(i % 100000)) {
  //   io_board_info.publish(&info_msg);
  // }
}

int main()
{

  uint32_t lasttime = 0UL;


  // Initialize ROS
  nh.initNode();
  
  nh.advertise(io_board_out);
  // nh.advertise(io_board_info);
  
  ros::Subscriber<io_to_board> sub("to_ioboard", &ioboard_cb);
  nh.subscribe(sub);

  int i = 0;
  while(1)
  {
    i++;
    // Send the message every second
    if(avr_time_now() - lasttime > 10)
    {
      io_board_out.publish(&out_msg);

      // out_msg.motor_right = temp_output[0];
      // out_msg.motor_left  = temp_output[1];

      // io_board_out.publish(&out_msg);
      // io_board_info.publish(&info_msg);

      lasttime = avr_time_now();
    }
    nh.spinOnce();
    // LUFA functions that need to be called frequently to keep USB alive
    CDC_Device_USBTask(&Atmega32u4Hardware::VirtualSerial_CDC_Interface);
    USB_USBTask();
  }

  return 0;
}
