#include <DHT.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>

//Pin K1 will turn the cooler off
//Pin K2 will reset the cooler if it is in the error mode
volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* pin_k = (unsigned char*) 0x106;

volatile unsigned char* port_d = (unsigned char*) 0x2B;
volatile unsigned char* ddr_d = (unsigned char*) 0x2A;
volatile unsigned char* pin_d = (unsigned char*) 0x29;

volatile unsigned char* port_f = (unsigned char*) 0x31;
volatile unsigned char* ddr_f = (unsigned char*) 0x30;
volatile unsigned char* pin_f = (unsigned char*) 0x2F;

volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109;


//UART for the printing things
#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//For stepper
#define STEPS 32
Stepper stepper(STEPS, 3, 5, 4, 6);
int stepperMult = 0;
int stepCounter = 0;
bool alreadyPressed = false;

//one minute timer
const unsigned long interval = 60000;
unsigned long previousMillis;

//Set up temp and humidity monitor
DHT dht(7, DHT11);
#define DHT11_PIN 7

//Set up LCD
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

RTC_DS1307 RTC;


//Bools to detect the state of the swamp cooler
bool disabled;
bool idle;
bool running;
bool error;

int prevState = 0;

bool takeMeasurements = true;

void setup() {
  Serial.begin(9600);
  disabled = true;
  idle = false;
  running = false;
  error = false;

  //Setting up button pins
  *ddr_k &= ~(0x01);
  *ddr_k &= ~(0x01 << 1);
  *ddr_k &= ~(0x01 << 2);
  *ddr_k &= ~(0x01 << 3);

  *ddr_d |= (0x01 << 2);
  attachInterrupt(digitalPinToInterrupt(2), start, HIGH);

  *ddr_f |= (0x01 << 1);
  *ddr_f |= (0x01 << 2);
  *ddr_f |= (0x01 << 3);
  *ddr_f |= (0x01 << 4);
  *port_f &= ~(0x01 << 1);
  *port_f &= ~(0x01 << 2);
  *port_f &= ~(0x01 << 3);
  *port_f &= ~(0x01 << 4);

  *ddr_l |= (0x01 << 5);
  *port_l &= ~(0x01 << 5);

  //Setting up ADC for water sensor
  U0init(9600);
  adc_init();
  previousMillis = millis();

  dht.begin();

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Bootin Up!");

  stepper.setSpeed(200);

  //For RTC
  PORTD |= (1 << PD0) | (1 << PD1);
  RTC.begin();
  DateTime now = DateTime(2025, 5, 9, 10, 48, 0);
  RTC.adjust(now);
}

void loop() {
  DateTime rightNow = RTC.now();
  *port_f &= ~(0x01 << 1);
  *port_f &= ~(0x01 << 2);
  *port_f &= ~(0x01 << 3);
  *port_f &= ~(0x01 << 4);

  if(disabled) {
    if(prevState != 1) {
      lcd.clear();
      lcd.print("Cooler Off");
      //analog write is only for fan motor
      analogWrite(44, 0);
    }
    prevState = 1;
  }

  if(!disabled) {
    if(*pin_k & (0x01 << 1)) {
      running = false;
      idle = false;
      error = false;
      disabled = true;
    }
  }

  if(running || idle) {
    unsigned long currMillis = millis();
    if(running) {
      //analog write is only for fan motor
      analogWrite(44, 255);
    }
    else {
      //analog write is only for fan motor
      analogWrite(44, 0);
    }
    
      previousMillis = currMillis;
      unsigned int threshold = 200;
      unsigned int value = adc_read(0);
      // read the water sensor value by calling adc_read() and check the threshold to display the message accordingly
      if(value < threshold) {
        running = false;
        idle = false;
        error = true;
      }

    //}
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    if(running) {
      if(temp < 18) {
        running = false;
        idle = true;
      }
    }
    else if(idle) {
      if(temp >= 18) {
        running = true;
        idle = false;
      }
    }

    if(prevState == 1 || prevState == 4) {
      lcd.clear();
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.setCursor(0, 0);
    }

    if(currMillis - previousMillis >= 60000) {
      previousMillis = currMillis;
      lcd.clear();
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.setCursor(0, 0);
    }

    if(*pin_k & (0x01 << 3)) {
      stepper.step(200);
    }

    if(running) {
      prevState = 3;
    }
    else if(idle) {
      prevState = 2;
    }
  }

  if(error) {
    if(*pin_k & (0x01 << 2) && adc_read(0) >= 200) {
      error = false;
      idle = true;
    }
    if(prevState != 4) {
      lcd.clear();
      lcd.print("Error: Water");
      lcd.setCursor(0, 1);
      lcd.print("level too low!");
      lcd.setCursor(0, 0);
      //analog write is only for fan motor
      analogWrite(44, 0);
    }
    prevState = 4;
  }

  int portNum;
  if(disabled) {
    portNum = 1;
  }
  else if(idle) {
    portNum = 2;
  }
  else if(error) {
    portNum = 3;
  }
  else if(running) {
    portNum = 4;
  }
  *port_f |= (0x01 << portNum);
}

void start() {
  if(disabled) {
    disabled = false;
    running = true;
    idle = false;
    error = false;
  }
}

void writeTime(DateTime now) {
  char nums[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hrs = now.hour();
  int mins = now.minute();
  int secs = now.second();

  U0putchar(nums[month/10 % 10]);
  U0putchar(nums[month % 10]);
  U0putchar('/');
  U0putchar(nums[day/10 % 10]);
  U0putchar(nums[day % 10]);
  U0putchar('/');
  U0putchar(nums[year/10 % 10]);
  U0putchar(nums[year % 10]);
  U0putchar('\n');

  U0putchar(nums[hrs/10 % 10]);
  U0putchar(nums[hrs % 10]);
  U0putchar(':');
  U0putchar(nums[mins/10 % 10]);
  U0putchar(nums[mins % 10]);
  U0putchar(':');
  U0putchar(nums[secs/10 % 10]);
  U0putchar(nums[secs % 10]);
  U0putchar('\n');
}

void adc_init() //write your code after each commented line and follow the instruction 
{
  // setup the A register
 // set bit   7 to 1 to enable the ADC
 *my_ADCSRA |= 0b10000000;

 // clear bit 6 to 0 to disable the ADC trigger mode
 *my_ADCSRA &= 0b10111111;

 // clear bit 5 to 0 to disable the ADC interrupt
 *my_ADCSRA &= 0b11011111;

 // clear bit 0-2 to 0 to set prescaler selection to slow reading
 *my_ADCSRA &= 0b11111000;

  // setup the B register
// clear bit 3 to 0 to reset the channel and gain bits
*my_ADCSRB &= 0b11110111;

 // clear bit 2-0 to 0 to set free running mode
 *my_ADCSRB &= 0b11111000;

  // setup the MUX Register
 // clear bit 7 to 0 for AVCC analog reference
 *my_ADMUX &= 0b01111111;

// set bit 6 to 1 for AVCC analog reference
*my_ADMUX |= 0b01000000;

  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;

 // clear bit 4-0 to 0 to reset the channel and gain bits
 *my_ADMUX &= 0b11110000;

}
unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
  // clear the channel selection bits (MUX 4:0)
 *my_ADMUX &= 0b11110000;
 

  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= 0b11110111;

 
  // set the channel selection bits for channel 0
  *my_ADMUX += adc_channel_num;

 

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;

  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  
  unsigned int val = (*my_ADC_DATA & 0x03FF);
  return val;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
