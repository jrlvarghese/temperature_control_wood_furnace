/* CONTROL SYSTEM FOR FIREWOOD FURNACE WITH HEAT EXCHANGER */

/* LIBRARIES */
#include <TM1637Display.h> // for LED display
#include <OneButton.h>  // button press detection
#include "max6675.h"    // temperature sensor module
#include <Wire.h>       // I2C communication
#include <AHTxx.h>      // AHT temperature sensor

/*INPUT PINS*/
// ROTARY ENCODER
#define ROT_SW 2    // this is an interrupt pin
#define ROT_DAT 3
#define ROT_CLK 4

//HE TEMP SENSOR
#define HE_SO 7
#define HE_CS 8
#define HE_SCK 9

/* OUTPUT PINS */
// LED DISPLAY
#define LED_DAT 5
#define LED_CLK 6

// SHIFT REGISTER FOR CONTROLING BLOWER SPEED
#define SR_DAT 13
#define SR_CLK 10
#define SR_LATCH 12

// EEPROM ADDRESS
#define eeprom 0x50

/* VARIABLES ARE DECLARED HERE */
bool rot_clk_present_state = true;
volatile bool rot_clk_last_state = true;
volatile bool menu_state = false;

int value = 0;
int prev_value = -1;

bool troubleshoot = false;

unsigned long prev_serial_time = 0;
unsigned long serial_time = 1000;

/* SET UP MODULES */
// Create a display object of type TM1637Display
TM1637Display display = TM1637Display(LED_CLK, LED_DAT);
// Create a MAX6675 object
MAX6675 he_temp(HE_SCK, HE_CS, HE_SO);

OneButton button(ROT_SW, true);

float ahtValue;                               //to store T/RH result

AHTxx aht20(AHTXX_ADDRESS_X38, AHT2x_SENSOR); //sensor address, sensor type

/* NUM/LETTER ARRAY FOR LED DISPLAY */
// dash line on startup
const uint8_t dash[] = {
  SEG_G, SEG_G, SEG_G, SEG_G
};

void setup(){
    Serial.begin(9600);
    troubleshoot = true;
    // setup output pins
    pinMode(LED_CLK, OUTPUT);
    pinMode(LED_DAT, OUTPUT);

    pinMode(SR_CLK, OUTPUT);
    pinMode(SR_DAT, OUTPUT);
    pinMode(SR_LATCH, OUTPUT);

    // setup input pins
    pinMode(ROT_CLK, INPUT);
    pinMode(ROT_DAT, INPUT);
    pinMode(ROT_SW, INPUT);

    // Set the LED display brightness to 5 (0=dimmest 7=brightest)
	display.setBrightness(4);
    // display dash on start up
	display.setSegments(dash);
    delay(1000);
    //clear display
	display.clear();

    // attachInterrupt(digitalPinToInterrupt(ROT_SW),checkTicks,FALLING);
    button.attachLongPressStop(long_press);

    while(aht20.begin() != true){
        Serial.println(F("AHT2x not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
        delay(5000);
    }
    delay(500);
    // create a wire object
    Wire.begin();

    write_EEPROM(1, 128);

}

void loop(){
    // display.setSegments(dash);
    // delay(1000);
    button.tick();
    // delay(10);
    
    while(menu_state){
        button.tick();
        value = updateViaEncoder(value, 0, 20);
        if(prev_value != value){
            Serial.println(value);
        }
        prev_value = value;
        if(troubleshoot&&((millis() - prev_serial_time)>=serial_time)){
            Serial.println("Inside menu");
            prev_serial_time = millis();
        }
    }

    if(troubleshoot && ((millis() - prev_serial_time)>=serial_time)){
        prev_serial_time = millis();
        Serial.print("C = "); 
        Serial.println(he_temp.readCelsius());
        Serial.print("AHT C = ");
        Serial.println(aht20.readTemperature());
        Serial.print("AHT H = ");
        Serial.println(aht20.readHumidity());
        Serial.println(read_EEPROM(1));
    }
}


/* FUNCTIONS */

// Function to be implemented after detecting long press
void long_press(){
    menu_state = !menu_state;
}

// check ticks 
void checkTicks(){
    button.tick();
}

// function to modify given value using rotary encoder
int updateViaEncoder(int value, int min, int max){
    // Read present state of the rotary encoder pin A
    rot_clk_present_state = digitalRead(ROT_CLK);
    // Check rotation status and increment or decrement based on direction
    if(rot_clk_present_state != rot_clk_last_state && rot_clk_present_state == true){
        if(digitalRead(ROT_DAT) == rot_clk_present_state){
            value++;
        }else{
            value--;
        }
        //Serial.println(count);
    }
    // remember the last state
    rot_clk_last_state = rot_clk_present_state;
    //    delay(1);
    // check if value exceeding the limits
    (value>max)?value=max:value;
    (value<min)?value=min:value;

    return value;
}

// shiftout function for shift register
void shiftOut(byte data){
    bool d_out = false;
    digitalWrite(SR_DAT, LOW);
    digitalWrite(SR_CLK, LOW);

    for(int i=7; i>=0; i--){
        digitalWrite(SR_CLK, LOW);
        if(data & (1<<i)){
            d_out = true;
        }else{
            d_out = false;
        }
        digitalWrite(SR_DAT, d_out);
        digitalWrite(SR_CLK, HIGH);

        digitalWrite(SR_DAT, LOW);
    }
    digitalWrite(SR_CLK, LOW);
}

void speedSelect(int speed){
    byte speed_arr[6] = {0b00000000, 0b00100000, 0b00010000,
                        0b00001000, 0b00000100, 0b00000010};
    digitalWrite(SR_LATCH, LOW);
    shiftOut(speed_arr[speed]);
    digitalWrite(SR_LATCH, HIGH);    
}

// eeprom write function
void write_EEPROM(unsigned char addr, unsigned char data){
  Wire.beginTransmission(eeprom);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

// eprom read function
byte read_EEPROM(unsigned char addr){
  byte data;
  Wire.beginTransmission(eeprom);
  Wire.write(addr);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom(0x50,1);
  delay(5);
  if(Wire.available()){
    data = Wire.read();
  }
  return data;
}