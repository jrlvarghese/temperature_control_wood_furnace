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
volatile bool selected = false;

int value = 0;
int prev_value = -1;

bool troubleshoot = false;

unsigned long current_millis = 0;
// time tracker for serial print
unsigned long prev_serial_time = 0;
unsigned long serial_interval = 1000;
// time tracker for led display
unsigned long prev_disp_time = 0;
unsigned long disp_interval = 2000;
// time tracker for sensors
unsigned long prev_sense_time = 0;
unsigned long sense_interval = 1000;
// time tracker for menu
unsigned long menu_max_time = 30000;
unsigned long menu_timer = 0;
// time tracker for submenu
unsigned long submenu_max_time = 10000;
unsigned long submenu_timer = 0;

// reading variable
float he_temp_read = 0.0;
float cabin_temp_read = 0.0;
float cabin_humid_read = 0.0;

// variables for counters
int disp_count = 0;
/* SET UP MODULES */
// Create a display object of type TM1637Display
TM1637Display display = TM1637Display(LED_CLK, LED_DAT);
// Create a MAX6675 object
MAX6675 he_temp(HE_SCK, HE_CS, HE_SO);
bool max_sensor_status = false;

OneButton button(ROT_SW, true);

float ahtValue;                               //to store T/RH result

AHTxx aht20(AHTXX_ADDRESS_X38, AHT2x_SENSOR); //sensor address, sensor type
bool aht_sensor_status = false;

/* NUM/LETTER ARRAY FOR LED DISPLAY */
// dash line on startup
const uint8_t dash[] = {
  SEG_G, SEG_G, SEG_G, SEG_G
};
// Create an array that turns all segments ON
const uint8_t allON[] = {0xff, 0xff, 0xff, 0xff};

// Create an array that turns all segments OFF
const uint8_t allOFF[] = {0x00, 0x00, 0x00, 0x00};

// Create an array that sets individual segments per digit to display the word "dOnE"
const uint8_t done[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};

// // Create degree celsius symbol
// const uint8_t celsius[] = {
//   SEG_A | SEG_B | SEG_F | SEG_G,  // Degree symbol
//   SEG_A | SEG_D | SEG_E | SEG_F   // C
// };

const uint8_t celsius[] = {
  SEG_D | SEG_E | SEG_F | SEG_A
};

const uint8_t humid[] = {
  SEG_B | SEG_C | SEG_E | SEG_F | SEG_G
};

const uint8_t temp[] = {
  SEG_D | SEG_E | SEG_F | SEG_G
};

// const uint8_t heat[] = {
//   SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
//   SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,
//   SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
//   SEG_D | SEG_E | SEG_F | SEG_G
// };

const uint8_t heat[] = {
  SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G
};

const uint8_t fail[] = {
  SEG_A | SEG_E | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
  SEG_E | SEG_F,
  SEG_E | SEG_F | SEG_D
};

const uint8_t he[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t arr[2][1] = {{temp},{humid}};



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

    // attach long press function to button
    button.attachLongPressStop(long_press);
    // attach click function
    button.attachClick(on_click);

    if(aht20.begin() != true){
        Serial.println(F("AHT2x not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
        delay(1000);
        aht_sensor_status = false;
    }else{
        aht_sensor_status = true;
    }
    delay(500);
    // create a wire object
    Wire.begin();

    write_EEPROM_after_check(1, 128);

}

void loop(){
    // get current millis
    current_millis = millis();
    // button tick is run always to detect button press
    button.tick();
    // delay(10);
    
    while(menu_state){// main menu
        current_millis = millis();
        // button tick is run always to detect button press
        button.tick();

        value = updateViaEncoder(value, 0, 8);
        if(prev_value != value){
            display.clear();
            display.showNumberDec(value, false, 1, 0);
        }
        prev_value = value;
        if(troubleshoot&&((millis() - prev_serial_time)>=serial_interval)){
            Serial.println("Inside menu");
            prev_serial_time = millis();
            Serial.println(value);
        }
    }

    // update led display on regular intervals
    if(current_millis- prev_disp_time > disp_interval){
        disp_count>2?disp_count = 0:disp_count;
        // display readings only if sensor is working
        if(aht_sensor_status){
            if(disp_count==0){
                display.clear();
                if((int(he_temp_read)/100) == 0){
                    display.showNumberDec(int(cabin_temp_read), false, 2, 0);
                }else{
                    display.showNumberDec(int(cabin_temp_read), false, 3, 0);
                }
                display.setSegments(celsius, 1, 3);
            }
            if(disp_count==1){
                display.clear();
                // display.showNumberDec(setHumidity, false, 2, 0);
                display.showNumberDec(int(cabin_humid_read), false, 2, 0);
                display.setSegments(humid, 1, 3);
            }
            if(disp_count==2){
                display.clear();
                if((int(he_temp_read)/100) == 0){
                    display.showNumberDec(int(he_temp_read), false, 2, 0);
                }else{
                    display.showNumberDec(int(he_temp_read), false, 3, 0);
                }
                display.setSegments(he, 1, 3);
            }
        }else{// if sensor is not connected or failed
            display.clear();
            display.setSegments(fail,4,0);
        }
        // increment display counter
        disp_count++;
        prev_disp_time = current_millis;
    }

    // get temperature readings in regular intervals
    if((current_millis - prev_sense_time)>sense_interval){
        prev_sense_time = current_millis;
        he_temp_read = he_temp.readCelsius();
        cabin_temp_read = aht20.readTemperature();
        cabin_humid_read = aht20.readHumidity();
    }

    if(troubleshoot && ((current_millis - prev_serial_time)>=serial_interval)){
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

void on_click(){
    selected = !selected;
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

// eeprom write function
void write_EEPROM_after_check(unsigned char addr, unsigned char data){
  byte read_data = read_EEPROM(addr);
  if(data != read_data){
    Wire.beginTransmission(eeprom);
    Wire.write(addr);
    Wire.write(data);
    Wire.endTransmission();
  }else{
    return;
  }
}