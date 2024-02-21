/* CONTROL SYSTEM FOR FIREWOOD FURNACE WITH HEAT EXCHANGER */

/* LIBRARIES */
#include <TM1637Display.h> // for LED display
#include <OneButton.h>  // button press detection
// #include "max6675.h"    // temperature sensor module
#include <Wire.h>       // I2C communication
#include <AHTxx.h>      // AHT temperature sensor
#include <Servo.h>      // servo control 

/*INPUT PINS*/
// ROTARY ENCODER
#define ROT_SW 2    // this is an interrupt pin
#define ROT_DAT 3
#define ROT_CLK 4

/* OUTPUT PINS */
// LED DISPLAY
#define DISP_A_DAT 9
#define DISP_A_CLK 8
#define DISP_B_DAT 11
#define DISP_B_CLK 10

/* SET UP MODULES */
// Create a display object of type TM1637Display
TM1637Display disp_current = TM1637Display(DISP_A_CLK, DISP_A_DAT);
TM1637Display disp_set = TM1637Display(DISP_B_CLK, DISP_B_DAT);

// Create a module for button
OneButton button(ROT_SW, true, true); // input pullup activated and switch is active low

/* VARIABLES ARE DECLARED HERE */
bool rot_clk_present_state = true;
volatile bool rot_clk_last_state = true;
volatile bool menu_state = false;
volatile bool selected = false;

int menu_item = 0;
int prev_menu_item = -1;

int variable = 0;
int prev_variable = -1;

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

const uint8_t letter_p[] = {
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G
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

// for test purposes
int i=0;

void setup(){
    Serial.begin(9600);
    // setup output pins
    pinMode(DISP_A_CLK, OUTPUT);
    pinMode(DISP_A_DAT, OUTPUT);
    pinMode(DISP_B_CLK, OUTPUT);
    pinMode(DISP_B_DAT, OUTPUT);
    
    // setup input pins
    pinMode(ROT_CLK, INPUT);
    pinMode(ROT_DAT, INPUT);
    // pinMode(ROT_SW, INPUT);

    // Set the LED display brightness to 5 (0=dimmest 7=brightest)
	  disp_current.setBrightness(5);
    disp_set.setBrightness(5);
    // display dash on start up
	  disp_current.setSegments(dash);
    disp_set.setSegments(dash);
    delay(1000);
    //clear display
	  disp_current.clear();
    disp_set.clear();
    
    // attach long press function to button
    button.attachLongPressStop(long_press);
    // attach click function
    button.attachClick(on_click);
}

void loop(){
  // get current millis
  current_millis = millis();
  // button tick is run always to detect button press
  button.tick();

  // update the display at predefined interval
  if((prev_disp_time-current_millis)>disp_interval){
    disp_current.clear();
    disp_current.showNumberDec(i);
    i++;
    if(i>999)i=0;
    prev_disp_time = current_millis;
  }

  //get the menu time, so that to track the time before entering menu
  menu_timer = current_millis;
  prev_menu_item = -1;
  // MAIN MENU STARTS HERE
  while(menu_state){
    current_millis = millis();
    // run button ticks to track button press
    button.tick();
    // update the menu selection while turning rotary encoder knob
    menu_item = updateViaEncoder(menu_item, 0, 7);
    // if there is change in menu selection update the display
    if(prev_menu_item != menu_item){
      show_menu_option(menu_item);
      // reset the menu timer: if there is no change in menu items it remains the same
      menu_timer = current_millis;
      // update the previous menu item so that when it changes display get updated
      prev_menu_item = menu_item;
    }
    // if the menu remains inactive for a predefined time exit menu
    if((current_millis - menu_timer)>menu_max_time){
      menu_state = false;
    }
        
  }//END OF WHILE LOOP FOR MAIN MENU
  
    
}


/* FUNCTIONS */

// show menu selection 
void show_menu_option(int menu_item){
  disp_set.clear(); // clear the display
  disp_set.setSegments(letter_p, 1, 0); // load letter p
  disp_set.showNumberDec(menu_item+1, false, 1, 1); // load the menu number
}

// Function to be implemented after detecting long press
void long_press(){
    menu_state = !menu_state;
}

void on_click(){
    selected = !selected;
    // // save to eeprom only if clicked inside a menu
    // if(menu_state){
    //     write_EEPROM_after_check(menu_item, parameter_arr[menu_item]);
    // }
}

// check ticks 
void checkTicks(){
    button.tick();
}

// function to modify given menu_item using rotary encoder
int updateViaEncoder(int menu_item, int min, int max){
    // Read present state of the rotary encoder pin A
    rot_clk_present_state = digitalRead(ROT_CLK);
    // Check rotation status and increment or decrement based on direction
    if(rot_clk_present_state != rot_clk_last_state && rot_clk_present_state == true){
        if(digitalRead(ROT_DAT) == rot_clk_present_state){
            menu_item++;
        }else{
            menu_item--;
        }
        //Serial.println(count);
    }
    // remember the last state
    rot_clk_last_state = rot_clk_present_state;
    //    delay(1);
    // check if menu_item exceeding the limits
    (menu_item>max)?menu_item=max:menu_item;
    (menu_item<min)?menu_item=min:menu_item;

    return menu_item;
}

// // shiftout function for shift register
// void shiftOut(byte data){
//     bool d_out = false;
//     digitalWrite(SR_DAT, LOW);
//     digitalWrite(SR_CLK, LOW);

//     for(int i=7; i>=0; i--){
//         digitalWrite(SR_CLK, LOW);
//         if(data & (1<<i)){
//             d_out = true;
//         }else{
//             d_out = false;
//         }
//         digitalWrite(SR_DAT, d_out);
//         digitalWrite(SR_CLK, HIGH);

//         digitalWrite(SR_DAT, LOW);
//     }
//     digitalWrite(SR_CLK, LOW);
// }

// void speedSelect(int speed){
//     byte speed_arr[6] = {0b00000000, 0b00100000, 0b00010000,
//                         0b00001000, 0b00000100, 0b00000010};
//     digitalWrite(SR_LATCH, LOW);
//     shiftOut(speed_arr[speed]);
//     digitalWrite(SR_LATCH, HIGH);    
// }

// // eeprom write function
// void write_EEPROM(unsigned char addr, unsigned char data){
//   Wire.beginTransmission(eeprom);
//   Wire.write(addr);
//   Wire.write(data);
//   Wire.endTransmission();
// }

// // eprom read function
// byte read_EEPROM(unsigned char addr){
//   byte data;
//   Wire.beginTransmission(eeprom);
//   Wire.write(addr);
//   Wire.endTransmission();
//   delay(5);
//   Wire.requestFrom(0x50,1);
//   delay(5);
//   if(Wire.available()){
//     data = Wire.read();
//   }
//   return data;
// }

// // eeprom write function
// void write_EEPROM_after_check(unsigned char addr, unsigned char data){
//   byte read_data = read_EEPROM(addr);
//   if(data != read_data){
//     Wire.beginTransmission(eeprom);
//     Wire.write(addr);
//     Wire.write(data);
//     Wire.endTransmission();
//   }else{
//     return;
//   }
// }

// /* FUNCTION TO CALCULATE AVERAGE */
// float get_avg(float arr[]){
//   float sum = 0;
//   for (int i=0; i<10; i++){
//     sum+=arr[i];
//   }
//   return sum/10.0;
//  }

//  /* CONTROL BLOWER*/
//  void blower_control(bool state){
//     digitalWrite(BLOWER, state);
//  }

// /* CONTROL WASTEGATE VALVE */
// void wg_control(bool state){
//     wg_servo.attach(11);    // attach servo pin everytime
//     if(state){// if state is high open the wastegate
//        for(int i=0; i<=180; i++){
//         wg_servo.write(i);
//         delay(20);
//        }
//     }else if(state == LOW){
//         for(int i=180; i>=0; i--){
//         wg_servo.write(i);
//         delay(20);
//        }
//     }
//     // detach servo to avoid flickering movements
//     wg_servo.detach();
//     // set the pin to low
//     // digitalWrite(WASTE_GATE, LOW);
// }

// bool is_cabin_upper_limit(){
//     if((avg_cabin_temp - cabin_set_temp) > 0.1){
//         return true;
//     }
// }

// bool is_cabin_lower_limit(){
//     if(((cabin_set_temp - cabin_temp_hyster)-avg_cabin_temp) > 0.1  ){
//         return true;
//     }

// }

// bool is_he_upper_threshold(){
//     if((avg_he_temp - he_min_temp) > 0.1){
//         return true;
//     }
// }

// bool is_he_lower_threshold(){
//     if(((he_min_temp - he_hyster) - avg_he_temp) > 0.1 ){
//         return true;
//     }
// }

