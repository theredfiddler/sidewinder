#include <Arduino.h>
#include <SPI.h>
// #include <MIDI.h>
#include "FastLED.h"
#include <iostream>
// #include <Adafruit_TinyUSB.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>


#define NUM_STRIPS 4
#define NUM_LEDS_PER_STRIP 140

#define NUM_RING_LEDS (NUM_STRIPS * NUM_LEDS_PER_STRIP)
#define NUM_LEDS_PER_RING 28
#define NUM_KNOBS 20

#define LED_RING_DATA_PIN_1 39
#define LED_RING_DATA_PIN_2 40
#define LED_RING_DATA_PIN_3 41
#define LED_RING_DATA_PIN_4 42

#define LED_BUTTONS_DATA_PIN 46

#define MIDI_TX_PIN 17

#define NUM_KNOB_LEDS 20

const int latchPin = 45; 

bool bits[128] = {0};

int ring_led_reorder[NUM_RING_LEDS] = {0};

int active_sequence = 0;

CRGB ring_leds_array[NUM_RING_LEDS];
CRGB button_leds_array[15];

const int button_input_pins[15] = {1, 2, 3, 4, 5, 6, 7, 10, 11, 14, 15, 16, 26, 47, 33};

bool button_last_state[15] = {0};
bool button_pushed_flag[15] = {0};
bool button_released_flag[15]= {0};
bool zoom_complete = true;

int button_millis_last_pushed[15] = {0};


enum button_display_modes {
      NORMAL,
      QUANTIZER,
};

button_display_modes button_display_mode;

enum zoom_modes {
  ZOOMED_OUT,
  ZOOMED_IN
};

zoom_modes current_zoom_mode;
zoom_modes zoom_mode_when_button_pressed;

bool quant[12] = {1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1};
uint8_t quant_button_address[12] = {6, 9, 5, 10, 4, 3, 12, 2, 13, 1, 14, 0};


uint8_t pitch_quantize(uint8_t input){
      if (input > 121){ //if input is above 121, we want to only count down so it doesn't go over 127
            for (int i = 0; i < 6; i++){
                  
                  if (input + i <= 127){
                        if (quant[(input + i) % 12] == true){
                        return(input + i);
                        }
                  }
                  if (input > 127){
                  input = 127;
                  }
                  if (quant[(input - i) % 12] == true){
                        return(input - i);
                  }
            }
            return input;
      }

      if (input < 6){ //if input is below 6, we want to start at 6
            for (int i = 0; i < 6; i++){
                        if (quant[input+ i] == true){
                        return(input + i);
                        }
                        if (input >= i){
                        if (quant[input - i] == true){
                              return(input -i);
                        }
                        }
            }
            return input;          
      }


      for (int i=0; i<6; i++){ //for everything else, count up and down
            if (quant[(input + i) % 12] == true){
                  return (input + i);
            }
            if (quant[(input - i) % 12] == true){
                  return (input - i);
            }
                  
      }


      return input;
}




uint8_t scale(int input, uint8_t scaling_value){

      int scale_multiplier = (scaling_value / 128) * .5;

      return 64 + ((input - 64) * (scale_multiplier));

}

int remap_16_to_20(int address16){
      return address16 + (address16/4);
}
int remap_20_to_16(int address20){
      return address20 - ((address20)/5);
}

void remap_leds(){
      for (int i = 0; i < NUM_RING_LEDS; i++){
            ring_led_reorder[i] = i;
      };
}



// USB MIDI object
Adafruit_USBD_MIDI gabe_usb_midi;
// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, gabe_usb_midi, MIDI_GABE_USB);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI_GABE_DIN);


TaskHandle_t SPI_input_task;


int8_t knobIncremented[20] = {0};
bool knob_last_pushed_state[20] = {0};


int encoderCounter[20] = {0};
int8_t lrmem[20] = {3};
int8_t lrsum[20] = {0};


int8_t rotaryChange(int8_t l, int8_t r, int8_t knobIndex)
{
static int8_t TRANS[] = {0,-1,1,14,1,0,14,-1,-1,14,0,1,14,1,-1,0};


lrmem[knobIndex] = ((lrmem[knobIndex] & 0x03) << 2) + 2*l + r;
lrsum[knobIndex] = lrsum[knobIndex] + TRANS[lrmem[knobIndex]];


/* encoder not in the neutral state */
if(lrsum[knobIndex] % 4 != 0) return(0);


/* encoder in the neutral state */

if (lrsum[knobIndex] == 4)
{
lrsum[knobIndex] = 0;
return(1);
}
if (lrsum[knobIndex] == -4)
{
lrsum[knobIndex] = 0;
return(-1);
}
/* lrsum > 0 if the impossible transition */
lrsum[knobIndex] = 0;
return(0);
}


double bpm = 100; // BPM as a floating-point value
double lastBeatTime = 0.0;
uint8_t knob_to_light_up = 0;

uint8_t last_knob_played = 19;


uint8_t SPI_encoder_address[20] = {0};
uint8_t SPI_encoder_push_address[20] = {0};

uint8_t SPI_button_address[15] = {0};

bool is_button_currently_pushed[16] = {0};

uint16_t buttons_currently_pushed = 0;

void set_knob_value_color(uint8_t ring_address,uint8_t hue){
for (int i = 0; i <= NUM_LEDS_PER_RING; i++){
      ring_leds_array[(ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)])].setHue(hue);
}
}
void light_up_ring_base(int ring_address){
for(int i = 0; i < 3; i++){
}
}
bool beat_flag = false;

bool ready_for_beat() {
      int current_time_in_milliseconds = millis();
      int quarter_note_time_length = (60.0 / bpm) * 1000;
      int sixteenth_note_time_length = quarter_note_time_length / 4;
      // Check for zero BPM to avoid division by zero
      if (bpm == 0) {
      return false;
      }

      if (current_time_in_milliseconds - lastBeatTime >= sixteenth_note_time_length) {

      lastBeatTime = current_time_in_milliseconds;
      beat_flag = true;
      // Serial.println("Ready for Beat!");
      return true;
}
return false;
}


int crop(int input, int lowest, int highest){
if (input < lowest){
input = lowest;
}
if (input > highest){
input = highest;
}
return input;
}


class KnobClass {
public:

bool is_currently_pushed;

uint8_t dot_brightness;
uint8_t rgb_color;
uint8_t rgb_brightness;

bool last_pushed_state;
bool pushed_flag;
bool released_flag;

};


class SeqClass {
public:
uint8_t active_step;
uint8_t next_step;
bool is_assigned;
uint8_t sequence_color;
uint8_t last_pitch_played;
uint8_t step_counter;
uint8_t channel;

uint8_t value[16];
uint8_t velocity[16];
uint8_t gate_length[16];
uint8_t range[16];
uint8_t step_length[16];

double last_note_time;
uint8_t speed; //the time that one step takes up in a sequence


uint8_t offset; //this tweaks where the beginning step is in a sequence
uint8_t sequence_length; // determines the end step in the sequence that will trigger a repeat
uint8_t number_of_steps; // this is defined by the number of active steps within the sequence length. it is destructively modified when steps within the sequence length are added manually, and when altered, it adds the most spaced out step (or takes away).
SeqClass(){
      active_step = 0;
      last_pitch_played = 0;
      number_of_steps = 16;
      step_counter = 0;
      sequence_color = 20;
      // Initialize all array members
      std::fill_n(value, 16, 64);
      std::fill_n(velocity, 16, 64);
      std::fill_n(gate_length, 16, 64);
      std::fill_n(range, 16, 127);
      std::fill_n(step_length, 16, 1); // Default step length to 1
}


};

KnobClass Knob[20];


SeqClass Sequence[20];




uint8_t any_other_knob_pushed(uint8_t ring_address){
for (int i = 0; i < 20; i++){
      if (Knob[i].is_currently_pushed && (i != ring_address)){
            return i;
      }
}
return 200;
}


void clear_ring_leds(uint8_t ring_address){
for (int i = 0; i < 28; i++){
      ring_leds_array[ring_address*NUM_LEDS_PER_RING + i] = CRGB::Black;
}
}


void illuminate_ring_solid(uint8_t ring_address, uint8_t knob_value, uint8_t hue){
clear_ring_leds(ring_address);
for (int i = 1; i < (knob_value * .142 + 1); i++){
      ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].setHue(hue);
      ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].nscale8(255);
}
}   

void illuminate_ring_white_out(uint8_t ring_address, uint8_t knob_value, uint8_t hue, uint8_t whiteness){
clear_ring_leds(ring_address);
for (int i = 1; i < (knob_value * .142 + 1); i++){
      ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].setHSV(hue, 255 - whiteness, 255);
      // ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].nscale8(255);
}
}  

void illuminate_ring_solid_white(uint8_t ring_address, uint8_t knob_value, uint8_t intensity){
clear_ring_leds(ring_address);
for (int i = 1; i < (knob_value * .142 + 1); i++){
      ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]] = CRGB::White;
      ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].nscale8(intensity);
}
}  

void illuminate_knob_indicator(uint8_t ring_address, uint8_t hue){
      for (int i = 21; i <= 24; i++){
            ring_leds_array[ring_led_reorder[NUM_LEDS_PER_RING * ring_address + i]].setHue(hue);
            ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].nscale8(100);
      }
}



void illuminate_ring_half_bright(uint8_t ring_address, uint8_t knob_value, uint8_t hue){
clear_ring_leds(ring_address);
for (int i = 1; i < (knob_value * .142 + 1); i++){
      ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].setHue(hue);
      ring_leds_array[ring_led_reorder[((NUM_LEDS_PER_RING * ring_address) + i)]].nscale8(100);
}
}
void display_active_sequence(){
      for (int i = 0; i < 16; i++){
            if (Sequence[active_sequence].active_step == i){
                  illuminate_ring_solid(remap_16_to_20(i), Sequence[active_sequence].value[i], Sequence[active_sequence].sequence_color);
                  illuminate_knob_indicator(remap_16_to_20(i), Sequence[active_sequence].sequence_color + 20);
            }
            else {
                  illuminate_ring_half_bright(remap_16_to_20(i), Sequence[active_sequence].value[i], Sequence[active_sequence].sequence_color);
                  illuminate_knob_indicator(remap_16_to_20(i), Sequence[active_sequence].sequence_color);
            }
      }
      for (int i = 0; i < 4; i++){
            clear_ring_leds(i*5 + 4);
      }

}
void display_all_sequences(){
      
      
      for (int i = 0; i < 20; i++){


            int active_step = Sequence[i].active_step;
            int active_step_value = Sequence[i].value[active_step];
            int sequence_color = Sequence[i].sequence_color;
            illuminate_ring_solid(i, active_step_value, sequence_color);
            
            double current_time = millis();
            double time_since_last_step = current_time - Sequence[i].last_note_time;

            int fading_whiteness = (100 - time_since_last_step*(2*bpm/80));
            if (fading_whiteness < 0){
                  fading_whiteness = 0;
            }
      
  
                  illuminate_ring_white_out(i,  active_step_value, sequence_color, fading_whiteness);

            illuminate_knob_indicator(i, sequence_color);
      }
}

void play_next_note_in_sequence(uint8_t seq_id){
      uint8_t* active_step = &(Sequence[seq_id].active_step);
      uint8_t* next_step = &(Sequence[seq_id].next_step);
      uint8_t* step_counter = &(Sequence[seq_id].step_counter);
      uint8_t* last_pitch_played = &(Sequence[seq_id].last_pitch_played);
      double* last_note_time = &(Sequence[seq_id].last_note_time);
      uint8_t active_step_length = Sequence[seq_id].step_length[(Sequence[seq_id].active_step)];
      uint8_t seq_length = Sequence[seq_id].number_of_steps;

      *active_step = *next_step;

      if(*step_counter == 0){
            MIDI_GABE_DIN.sendNoteOff(*last_pitch_played, 100, 1);
            MIDI_GABE_USB.sendNoteOff(*last_pitch_played, 100, 1);

            uint8_t pitch_to_play = pitch_quantize( 64 + (Sequence[seq_id].value[*active_step]-64) / 7);
            uint8_t veloc_to_play = 64 + (Sequence[seq_id].velocity[*active_step] - 64) / 7;

            MIDI_GABE_DIN.sendNoteOn(pitch_to_play, veloc_to_play, 1);
            MIDI_GABE_USB.sendNoteOn(pitch_to_play, veloc_to_play, 1);

            *last_note_time = millis();
            // Serial.print("playing note");

            *last_pitch_played = pitch_to_play;
      }

      *step_counter +=1;

      if (*step_counter >= active_step_length){
            *step_counter = 0;
            *next_step = (*active_step + 1) % seq_length;
      }

}


void SPI_initial_calibration(){     //The RGB knobs have different resting states,
                              //some are zero, some are 1. This function detects
                              // each encoder's resting state in the setup so the rotaryChange
                              // function works correctly in the main loop
      digitalWrite(latchPin, LOW);
      delayMicroseconds(10);
      digitalWrite(latchPin, HIGH);


      // Step 2: Get the bits array via SPI


      uint16_t data = SPI.transfer16(0);
      uint16_t data2 = SPI.transfer16(0);  // Transfer 16 bits from the 74HC165
      uint16_t data3 = SPI.transfer16(0);
      uint16_t data4 = SPI.transfer16(0);
      uint16_t data5 = SPI.transfer16(0);

      for (int i = 15; i >= 0; --i) {
            bits[i] = data & 1;
            data >>= 1;
      }

      for (int i = 31; i >= 16; --i) {
            bits[i] = data2 & 1;
            data2 >>= 1;
      }
      
      for (int i = 47; i >= 32; --i) {
            bits[i] = data3 & 1;
            data3 >>= 1;
      }
      
      for (int i = 63; i >= 48; --i) {
            bits[i] = (data4 & 1);
            data4 >>= 1;
      }

      for (int i = 79; i >= 64; --i) {
            bits[i] = (data5 & 1);
            data5 >>= 1;
      }
      
      // for (int i = 0; i <= 79; i++){ //Print that shit
      //       Serial.print(bits[i]);
      // }
      // Serial.println();


      for (int i = 0; i < 5; i++){ //calibrate the encoders every 4 incoders
            SPI_encoder_address[i] = (i * 16) + 0;
            SPI_encoder_address[i + 5] = (i * 16) + 3;
            SPI_encoder_address[i + 10] = (i * 16) + 8;
            SPI_encoder_address[i + 15] = (i * 16) + 11;


            SPI_encoder_push_address[i] = (i * 16) + 2;
            SPI_encoder_push_address[i + 5] = (i * 16) + 5;
            SPI_encoder_push_address[i + 10] = (i * 16) + 10;
            SPI_encoder_push_address[i + 15] = (i * 16) + 13;


      }


      // for (int i = 0; i < 5; i++){
      //       SPI_button_address[3 * i] = 12 + (16 * i);
      //       SPI_button_address[3 * i + 1] = 13 + (16 * i);
      //       SPI_button_address[3 * i + 2] = 14 + (16 * i);
      // }
}

void print_when_pressed(){
for (int i = 0; i < 20; i++){
      if (Knob[i].pushed_flag = true){
            Serial.print(" Knob " + String(i) + " has been pushed, removing flag now");
            Knob[i].pushed_flag = false;
      }
      if (Knob[i].released_flag = true){
            Serial.print(" Knob " + String(i) + " has been released, removing flag now");
            Knob[i].pushed_flag = false;
      }           
}
}


void SPI_input_gather(void * parameter){

for(;;){
      if(millis() > 1000 && millis() < 1520){
            SPI_initial_calibration();
      }
      // Step 1: Sample
      digitalWrite(latchPin, LOW);
      delayMicroseconds(50);
      digitalWrite(latchPin, HIGH);


      // Step 2: Get the bits array via SPI
      uint16_t data = SPI.transfer16(0);  // Transfer 16 bits from the 74HC165
      uint16_t data2 = SPI.transfer16(0); // Transfer 16 bits from the 74HC165
      uint16_t data3 = SPI.transfer16(0); // Transfer 16 bits from the 74HC165
      uint16_t data4 = SPI.transfer16(0); // Transfer 16 bits from the 74HC165
      uint16_t data5 = SPI.transfer16(0); // Transfer 16 bits from the 74HC165


      for (int i = 15; i >= 0; --i) {
            bits[i] = data & 1;
            data >>= 1;
      }


      for (int i = 31; i >= 16; --i) {
            bits[i] = data2 & 1;
            data2 >>= 1;
      }
      
      for (int i = 47; i >= 32; --i) {
            bits[i] = data3 & 1;
            data3 >>= 1;
      }
      
      for (int i = 63; i >= 48; --i) {
            bits[i] = (data4 & 1);
            data4 >>= 1;
      }


      for (int i = 79; i >= 64; --i) {
            bits[i] = (data5 & 1);
            data5 >>= 1;
      }


      // for (int i = 0; i <= 79; i++){ ////// PRINT SERIAL OUTPUT
      //       if (i%4 == 0){
      //             Serial.print("  ");
      //       }
      //       Serial.print(bits[i]);
      // }
      // Serial.println();
            
      for (int i = 0; i < 20; i++){ //checking all 20 knobs, setting state of whether they are being turned right, left, or being pushed
            uint8_t address = SPI_encoder_address[i];

            knobIncremented[i] = rotaryChange(bits[address]^1, bits[address + 1]^1, i);
            
            Knob[i].is_currently_pushed = bits[SPI_encoder_push_address[i]]^1;
            
            if (Knob[i].is_currently_pushed != Knob[i].last_pushed_state){
                  if (Knob[i].is_currently_pushed){
                        Knob[i].pushed_flag = true;
                        Serial.println("knob " + String(i) + " was just pushed");
                  }
                  else{
                        Knob[i].released_flag = true;
                        Serial.println("knob " + String(i) + " was just released");
                  }
            }
            Knob[i].last_pushed_state = Knob[i].is_currently_pushed;
      }
      
      for (int i = 0; i < 20; i++){ //checking the seq knobs 1-16, ignoring the side knobs
            if (knobIncremented[i] != 0){
                  int grid_address = remap_20_to_16(i);
                  if (current_zoom_mode == ZOOMED_OUT){
                        if (digitalRead(button_input_pins[0]) == LOW){
                              zoom_complete = false;
                              active_sequence = i;
                        }
                        else{
                              for (int d = 0; d < 16; d ++){
                                    if (Sequence[i].value[d] + knobIncremented[i] > 127){
                                          Sequence[i].value[d] = 127;
                                    }
                                    if (Sequence[i].value[d] + knobIncremented[i] * 7 < 0){
                                          Sequence[i].value[d] = 1;
                                    }
                                    else {
                                          Sequence[i].value[d] += 7 * knobIncremented[i]; 
                                    }
                              }
                              active_sequence = i;                              
                        }


                  }
                  if (current_zoom_mode == ZOOMED_IN){
                        if (zoom_complete == false){
                              break;
                        }

                        if (Sequence[active_sequence].value[grid_address] + knobIncremented[i] > 127){
                              break;
                        }
                        if (Sequence[active_sequence].value[grid_address] + knobIncremented[i] * 7 < 0){
                              break;
                        }
                        Sequence[active_sequence].value[grid_address] += 7 * knobIncremented[i];
                              // Serial.println("knob of step" + String(grid_address) + "in sequence" + 
                              //       String(active_sequence) + " is now" + String(Sequence[active_sequence].value[grid_address]));
                        
                        if(Sequence[active_sequence].value[grid_address] > 127){
                              Sequence[active_sequence].value[grid_address] = 127;
                        }
                        if(Sequence[active_sequence].value[grid_address] < 1){
                              Sequence[active_sequence].value[grid_address] = 0;
                        }
                  }
            }
      }
}
}



void display_buttons(){
      if (button_display_mode == NORMAL){
            for (int i = 0; i<15; i++){
                  if (digitalRead(button_input_pins[i])== LOW){
                        button_leds_array[i] = CRGB::White;
                  }
                  else button_leds_array[i].setHue(Sequence[active_sequence].sequence_color);
            }
      }
      
}



void set_button_flags() {
    for (int i = 0; i < 15; i++) {
        int currentState = digitalRead(button_input_pins[i]);

        if (currentState != button_last_state[i]) {

            if (currentState == LOW) {
                button_pushed_flag[i] = 1;
                Serial.println("button " + String(i) + " pushed");
                Serial.println(String(button_display_mode));
            } else {
                button_released_flag[i] = 1;
                Serial.println("Button " + String(i) +  "released");
            }
            button_last_state[i] = currentState;
        }
    }
}

void determine_zoom_from_buttons() {
      if (button_display_mode == QUANTIZER){
            return;
      }
      unsigned long currentMillis = millis();
      int long_press_delay = 100;

      if (zoom_complete == false){
            current_zoom_mode = ZOOMED_IN;
      }

      if (button_pushed_flag[0] == 1) {
            zoom_mode_when_button_pressed = current_zoom_mode;
            button_pushed_flag[0] = 0;
            button_millis_last_pushed[0] = currentMillis;
            if (current_zoom_mode == ZOOMED_OUT){
            }
            if (current_zoom_mode == ZOOMED_IN){
                  current_zoom_mode = ZOOMED_OUT;
            }

      }

      if (button_released_flag[0] == 1) {
            if (zoom_mode_when_button_pressed == ZOOMED_OUT){
                  if (current_zoom_mode == ZOOMED_OUT){
                        if (currentMillis-button_millis_last_pushed[0] < long_press_delay){
                              current_zoom_mode = ZOOMED_IN;
                        }
                  }
                  if (current_zoom_mode == ZOOMED_IN){
                        zoom_complete = true;
                  }
            }
            if (zoom_mode_when_button_pressed == ZOOMED_IN){
                  if (current_zoom_mode == ZOOMED_OUT){
                        if (currentMillis-button_millis_last_pushed[0] < long_press_delay){
                              current_zoom_mode = ZOOMED_OUT;
                        }
                        if (currentMillis-button_millis_last_pushed[0] > long_press_delay){
                              current_zoom_mode = ZOOMED_IN;
                        }
                        zoom_complete = true;
                  }
            }
            zoom_complete = true;
            button_released_flag[0] = 0;
      }
}

void determine_quant_mode_from_buttons() {
      if (button_pushed_flag[11] == 1){
            Serial.println("Button Released!");
            if (button_display_mode == NORMAL){
                  Serial.print("Switching to quantize!");
                  button_display_mode = QUANTIZER;
                  button_pushed_flag[11] = 0;
            }
            else {
                  Serial.print("Switching to normal!");
                  button_display_mode = NORMAL;
                  button_pushed_flag[11] = 0;
                  
            }

      }
      if (button_display_mode == QUANTIZER){
            for (int i = 0; i<12; i++){
                  if (quant[i] == 1){
                        button_leds_array[quant_button_address[i]] = CRGB::White;
                  }
                  else {
                        button_leds_array[quant_button_address[i]] = CRGB::Black;
                  }
                  if (button_pushed_flag[quant_button_address[i]] == 1){
                        if (quant[i]==0){
                              quant[i] = 1;
                        }
                        else {
                              quant[i] = 0;
                        }
                        
                        Serial.print("toggling");
                        Serial.print(quant[i]);
                        button_pushed_flag[quant_button_address[i]] = 0;
                  }
            }
      }
}
      


void display_zoom_mode(){
      if (current_zoom_mode == ZOOMED_OUT){
            display_all_sequences();
      }
      else{
            display_active_sequence();
      }

}
      
void setup() {
Serial.begin(31250);
Serial.print("Setting up Now!!");
Serial.print("Setting up Now!!");
Serial.print("Setting up Now!!");
      
      //      TinyUSBDevice.clearConfiguration(); // this must be disabled for Serial to work and re-enabled any time you want the name changed
      //      TinyUSBDevice.setManufacturerDescriptor("Test alsdfk");
      //      TinyUSBDevice.setProductDescriptor("test");
      //      TinyUSBDevice.setSerialDescriptor("Sidewinder CDC");

  for (int i = 0; i < 15; i++) {
    pinMode(button_input_pins[i], INPUT_PULLUP);
  }

pinMode(latchPin, OUTPUT);
SPI.setDataMode(SPI_MODE0);
SPI.setBitOrder(MSBFIRST);
SPI.begin();
digitalWrite(latchPin, HIGH); 
digitalWrite(latchPin, LOW);  
      
      // gabe_usb_midi.setStringDescriptor("TinyUSB Gabe MIDI");
      // Initialize MIDI, and listen to all MIDI channels
            // This will also call usb_midi's begin()
      MIDI_GABE_USB.begin(MIDI_CHANNEL_OMNI);
      // while( !TinyUSBDevice.mounted() ) delay(1);
      MIDI_GABE_DIN.begin();
  
// 


xTaskCreatePinnedToCore(
SPI_input_gather, /* Function to implement the task */
"Task1", /* Name of the task */
10000,  /* Stack size in words */
NULL,  /* Task input parameter */
0,  /* Priority of the task */
&SPI_input_task,  /* Task handle. */
0); /* Core where the task should run */


FastLED.addLeds<WS2812B, LED_RING_DATA_PIN_1, GRB>(ring_leds_array, 0, NUM_LEDS_PER_STRIP);
FastLED.addLeds<WS2812B, LED_RING_DATA_PIN_2, GRB>(ring_leds_array, NUM_LEDS_PER_STRIP * 1, NUM_LEDS_PER_STRIP);
FastLED.addLeds<WS2812B, LED_RING_DATA_PIN_3, GRB>(ring_leds_array, NUM_LEDS_PER_STRIP * 2, NUM_LEDS_PER_STRIP);
FastLED.addLeds<WS2812B, LED_RING_DATA_PIN_4, GRB>(ring_leds_array, NUM_LEDS_PER_STRIP * 3, NUM_LEDS_PER_STRIP);

FastLED.addLeds<WS2812B, LED_BUTTONS_DATA_PIN, GRB>(button_leds_array, 15);


//     FastLED.addLeds<WS2812B, LED_RING_DATA_PIN, GRB>(ring_leds_array, NUM_RING_LEDS);
FastLED.setBrightness(80);
FastLED.show();

remap_leds();
Serial.print("Setting up Now!!");


for (int i = 0; i < 20; i++){
      Sequence[i].sequence_color = 15*i;
}

Sequence[0].number_of_steps = 16;
current_zoom_mode = ZOOMED_IN;
button_display_mode = NORMAL;
Serial.print("Setup Complete");
}


int current_ring = 0;
int last_ring = 20;


void loop() {


if (ready_for_beat() == true){
      for (int i = 0; i<20; i++){
            play_next_note_in_sequence(i);
      }

      // Serial.print("beat is happening");
}

set_button_flags();
determine_zoom_from_buttons();
display_zoom_mode();

determine_quant_mode_from_buttons();

display_buttons();
FastLED.show();


  

}

