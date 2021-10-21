/*
  Co2 Ampel;
  - Shows 1 large meter in the center of the display (default is Co2)
  - Shows temperature and humidity left and right
  - Shows battery power upper right corner (1800mAh)
  - deep sleep mode via Button 1 (pressed long, 2 seconds)
  - toggles between three screen elements (also toggles the large meter in center)
  - adjusted battery percentage to 2000mAh battery
  - implemented protothreads --> set different timers for sensor updates
  - changed behaviour after initial start --> read sensors once at start and then change
    update interval to asynchron readouts
  

  DIN EN 13779:2007-09

500ppm saubere Aussenluft
bis 800ppm hohe Raumluftqualität
800 bis 1000 mittlere Raumluftqualität
1000 bis 1400 mäßige Raumluftqualität
größer 1400 niedrige Raumluftqualität

  
*/

#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <BME280I2C.h>
#include <Wire.h>
#include "esp_adc_cal.h"
#include "intro_ampel.h"
#include "outro_ampel.h"
#include "alarm_clock.h"
#include "Button2.h"
#include <MHZ.h>
#include <pt.h>

// pin for pwm reading
#define CO2_IN 37


MHZ co2(CO2_IN, MHZ14A);


BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

#define TFT_GREY 0x5AEB
#define ILI9341_GREY 0x2104 // Dark grey 16 bit colour

// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0
#define Beeper_Pin          27



TFT_eSPI tft = TFT_eSPI(135, 240);       // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

uint32_t runTime = -99999;       // time for next update

int reading = 0; // Value to be displayed
int d = 0; // Variable used for the sinewave test waveform
int btnCick = false;
static int toggle_tft = 0;

static int vref = 1100;
static int NUM_SAMPLES = 10;
static int sample_count = 0;
static float v = 0;
static int ppm_pwm = 0;
static float pres, temp, hum;

static String battstate = "Green";

static int beepstate = 0;
int set_alarm = 0;
int ppm_alarm_act = 0;

static int firststart_co2 = 1;
static int firststart_bme = 1;
static int firststart_bat = 1;


//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}


void button_init()
{
    btn1.setLongClickHandler([](Button2 & b) {
        btnCick = false;
        int r = digitalRead(TFT_BL);
        tft.fillScreen(TFT_BLUE);
//        tft.setTextColor(TFT_WHITE, TFT_BLACK);
//        tft.drawCentreString("Ampel geht schlafen...", tft.width() / 2, tft.height() / 2-30,2);
//        tft.drawCentreString("Bitte erneut drücken um zu starten", tft.width()/2, tft.height()/2,2); // Value in middle
        tft.setSwapBytes(true);
        tft.pushImage(0, 0,  240, 135, outro_ampel);
        espDelay(7000);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
        delay(200);
        esp_deep_sleep_start();
    });
    btn1.setPressedHandler([](Button2 & b) {
        Serial.println("Set Alarm..");
        btnCick = true;
        set_alarm = (set_alarm + 1) % 2;
    });

    btn2.setPressedHandler([](Button2 & b) {
        btnCick = false;
        toggle_tft = (toggle_tft + 1) % 3; 
        Serial.println("Toggle Index: " + String(toggle_tft));
        tft.fillScreen(TFT_BLACK);
    });
}

void button_loop()
{
    btn1.loop();
    btn2.loop();
}





// Declare 3 protothreads
static struct pt pt1, pt2, pt3, pt4;

// First protothread function to blink LED 1 every 1 second
static int protothreadReadMHZ(struct pt *pt)
{
  static unsigned long checkMHZ = 0;
  PT_BEGIN(pt);
  while(1) {
    checkMHZ = millis();
    if (firststart_co2 == 1) {
      PT_WAIT_UNTIL(pt, millis() - checkMHZ > 1000);
      ppm_pwm = co2.readCO2PWM();
      firststart_co2 = 0;
    } else if (firststart_co2 == 0) {
      PT_WAIT_UNTIL(pt, millis() - checkMHZ > 20000);
      ppm_pwm = co2.readCO2PWM();
    }
  }
  PT_END(pt);
}


static int protothreadReadBME(struct pt *pt)
{
  static unsigned long checkBME = 0;
  PT_BEGIN(pt);
  while(1) {
    checkBME = millis();
    if (firststart_bme == 1) {
      PT_WAIT_UNTIL(pt, millis() - checkBME > 1000);
      BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
      BME280::PresUnit presUnit(BME280::PresUnit_Pa);
      bme.read(pres, temp, hum, tempUnit, presUnit);
      firststart_bme = 0;
    } else if (firststart_bme == 0) {
      PT_WAIT_UNTIL(pt, millis() - checkBME > 15000);
      BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
      BME280::PresUnit presUnit(BME280::PresUnit_Pa);
      bme.read(pres, temp, hum, tempUnit, presUnit);
    }
  }
  PT_END(pt);
}


static int protothreadCheckBattery(struct pt *pt)
{
  static unsigned long checkBatt = 0;
  static int deadband = 2;
  PT_BEGIN(pt);
  while(1) {
    checkBatt = millis();
    if (firststart_bat == 1) {
      PT_WAIT_UNTIL(pt, millis() - checkBatt > 1000);
      firststart_bat = 0;
    } else if (firststart_bme == 0) {
      PT_WAIT_UNTIL(pt, millis() - checkBatt > 10000);
    }
    
    while (sample_count < NUM_SAMPLES) {
          v += analogRead(ADC_PIN);
          sample_count++;
          //delay(10);
        }
        float battery_voltage = (v / NUM_SAMPLES / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        float batteryLevel = _min(map(v / NUM_SAMPLES, 1940, 2318, 0, 100), 100); //1100
        if (v/NUM_SAMPLES < 1200) batteryLevel = 0;
        //String voltage = String(battery_voltage) + "V";
        //String battery = String(batteryLevel,0) + "%";
        String digire = String(v/NUM_SAMPLES,0);

        // Delete previous battery bar value
        tft.fillRoundRect(204, 8, 28,10,2,TFT_BLACK);
        tft.drawRoundRect(203, 7, 30, 12,5, TFT_WHITE);

        if (battstate == "Green"){
          tft.fillRoundRect(204, 8, batteryLevel*28/100,10,2,TFT_GREEN);
        } else if (battstate == "Red"){
          tft.fillRoundRect(204, 8, batteryLevel*28/100,10,2,TFT_RED);
        }

        if ((battstate == "Green") && (batteryLevel < 15)){
          battstate = "Red";
        }

        if ((battstate == "Red") && (batteryLevel > 25)) {
          battstate = "Green";
        }

        //tft.drawString(digire,  210, 25 );

        sample_count = 0;
        v = 0;
  }
  PT_END(pt);
}


// Beeper protothread function
static int protothreadBeeper(struct pt *pt)
{
  static unsigned long checkBeeper = 0;
  PT_BEGIN(pt);
  while(1) {
    checkBeeper = millis();
    PT_WAIT_UNTIL(pt, millis() - checkBeeper > 1000);
    //Serial.println("ALARM CALL ON");
    digitalWrite(Beeper_Pin, HIGH);
    delay(1000); 
    digitalWrite(Beeper_Pin, LOW);
    beepstate = 1;
  }
  PT_END(pt);
}





void setup(void) {
  Serial.begin(38400);
  Serial.println("Start");

  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);
  pinMode(CO2_IN, INPUT);
  
  tft.init();
  tft.setRotation(1); // 1: Portrait USB right, 3: Portrait USB left
  tft.fillScreen(TFT_BLACK);

  tft.setSwapBytes(true);
  tft.pushImage(0, 0,  240, 135, intro_ampel);
  espDelay(3000);

  button_init();
  pinMode(27, OUTPUT);
  digitalWrite(Beeper_Pin, LOW);

  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
      vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
      Serial.println("Default Vref: 1100mV");
  }

    tft.fillScreen(TFT_BLACK);
    Serial.println("Toggle Index: " + String(toggle_tft));

  // ######### Start BME280 initialization via i2c bus (which is Pin21 = SDA / Pin22 = SCL)
  Wire.begin(21,22);
  bme.begin();


  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);
  PT_INIT(&pt4);
 
}


void loop() {
  protothreadReadMHZ(&pt1);
  protothreadReadBME(&pt2);
  protothreadCheckBattery(&pt3);
  button_loop();

  if (ppm_alarm_act == 1) {
    if ((beepstate == 0) && (ppm_pwm > 2000)) {
      protothreadBeeper(&pt4);
    }
  }
  if (ppm_pwm < 1500) {
    beepstate = 0;
  }
    
  
  if (toggle_tft == 0) { // This is for CO2
    if (millis() - runTime >= 500L) { // Execute every 0.5s
    runTime = millis();


    // Set the the position, gap between meters, and inner radius of the meters
    int radius = 70;
    int xpos = tft.width()/2-radius, ypos = 0, gap = 0;

    xpos = gap + ringMeter(ppm_pwm, 0, 2500, xpos, ypos, radius, "ppm", GREEN2RED); // Draw analogue meter

    // Show Temp Value in corner
    
    float temp_read = temp;
    String temp_s = String(temp_read,0);
    tft.drawCentreString(temp_s, 25, 100, 2); // Value in middle
    tft.drawCentreString("degC", 25, 115, 2); // Value in middle

    // Show Humidity Value in corner
    float hum_read = hum;
    String hum_s = String(hum_read,0);
    tft.drawCentreString(hum_s, 210, 100, 2); // Value in middle
    tft.drawCentreString("%RH", 210, 115, 2); // Value in middle
    }
    if (set_alarm == 1) {
      tft.pushImage(104, 103,  32, 32, alarm_clock);
      ppm_alarm_act = 1;
    } else if (set_alarm == 0) {
      // Set rectangle to delete icon
      tft.fillRect(104, 103, 32,32,TFT_BLACK);
      ppm_alarm_act = 0;
    }
  }

  if (toggle_tft == 1) { // Where we want Temperature on meter --> shift everything to the right
    if (millis() - runTime >= 500L) { // Execute every 2s
    runTime = millis();


    // Set the the position, gap between meters, and inner radius of the meters
    int radius = 70;
    int xpos = tft.width()/2-radius, ypos = 0, gap = 0;

    xpos = gap + ringMeter(temp, 5, 35, xpos, ypos, radius, "degC", BLUE2RED); // Draw analogue meter


    // Show Humidity Value in corner
    String hum_s = String(hum,0);
    tft.drawCentreString(hum_s, 25, 100, 2);
    tft.drawCentreString("%RH", 25, 115, 2);

    // Show PPM Value in corner
    tft.drawCentreString(String(ppm_pwm), 210, 100, 2); // Value in middle
    tft.drawCentreString("ppm", 210, 115, 2); // Value in middle
    }
    if (set_alarm == 1) {
      tft.pushImage(104, 103,  32, 32, alarm_clock);
      ppm_alarm_act = 1;
    } else if (set_alarm == 0) {
      // Set rectangle to delete icon
      tft.fillRect(104, 103, 32,32,TFT_BLACK);
      ppm_alarm_act = 0;
    }
  }

  if (toggle_tft == 2) { // Where we want Humidity on meter --> shift everything to the right
    if (millis() - runTime >= 500L) { // Execute every 2s
    runTime = millis();


    // Set the the position, gap between meters, and inner radius of the meters
    int radius = 70;
    int xpos = tft.width()/2-radius, ypos = 0, gap = 0;

    xpos = gap + ringMeter(hum, 0, 100, xpos, ypos, radius, "%RH", BLUE2BLUE); // Draw analogue meter

    // Show PPM Value in corner
    tft.drawCentreString(String(ppm_pwm), 25, 100, 2);
    tft.drawCentreString("ppm", 25, 115, 2);

    // Show Temp Value in corner
    String temp_s = String(temp,0);
    tft.drawCentreString(temp_s, 210, 100, 2); // Value in middle
    tft.drawCentreString("degC", 210, 115, 2); // Value in middle
    }
    if (set_alarm == 1) {
      tft.pushImage(104, 103,  32, 32, alarm_clock);
      ppm_alarm_act = 1;
    } else if (set_alarm == 0) {
      // Set rectangle to delete icon
      tft.fillRect(104, 103, 32,32,TFT_BLACK);
      ppm_alarm_act = 0;
    }
  }
}


// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
static int ringMeter(int value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  
  x += r; y += r;   // Calculate coords of centre of ring

  int w = r / 3;    // Width of outer ring is 1/3 of radius
  
  int angle = 150;  // Half the sweep angle of meter (300 degrees)

  int text_colour = 0; // To hold the text colour

  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

  byte seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
  byte inc = 5; // Draw segments every 5 degrees, increase to 10 for segmented ring

  // Draw colour blocks every inc degrees
  for (int i = -angle; i < angle; i += inc) {

    // Choose colour from scheme
    int colour = 0;
    switch (scheme) {
      case 0: colour = TFT_RED; break; // Fixed colour
      case 1: colour = TFT_GREEN; break; // Fixed colour
      case 2: colour = TFT_BLUE; break; // Fixed colour
      case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
      case 4: colour = rainbow(map(i, -angle, angle, 63, 127)); break; // Green to red (high temperature etc)
      case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
      default: colour = TFT_BLUE; break; // Fixed colour
    }

    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;


    

    if ((i < v)&&(toggle_tft == 0)) { // Fill in coloured segments with 2 triangles for CO2
      if (value <= 1000) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
        text_colour = colour; // Save the last colour drawn
      }
      else if (1000 < value && value <= 2000) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
        text_colour = colour; // Save the last colour drawn   
      }
      else if (value > 2000) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
        text_colour = colour; // Save the last colour drawn
      }

    }
    
     else if ((i < v)&&(toggle_tft == 1)) {
       tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
       tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
     }
     else if ((i < v)&&(toggle_tft == 2)) {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
     }
     else // Fill in blank segments
     {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, ILI9341_GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, ILI9341_GREY);
     }

    if (toggle_tft == 0) {
     tft.drawLine(85, 10, 98, 29, TFT_WHITE);
     tft.drawLine(84, 10, 97, 29, TFT_WHITE);
     tft.drawLine(83, 10, 96, 29, TFT_WHITE);

     tft.drawLine(166, 69, 189, 69, TFT_WHITE);
     tft.drawLine(166, 70, 189, 70, TFT_WHITE);
     tft.drawLine(166, 71, 189, 71, TFT_WHITE);
    }
  }

  // Convert value to a string
  char buf[10];
  byte len = 4; if (value > 999) len = 5;
  dtostrf(value, len, 0, buf);

  float display_val_f = float(value);
  String displayval = String(display_val_f,0);

  // Set rectangle to delete any previous value
  tft.fillRect(x - 37, y - 22, 72,27,TFT_BLACK);

  // Set the text colour to default  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Uncomment next line to set the text colour to the last segment value!
  // tft.setTextColor(text_colour, ILI9341_BLACK);
  
  // Print value, if the meter is large then use big font 6, othewise use 4
  tft.drawCentreString(buf, x - 5, y - 20, 4); // Value in middle

  // Print units, if the meter is large then use big font 4, othewise use 2
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString(units, x, y + 5, 2); // Units display

  // Calculate and return right hand side x coordinate
  return x + r;

}

// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}
