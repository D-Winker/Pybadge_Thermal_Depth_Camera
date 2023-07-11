// TODO: Clean up this header
// The VL53L5CX has a 45x45 degree FoV (65 degree diagonal FoV)
// Regarding FoV: AN5894, Description of the fields of view of STMicroelectronics' Time-of-Flight sensors
// The MLX90640 has a 55x35 degree FoV
//
// VL53LCX
// Initially had issues with the VL53LCX because the PyBadge puts VBat on the Qwiic connector instead of 3.3V
// The sensor returns perpendicular distance, not radial distance
// 
// SPDX-FileCopyrightText: 2020 Anne Barela for Adafruit Industries
//
// SPDX-License-Identifier: MIT

/*
  ThermalImager_009b - Collect thermal image values from a MLX90640 sensor array,
                       display them as color-mapped pixels on a TFT screen,
  include data capture to flash media, and a user configuration menu.
  Written by Eduardo using code from these sources.

  Arcada and MLX90640 libraries from adafruit.com

  Ver.  1 - Read temps, auto-range extremes, display gray squares on TFT
  Ver.  2 - Add Ironbow color palette, low+center+high markers
  Ver.  3 - Add crude BMP image write to SD
  Ver.  4 - Attach interrupts to improve button response
  Ver.  5 - Store BMPs to SD in an orderly manner, in folders
  Ver.  6 - Port to Teensy 3.2, where the libraries used are suited
  Ver.  7 - Port to Adafruit PyBadge using Arcada library.  Use simulated data while awaiting hardware release
  Ver.  8 - Convert menu to scrolling style and add settings for emissivity and frame rate, more if feasible.
  Ver.  9 - Bring in the current Adafruit library and read a real sensor.
*/

#include <Adafruit_MLX90640.h>
#include "Adafruit_Arcada.h"
Adafruit_MLX90640 mlx;
Adafruit_Arcada arcada;

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // http://librarymanager/All#SparkFun_VL53L5CX

// The below constant is set in "platform.h"
const uint8_t targetsPerZone = VL53L5CX_NB_TARGET_PER_ZONE;
// The sensor can return up to 4 distances per pixel; i.e. detect 4 targets per zone.
// The targets must be at least 600mm apart. 
// It looks like setting targets per zone to 4 doesn't work; 3 does!
// Code size when setting (1, 2, 3, 4) targets is (162400, 164416, 164368, 162560) bytes
// ...looks like a problem with 4.
// Returns are all in one array, [zone 1 return 1, zone 1 return 2, ..., zone 64 return 1, ...]

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData vl53Data; // Result data class structure
float vl53Nearest[64];  // At the moment, I have my VL53L5CX set to return up to 3 values per zone. For convenience, I'll pull out the nearest values per zone and store them here.
float vl53All[64*targetsPerZone];  // Store the raw data from the VL53L5CX

#if !defined(USE_TINYUSB)
  #warning "Compile with TinyUSB selected!"
#endif

File myFile;

float mlx90640To[768];   // Here we receive the float vals acquired from MLX90640
uint8_t upsample = 1;  // This can be changed from 1 to 4 with the Up button
uint8_t sensorMode = 0;  // 0 = thermal, 1 = depth, 2 = 'intelligent' depth, 3 = thermal + depth, 4 = thermal + 'intelligent' depth

#define DE_BOUNCE 200
  // Wait this many msec between button clicks
#define MENU_LEN 13
  // Number of total available menu choices
#define MENU_ROWS 9
  // Number of menu lines that can fit on screen
#define MENU_VPOS 6
#define GRAY_33 0x528A
#define BOTTOM_DIR "MLX90640"
#define BOTTOM_DIR_CSV "VL53L5CX"
#define DIR_FORMAT "/dir%05d"
#define BMP_FORMAT "/frm%05d.bmp"
#define CSV_FORMAT "/frm%05d.csv"
#define CFG_FLNAME "/config.ini"
#define MAX_SERIAL 999

// BMP File Header, little end first, Photoshop ver.
const PROGMEM uint8_t BmpPSPHead[14] = {
 0x42, 0x4D,             // "BM" in hex
 0x38, 0x09, 0x00, 0x00, // File size, 2360
 0x00, 0x00,             // reserved for app data 1
 0x00, 0x00,             // reserved for app data 2
 0x36, 0x00, 0x00, 0x00  // Offset of first pixel, 54
};

// BMP 24-bit DIB Header, little end first, Photoshop ver.
const PROGMEM uint8_t DIBHeadPSP1[40] = {
 0x28, 0x00, 0x00, 0x00,  // Header size, 40
 0x20, 0x00, 0x00, 0x00,  // pixel width, 32
 0x18, 0x00, 0x00, 0x00,  // pixel height, 24
 0x01, 0x00,              // color planes, 1
 0x18, 0x00,              // bits per pixel, 24
 0x00, 0x00, 0x00, 0x00,  // Compression method, 0==none
 0x00, 0x00, 0x00, 0x00,  // Raw bitmap data size, dummy 0
 0x12, 0x0B, 0x00, 0x00,  // Pixels per meter H, 2834
 0x12, 0x0B, 0x00, 0x00,  // Pixels per meter V, 2834
 0x00, 0x00, 0x00, 0x00,  // Colors in palette, 0==default 2^n
 0x00, 0x00, 0x00, 0x00   // Number of important colors, 0
};

// BMP file data, 2 byte padding
const PROGMEM uint8_t PSPpad[2] = {0x00, 0x00};

//Byte arrays of bitmapped icons, 16 x 12 px:
const PROGMEM uint8_t battIcon[] = {
0x0f, 0x00, 0x3f, 0xc0, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 
0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 0x20, 0x40, 0x3f, 0xc0};

const PROGMEM uint8_t camIcon[] = {
0x01, 0xe0, 0x61, 0x20, 0xff, 0xf0, 0x80, 0x10, 0x86, 0x10, 0x89, 0x10, 
0x90, 0x90, 0x90, 0x90, 0x89, 0x10, 0x86, 0x10, 0x80, 0x10, 0xff, 0xf0};

const PROGMEM uint8_t SDicon[] = {
0x0f, 0xe0, 0x1f, 0xe0, 0x3c, 0x60, 0x78, 0x60, 0x70, 0x60, 0x60, 0x60, 
0x60, 0x60, 0x60, 0x60, 0x6f, 0x60, 0x60, 0x60, 0x7f, 0xe0, 0x7f, 0xe0};

const PROGMEM uint8_t snowIcon[] = {
0x15, 0x00, 0x4E, 0x40, 0xC4, 0x60, 0x75, 0xC0, 0x9F, 0x20, 0x0E, 0x00,
0x0E, 0x00, 0x9F, 0x20, 0x75, 0xC0, 0xC4, 0x60, 0x4E, 0x40, 0x15, 0x00};

uint8_t pixelArray[2304];   // BMP image body, 32 pixels * 24 rows * 3 bytes

// Some global values that several functions will use, including
// 5 floats to append to the BMP pixel data:
// coldest pixel, coldest color, center temp, hottest color, hottest pixel
float sneakFloats[5] = {3.1415926, 0.0, -11.7, 98.6, -12.34};      // Test values that get overwritten
float sneakFloatsVL53[5] = {3.1415926, 0.0, -11.7, 98.6, -12.34};      // Test values that get overwritten
uint16_t highAddr = 0, lowAddr = 0, highAddr53 = 0, lowAddr53 = 0; // Append the pixel addresses, too

uint16_t backColor, lowPixel, highPixel, buttonRfunc = 1,
         emissivity = 95, frameRate = 4, sharpener = 25,
         thermRange = 0, paletteNum = 1, colorPal[256],            // Array for color palettes
         nextDirIndex = 0, nextFileIndex = 0, nextFrameSequence = 1;  // These keep count of SD files and dirs, 0==error
uint32_t deBounce = 0, buttonBits = 0;
boolean mirrorFlag = false, celsiusFlag = false, markersOn = true,
        screenDim = false, smoothing = false, showLastCap = false,
        save1frame = false, recordingInProg = false, buttonActive = false;
float battAverage = 0.0, colorLow = 0.0, colorHigh = 100.0, colorLow53 = 0.0, colorHigh53 = 100.0;        // Values for managing color range
volatile boolean clickFlagMenu = false, clickFlagSelect = false, clickFlagUpsample = false, clickFlagSmoothstep = false, clickFlagSwitchMode = false;   // Volatiles for timer callback handling
volatile boolean useSmoothstep = false;

void setup()
{
  if (!arcada.arcadaBegin()) {    // Start TFT and fill with black
    // Serial.print("Failed to begin");
    while (1);
  }
  arcada.filesysBeginMSD();       // Set up SD or QSPI flash as an external USB drive

  arcada.displayBegin();              // Activate TFT screen
  arcada.display->setRotation(1);     // wide orientation
  arcada.display->setTextWrap(false);
  arcada.setBacklight(255);           // Turn on backlight
  battAverage = arcada.readBatterySensor();

  Serial.begin(115200);
//  while(!Serial); // Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");

  if(arcada.filesysBegin()){              // Initialize flash storage, begin setting up indices for saving BMPs
    if(!arcada.exists(BOTTOM_DIR) && !arcada.exists(BOTTOM_DIR_CSV)) {      // Are base directories absent?
      if(arcada.mkdir(BOTTOM_DIR) && arcada.mkdir(BOTTOM_DIR_CSV))        // Can they be added?
        nextDirIndex = nextFileIndex = 1;  // Success, prepare to store numbered files & dirs
        
    } else if (arcada.exists(BOTTOM_DIR)) {      // "MLX90640" directory exists, can we add files | directories?
      arcada.mkdir(BOTTOM_DIR_CSV);
      // Get the number of the next unused serial directory path (keep CSV and BMP dirs synced)
      nextDirIndex = availableFileNumber(1, BOTTOM_DIR + String(DIR_FORMAT));
      // and the next unused serial file name
      nextFileIndex = availableFileNumber(1, BOTTOM_DIR + String(BMP_FORMAT));
      
    } else if (arcada.exists(BOTTOM_DIR_CSV)) {
      arcada.mkdir(BOTTOM_DIR);
      // Get the number of the next unused serial directory path (keep CSV and BMP dirs synced)
      nextDirIndex = availableFileNumber(1, BOTTOM_DIR_CSV + String(DIR_FORMAT));
      // and the next unused serial file name
      nextFileIndex = availableFileNumber(1, BOTTOM_DIR_CSV + String(BMP_FORMAT));
      
    } else {  // Both directories exist
      // Get the number of the next unused serial directory path (keep CSV and BMP dirs synced)
      nextDirIndex = availableFileNumber(1, BOTTOM_DIR + String(DIR_FORMAT));
      uint16_t nextDirIndexTemp = availableFileNumber(1, BOTTOM_DIR_CSV + String(DIR_FORMAT));
      if (nextDirIndexTemp > nextDirIndex) {
        nextDirIndex = nextDirIndexTemp;
      }
      // and the next unused serial file name
      nextFileIndex = availableFileNumber(1, BOTTOM_DIR + String(BMP_FORMAT));
      uint16_t nextFileIndexTemp = availableFileNumber(1, BOTTOM_DIR_CSV + String(CSV_FORMAT));
      if (nextFileIndexTemp > nextFileIndex) {
        nextFileIndex = nextFileIndexTemp;
      }
    }
  }  // By now each global index variable is either 0 (no nums available), or the next unclaimed serial num

  // Set up VL53L5CX
  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(1000000);
  delay(100);  
  
  if (myImager.begin() == false)
  {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Sensor not found - check your wiring. Freezing", ARCADA_WHITE);
    while (1) ;
  } else {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Found the sensor!", ARCADA_WHITE);
  }
  
  myImager.setResolution(8*8); //Enable all 64 pads
  delay(1000);
  
  // Set the ranging mode
  bool response = myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  // Integration time has no impact in continuous mode, so no need to set it
  if (response == true) {
    SF_VL53L5CX_RANGING_MODE mode = myImager.getRangingMode();
    switch (mode) {
      case SF_VL53L5CX_RANGING_MODE::AUTONOMOUS:  // Lower power, lower performance
        arcada.display->fillScreen(ARCADA_BLACK);
        drawtext("Ranging mode set to autonomous.", ARCADA_WHITE);
        delay(5000);
        break;

      case SF_VL53L5CX_RANGING_MODE::CONTINUOUS:  // Higher power, higher performance
        arcada.display->fillScreen(ARCADA_BLACK);
        drawtext("Ranging mode set to continuous.", ARCADA_WHITE);
        delay(500);
        break;

      default:
        arcada.display->fillScreen(ARCADA_BLACK);
        drawtext("Error recovering ranging mode.", ARCADA_WHITE);
        delay(5000);
        break;
    }
  } else {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Cannot set ranging mode requested.", ARCADA_WHITE);
    delay(5000);
  }

  // Sharpener is essentially rejection of cross-pixel light detection. 
  // Too low and a single target can swamp the scene, too high and a single target may be the only visible part of the scene
  // Default value is 5%
  response = myImager.setSharpenerPercent(sharpener);
  if (response == true) {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Set sharpener.", ARCADA_WHITE);
    delay(500);
  } else {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Cannot set sharpener value.", ARCADA_WHITE);
    delay(5000);
  }

  // The sensor can report multiple detected targets per zone. The order can be closest target first, or strongest return first.
  // The minimum distance between two targets to be detected is 600 mm.
  // Number of targets per zone cannot be set with the API; it has to be done in the ‘platform.h’ file. The macro
  // VL53L5CX_NB_TARGET_PER_ZONE needs to be set to a value between 1 and 4. Default is 1. More targets uses more RAM. 
  // (I increased it to 4)
  response = myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
  if (response == true) {
    SF_VL53L5CX_TARGET_ORDER order = myImager.getTargetOrder();
    switch (order) {
      case SF_VL53L5CX_TARGET_ORDER::STRONGEST:
        arcada.display->fillScreen(ARCADA_BLACK);
        drawtext("Target order set to strongest.", ARCADA_WHITE);
        delay(5000);
        break;

      case SF_VL53L5CX_TARGET_ORDER::CLOSEST:
        arcada.display->fillScreen(ARCADA_BLACK);
        drawtext("Target order set to closest.", ARCADA_WHITE);
        delay(500);
        break;

      default:
        break;
    }
  } else {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Cannot set target order.", ARCADA_WHITE);
    delay(5000);
  }
  
  // Set the image frequency. Default is 1Hz.
  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  response = myImager.setRangingFrequency(15);

  if (response == true) {
    int fps = myImager.getRangingFrequency();
    if (fps > 0) {
      
      arcada.display->fillScreen(ARCADA_BLACK);
      char fullmsg[50] = "";
      char msg[] = "Ranging frequency set to ";
      char * ptr1 = msg;
      strcat(fullmsg, msg);
      ptr1 = fullmsg + strlen(fullmsg);
      itoa(fps, ptr1, 10);
      drawtext(ptr1, ARCADA_WHITE);
      delay(1000);
    } else {      
      arcada.display->fillScreen(ARCADA_BLACK);
      drawtext("Error recovering ranging frequency.", ARCADA_WHITE);
      delay(1000);
    }
  } else {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Cannot set frequency!", ARCADA_WHITE);
    delay(2000);
  }

  if (myImager.startRanging()) {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Starting ranging", ARCADA_WHITE);
  } else {
    arcada.display->fillScreen(ARCADA_BLACK);
    drawtext("Cannot start ranging!", ARCADA_WHITE);
    delay(2000);
  }

  // -----------------------------------------------
   
  // Thermal camera
  if(!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found!");
    arcada.haltBox("MLX90640 not found!");
    while(1)
      delay(10);  // Halt here
  }
  Serial.println("Found Adafruit MLX90640");

  Serial.print("Serial number: ");
  Serial.print(mlx.serialNumber[0], HEX);
  Serial.print(mlx.serialNumber[1], HEX);
  Serial.println(mlx.serialNumber[2], HEX);

  //mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);

  switch(frameRate) {
    case 0: mlx.setRefreshRate(MLX90640_0_5_HZ); break; // 6 frame rates, 0.5 to 16 FPS in powers of 2
    case 1: mlx.setRefreshRate(MLX90640_1_HZ); break;
    case 2: mlx.setRefreshRate(MLX90640_2_HZ); break;
    case 3: mlx.setRefreshRate(MLX90640_4_HZ); break;
    case 4: mlx.setRefreshRate(MLX90640_8_HZ); break;
    default: mlx.setRefreshRate(MLX90640_16_HZ); break;
  }
  Wire.setClock(1000000); // max 1 MHz

  for(int counter01 = 0; counter01 < 2304; ++counter01)
    pixelArray[counter01] = counter01 / 9;  // Initialize BMP pixel buffer with a gradient

  loadPalette(paletteNum);             // Load false color palette
  backColor = GRAY_33;                 // 33% gray for BG
  setBackdrop(backColor, buttonRfunc); // Current BG, current button labels

  arcada.timerCallback(50, buttonCatcher);  // Assign a 50Hz callback function to catch button presses
}


float smoothstep(float _f) {
    // https://iquilezles.org/articles/texture/
    return _f * _f * _f * (_f * (_f * 6.0 - 15.0) + 10.0);
}


// function to perform bilinear interpolation
float interpolate(float x, float y, float inputData[24*32], bool _useSmoothstep, int cols=32) {
  // calculate the integer indices of the nearest 4 pixels
  int x1 = floor(x);
  int x2 = ceil(x);
  int y1 = floor(y);
  int y2 = ceil(y);

  // Handle cases at the edge of the grid
  if (y2 > 7) {
    y2--;
    y1--;
  }
  if (x2 > 7) {
    x2--;
    x2--;
  }

  // calculate the fractional distances to each of the 4 pixels
  // 0.5 is added to input-pixel positions because we reference the center of each pixel
  float dx1 = abs((x2+0.5) - x);
  float dx2 = abs(x - (x1+0.5));
  float dy1 = abs((y2+0.5) - y);
  float dy2 = abs(y - (y1+0.5));

  // Apply a smoothstep to the distances
  if (_useSmoothstep) {
    dx1 = smoothstep(dx1);
    dx2 = smoothstep(dx2);
    dy1 = smoothstep(dy1);
    dy2 = smoothstep(dy2);
  }
  
  // calculate the weighted average of the pixel values
  float f11 = inputData[x1+cols*y1];
  float f12 = inputData[x1+cols*y2];
  float f21 = inputData[x2+cols*y1];
  float f22 = inputData[x2+cols*y2];
  float f = (f11 * dx1 * dy1 + f21 * dx2 * dy1 + f12 * dx1 * dy2 + f22 * dx2 * dy2) / (dx1 * dy1 + dx2 * dy1 + dx1 * dy2 + dx2 * dy2);

  // return the interpolated value
  return f;
}


void loop()
{
  static uint32_t frameCounter = 0;
  float scaledPix, highPix, lowPix, scaledPix53, highPix53, lowPix53;
  uint16_t markColor;

  // Show the battery level indicator, 3.7V to 3.3V represented by a 7 segment bar
  battAverage = battAverage * 0.95 + arcada.readBatterySensor() * 0.05; // *Gradually* track battery level
  highPix = (int)constrain((battAverage - 3.3) * 15.0, 0.0, 6.0) + 1;   // Scale it to a 7-segment bar
  markColor = highPix > 2 ? 0x07E0 : 0xFFE0;                            // Is the battery level bar green or yellow?
  markColor = highPix > 1 ? markColor : 0xF800;                         // ...or even red?
  arcada.display->fillRect(146, 2, 12, 12, backColor);                  // Erase old battery icon
  arcada.display->drawBitmap(146, 2, battIcon, 16, 12, 0xC618);         // Redraw gray battery icon
  arcada.display->fillRect(150, 12 - highPix, 4, highPix, markColor);   // Add the level bar

  // Check if distance sensor has data ready; read in the data
  if (myImager.isDataReady() == true)
  {
    if (!myImager.getRangingData(&vl53Data)) //Read distance data into array
    {  
      Serial.println("Failed to pull distance data");
      return;
    }
  }

  // Read all of the data into vl53All
  for (int i=0; i < 64*targetsPerZone; i++) {
    vl53All[i] = vl53Data.distance_mm[i];
  }
  // Read the nearest measurement per zone from vl53Data into vl53Nearest
  for (int16_t x=0; x < 8; x++) {
    for (int16_t y=0; y < 8; y++) {
      vl53Nearest[x+y*8] = vl53Data.distance_mm[(((7-y)*8) + x)*targetsPerZone];  //The ST library returns the data transposed from zone mapping shown in datasheet
    }
  }

  // First pass: Find furthest and closest pixels
  // TODO: Can I get rid of the special -53 versions and just set it appropriately depending on sensor mode?
  highAddr53 = lowAddr53 = 0;
  highPix53  = lowPix53  = vl53Nearest[highAddr53];

  if (sensorMode == 2 || sensorMode == 4) {
    for (int x=1; x < 64*targetsPerZone; x++) { // Compare every pixel   
      if(vl53Data.distance_mm[x] > highPix53) {   // Farther pixel found?
        highPix53 = vl53Data.distance_mm[x];      // Record its values
        highAddr53 = x;
      }
      if(vl53Data.distance_mm[x] < lowPix53 && vl53Data.distance_mm[x] > 0) {    // Closer pixel found?
        lowPix53 = vl53Data.distance_mm[x];       // Likewise
        lowAddr53 = x;
      }
    }
  } else {
    for (int x=1; x < 64; x++) { // Compare every pixel   
      if(vl53Nearest[x] > highPix53) {   // Farther pixel found?
        highPix53 = vl53Nearest[x];      // Record its values
        highAddr53 = x;
      }
      if(vl53Nearest[x] < lowPix53) {    // Closer pixel found?
        lowPix53 = vl53Nearest[x];       // Likewise
        lowAddr53 = x;
      }
    }
  }
  if(thermRange == 0) {    // Are the colors set to auto-range?
    colorLow53 = lowPix53;     // Then high and low color values get updated
    colorHigh53 = highPix53;
  }
  sneakFloatsVL53[0] = lowPix53;     // Retain these five distance values
  sneakFloatsVL53[1] = colorLow53;   // to append to the BMP file, if any
  sneakFloatsVL53[2] = vl53Nearest[32];  // Middle pixel?
  sneakFloatsVL53[3] = colorHigh53;
  sneakFloatsVL53[4] = highPix53;

  // Fetch 768 fresh temperature values from the MLX90640
  arcada.display->drawBitmap(146, 18, camIcon, 16, 12, 0xF400); // Show orange camera icon during I2C acquisition
  if(mlx.getFrame(mlx90640To) != 0) {
    Serial.println("Failed to pull thermal data");
    return;
  }
  arcada.display->fillRect(146, 18, 12, 12, backColor);         // Acquisition done, erase camera icon

  // First pass: Find hottest and coldest pixels
  highAddr = lowAddr = 0;
  highPix  = lowPix  = mlx90640To[highAddr];

  for (int x = 1 ; x < 768 ; x++) { // Compare every pixel
    if(mlx90640To[x] > highPix) {   // Hotter pixel found?
      highPix = mlx90640To[x];      // Record its values
      highAddr = x;
    }
    if(mlx90640To[x] < lowPix) {    // Colder pixel found?
      lowPix = mlx90640To[x];       // Likewise
      lowAddr = x;
    }
  }
  if(thermRange == 0) {    // Are the colors set to auto-range?
    colorLow = lowPix;     // Then high and low color values get updated
    colorHigh = highPix;
  }
  sneakFloats[0] = lowPix;     // Retain these five temperature values
  sneakFloats[1] = colorLow;   // to append to the BMP file, if any
  sneakFloats[2] = mlx90640To[400];  // Middle-ish, pixel?
  sneakFloats[3] = colorHigh;
  sneakFloats[4] = highPix;

  // calculate the ratio of the output size to the input size
  // The thermal camera's pixels are, nominally, drawn onto 4x4 pixels on the screen
  // But we can upsample, making each screen on the pixel either 2x2 or 1x1
  //float ratio = (float) upsample;
  //float mlx90640Up[768*upsample*upsample] = {0};  // For the upsampled output of the MLX90640 measurements
  uint8_t pxSize = 4.0 / upsample;

  // For the depth camera
  //float vl53Up[64*upsample*upsample] = {0};  // For the upsampled output of the VL53L5CX measurements

  // This will hold all of the pixels that we draw from either the thermal camera, depth camera, or both
  float imageArea[768*4*4] = {0};
  float imageArea2[768*4*4] = {0};  // A second channel of data to display
  
  // We want to overlay the depth camera data.
  // The VL53L5CX has a 45x45 degree FoV (65 degree diagonal FoV) and 8x8 sensing zones
  // 5.625 degrees per zone, horizontally and vertically
  // The MLX90640 has a 55x35 degree FoV and returns 32x24 pixels (width x height)
  // 1.71875 degrees per pixel horizontally, and 1.4583 degrees per pixel vertically
  // This is nominally drawn onto 4x4 pixel squares (1 camera pixel = 4x4 screen pixels)
  // 32 camera pixels = 128 screen pixels -> 1 pixel = 0.43 degrees
  // VL53 would be 104.72 pixels wide, and each zone takes 13x13 pixels
  // So...those numbers (4x4, 13x13, also 5.625 and 1.72) don't exactly line up nicely, but I guess
  // we can just fill out a buffer one pixel at a time.
  
  if (sensorMode == 0 || sensorMode == 3 || sensorMode == 4) {
    // DEBUG fill the imageArea array with a test pattern
    for(int y = 0; y < 96; y++) {
      for(int x = 0; x < 128; x++) {
        imageArea[128 * y + x] = abs(abs(y%2 - x%2)*255 - (x+y));
      }
    }
    // END DEBUG
    // loop over all the pixels in the output grid
    for (int i = 0; i < 24*upsample; i++) {
      for (int j = 0; j < 32*upsample; j++) {
        // calculate the corresponding coordinates in the input grid
        // this position is based on the center of the pixel
        float x = (float) j / (float) upsample + (1.0 / (float) upsample) / 2;
        float y = (float) i / (float) upsample + (1.0 / (float) upsample) / 2;
    
        // perform bilinear interpolation and store the result
        //TODO: Make it faster! I'm not sure how, but wow it's slow right now with the 4x interpolation!
        // 32 * upsample is the number of pixels in a row
        //mlx90640Up[32 * upsample * i + j] = interpolate(x, y, mlx90640To, useSmoothstep);
        float pixVal = interpolate(x, y, mlx90640To, useSmoothstep);
        //pixVal = abs(abs(int(y)%2 - int(x)%2)*512 - ((int(x)+1)+(int(y)+1))) / 2;  // DEBUG (checkerboard)
        
        // Use that "pixel" value to populate the sub-pixels (the individual pixels of the display)
        // If upsample == 4, then there are no sub-pixels, just one pixel per value
        int pxlMult = 4 / upsample;
        for (int pxlIndexi=0; pxlIndexi < pxlMult; pxlIndexi++) {
          for (int pxlIndexj=0; pxlIndexj < pxlMult; pxlIndexj++) {
            imageArea[128 * (i * pxlMult + pxlIndexi) + j * pxlMult + pxlIndexj] = pixVal;
          }
        }
      
      }
    }

    // Store the uninterpolated data
    for(int y = 0; y < 24; ++y) {  // Rows count from bottom up
      for(int x = 0; x < 32; x++) {
        scaledPix = constrain((mlx90640To[32 * y + x] - colorLow) / (colorHigh - colorLow) * 255.9, 0.0, 255.0);
        pixelArray[3 * (32 * y + x)] = (uint8_t)scaledPix;  // Store as a byte in BMP buffer
      }
    }
  } 
  if (sensorMode == 1 || sensorMode == 3) {  // Depth Sensor - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw data to screen, optionally upsample
    // Check if distance sensor has data ready, read from it, draw the data to screen
    if (myImager.isDataReady() == true)
    {
      if (myImager.getRangingData(&vl53Data)) //Read distance data into array
      {        
        if (sensorMode == 1) {
          // Take the closest return from each of the 64 zones and map the value
          // to one of the pixels in the "imageArea" array
          // In this case, we aren't fusing thermal and depth, so we don't need to worry about the thermal
          // imager. The depth data is a square grid, 8x8.
          // imageArea is 12,288 pixels, drawn in a 128 by 96 grid.
          // So, I guess we just ignore the 16 pixels on the left, and the 16 on the right,
          // and draw in a 96x96 square. Then each zone gets mapped nicely to 12x12 pixels.
          pxSize = 1;
          uint8_t offst = 16;
          for (int16_t x=0; x < 8*upsample; x++) {
            for (int16_t y=0; y < 8*upsample; y++) {
              float xf = (float) x / (float) upsample + (1.0 / (float) upsample) / 2;
              float yf = (float) y / (float) upsample + (1.0 / (float) upsample) / 2;
              int pxlMult = 12 / upsample;
              for (int16_t i=0; i < pxlMult; i++) {
                for (int16_t j=0; j < pxlMult; j++) {
                  imageArea[(offst + x * pxlMult + i) + ((y * pxlMult + j) * 128)] = interpolate(xf, yf, vl53Nearest, useSmoothstep, 8);
                }
              }
            }
          }
          colorLow = colorLow53;
          colorHigh = colorHigh53;
        } else if (sensorMode == 3) {
          // Populate imageAreaCh2 from the output array, populate it so the depth data aligns with the thermal data.
          // Nothing too complicated here for alignment (this isn't a great approximation)...I'll leave that for post processing.
          // The MLX90640 has a 55x35 degree FoV and returns 32x24 pixels (width x height)
          // 1.71875 degrees per pixel horizontally, and 1.4583 degrees per pixel vertically (something seems wrong here...)
          // This is nominally drawn onto 4x4 pixel squares (1 camera pixel = 4x4 screen pixels) -> 1 pixel = 0.43, 0.36 degrees
          // The VL53L5CX has a 45x45 degree FoV (65 degree diagonal FoV) and 8x8 sensing zones. 5.625 degrees per zone, horizontally and vertically.

          // The imageArea array is drawn as a 96x128 pixel image
          for (int16_t i=0; i < 96; i++) {
            for (int16_t j=0; j < 128; j++) {
              // TODO: Precalculate the below, if we have memory
              // Convert each pixel coordinate (cartesian, origin at upper left) into an angular deflection from center in x and y, from the perspective of the thermal camera.
              // Because both axes are an even number of pixels in length, the center is between four pixels - (63.5, 47.5) - remember indices run from [0, 95] and [0, 127]
              // From the above comment block, angle per-pixel is (0.4297, 0.3646). 
              float angleX = (j - 63.5) * 0.4297;
              float angleY = (i - 47.5) * 0.3646;
              
              // Convert each pixel coordinates from angular deflection from center to cartesian in the frame of the depth camera
              // From the above comment block, the depth camera's FoV is 45x45 degrees, so 22.5 degrees in each direction from center
              float x = angleX / 5.625 + 4;
              float y = angleY / 5.625 + 4;

              // Get the value for the output pixel by interpolating
              imageArea2[j + i * 128] = interpolate(x, y, vl53Nearest, useSmoothstep, 8);
              
            }
          }
        }
      }
    }
    
  } else if (sensorMode == 2 || sensorMode == 4) {
    // Draw data to screen with "intelligent" multi-return per zone processing.
    // (The sensor can return the distances measured to multiple objects per zone, as long as they're at least 600mm apart.)
    // (It's supposed to return up to 4 distances per zone, but I've only gotten that working with 3.)
    // The process is: average all returns for each zone, split each zone into 3x3 pixels (8x8 -> 24x24), do bilinear interpolation to fill the new grid, assign each pixel to the closest detected value in the zone.
    // Check if distance sensor has data ready, read from it, draw the data to screen
    if (myImager.isDataReady() == true)
    {
      if (myImager.getRangingData(&vl53Data)) //Read distance data into array
      {        
        //TODO: Do we need an EWMA option?

        // Average the distances reported for each zone, split each zone into 3x3 pixels, do bilinear interpolation, then assign each pixel the closest detected value in its zone
        // define the size of the input and output grids
        const int INPUT_SIZE = 8;
        const int OUTPUT_SIZE = 8*targetsPerZone;
        
        // input data array that will hold average values
        float inputData[INPUT_SIZE*INPUT_SIZE] = {{0}};  // initialize to 0
        // output data array that will hold interpolated values
        float outputData[OUTPUT_SIZE*OUTPUT_SIZE] = {{0}};  // initialize to 0
        
        // Populate inputData
        // There can be up to 3 returns. The value we want is the 
        // average of all of the valid returns. 
        for (int i=0; i < INPUT_SIZE; i++) {
          for (int j=0; j < INPUT_SIZE; j++) {
            float sum = vl53Data.distance_mm[((j*8) + i)*targetsPerZone+0];
            int count = 1;
            if (vl53Data.distance_mm[((j *8) + i)*targetsPerZone+1] > vl53Data.distance_mm[((j*8) + i)*targetsPerZone+0]) {
              sum += vl53Data.distance_mm[((j*8) + i)*targetsPerZone+0];
              count++;
            }
            if (vl53Data.distance_mm[((j*8) + i)*targetsPerZone+2] > vl53Data.distance_mm[((j*8) + i)*targetsPerZone+0]) {
              sum += vl53Data.distance_mm[((j*8) + i)*targetsPerZone+2];
              count++;
            }

            // The average of all of the returns
            inputData[i+j*8] = sum / count;
          }
        }
        
        // calculate the ratio of the output size to the input size
        float ratio = (float) OUTPUT_SIZE / INPUT_SIZE;
        
        // loop over all the pixels in the output grid
        for (int i = 0; i < OUTPUT_SIZE; i++) {
          for (int j = 0; j < OUTPUT_SIZE; j++) {
            // calculate the corresponding coordinates in the input grid
            // this position is based on the center of the pixel
            float x = (float) i / ratio + (1.0 / ratio) / 2;
            float y = (float) j / ratio + (1.0 / ratio) / 2;
      
            // perform bilinear interpolation and store the result (and flip vertically)
            outputData[i+j*8*targetsPerZone] = interpolate(x, y, inputData, useSmoothstep, 8);
          }
        }
    
        // Find the closest measurement for each point and replace the interpolated value with that measured value. 
        // loop over all the pixels in the output grid
        for (int i = 0; i < OUTPUT_SIZE; i++) {
          for (int j = 0; j < OUTPUT_SIZE; j++) {
            // calculate the corresponding indices in the measurement array
            float x = int((float) i / ratio);
            float y = int((float) j / ratio);
            int index = int(((y*8) + x)*targetsPerZone);
    
            // Pull the measurements associated with this pixel
            float measurement1 = vl53Data.distance_mm[index+0];  // Closest return
            float measurement2 = vl53Data.distance_mm[index+1];  // Second return
            float measurement3 = vl53Data.distance_mm[index+2];  // Furthest return
    
            // Determine which measurement is closest to the pixel (only consider valid measurements)
            float closestMeasurement = measurement1;
            if (abs(outputData[i+j*8*targetsPerZone] - measurement2) < abs(outputData[i+j*8*targetsPerZone] - closestMeasurement) && measurement2 > measurement1) {
              closestMeasurement = measurement2;
            }
            if (abs(outputData[i+j*8*targetsPerZone] - measurement3) < abs(outputData[i+j*8*targetsPerZone] - closestMeasurement) && measurement3 > measurement2) {
              closestMeasurement = measurement3;
            }
    
            // Replace the pixel value with the closest measured value
            outputData[i+j*8*targetsPerZone] = closestMeasurement;
          }
        }

        if (sensorMode == 2) {
          // Populate imageArea from the output array. Interpolate as appropriate.
          // The depth data is a square grid, 8x8, now 24x24 (assuming targetsPerZone = 3).
          // imageArea is 12,288 pixels, drawn in a 128 by 96 grid.
          // We can ignore the 16 pixels on the left, and the 16 on the right,
          // and draw in a 96x96 square. Then the 24x24 grid gets mapped to 4x4 pixels
          // Also, flip it vertically.
          pxSize = 1;
          uint8_t offst = 16;
          for (int16_t x=0; x < 8*targetsPerZone*upsample; x++) {
            for (int16_t y=0; y < 8*targetsPerZone*upsample; y++) {
              float xf = (float) x / (float) upsample + (1.0 / (float) upsample) / 2;
              float yf = (float) y / (float) upsample + (1.0 / (float) upsample) / 2;
              int pxlMult = 4 / upsample;
              for (int16_t i=0; i < pxlMult; i++) {
                for (int16_t j=0; j < pxlMult; j++) {
                  imageArea[(offst + x * pxlMult + i) + (((8*targetsPerZone*upsample-y-1) * pxlMult + j) * 128)] = interpolate(xf, yf, outputData, useSmoothstep, 8*targetsPerZone);
                }
              }
            }
          }
          
          colorLow = colorLow53;
          colorHigh = colorHigh53;
        } else if (sensorMode == 4) {
          // Populate imageAreaCh2 from the output array, populate it so the depth data aligns with the thermal data.
          // Nothing too complicated here for alignment (this isn't a great approximation)...I'll leave that for post processing.
          // The MLX90640 has a 55x35 degree FoV and returns 32x24 pixels (width x height)
          // 1.71875 degrees per pixel horizontally, and 1.4583 degrees per pixel vertically (something seems wrong here...)
          // This is nominally drawn onto 4x4 pixel squares (1 camera pixel = 4x4 screen pixels) -> 1 pixel = 0.43, 0.36 degrees
          // The VL53L5CX has a 45x45 degree FoV (65 degree diagonal FoV) and 8x8 sensing zones. 5.625 degrees per zone, horizontally and vertically.
          // Each zone has been split into 3x3, so 1.875 degrees per chunk of pixels.

          // The imageArea array is drawn as a 96x128 pixel image
          for (int16_t i=0; i < 96; i++) {
            for (int16_t j=0; j < 128; j++) {
              // TODO: Precalculate the below, if we have memory
              // Convert each pixel coordinate (cartesian, origin at upper left) into an angular deflection from center in x and y, from the perspective of the thermal camera.
              // Because both axes are an even number of pixels in length, the center is between four pixels - (63.5, 47.5) - remember indices run from [0, 95] and [0, 127]
              // From the above comment block, angle per-pixel is (0.4297, 0.3646). 
              float angleX = (j - 63.5) * 0.4297;
              float angleY = (i - 47.5) * 0.3646;
              
              // Convert each pixel coordinates from angular deflection from center to cartesian in the frame of the depth camera
              // From the above comment block, the depth camera's FoV is 45x45 degrees, so 22.5 degrees in each direction from center
              float degPerPx = 22.5 / (8 * targetsPerZone / 2);
              float x = angleX / degPerPx + 8 * targetsPerZone / 2;
              float y = angleY / degPerPx + 8 * targetsPerZone / 2;

              // Get the value for the output pixel by interpolating
              imageArea2[j + (95 - i) * 128] = interpolate(x, y, outputData, useSmoothstep, 8*targetsPerZone);
              
            }
          }
        }
      }
    }
  } 
  
  // The dual-channel (thermal+depth) modes will be drawn into the red and blue channels, rather than use the existing colormaps.
  // colorPal is an array of uint16_t's, in each value the top 5 bits define red, the middle 6 define green, and the bottom 5 bits define blue
  if (sensorMode > 2) {
    // Dual channel mode    
    // Draw the image from the single-pixel array
    if(mirrorFlag) {                 // Mirrored display (selfie mode)?
      for(int y = 0; y < 24*4; ++y) {  // Rows count from bottom up
        for(int x = 0; x < 32*4; x++) {
          scaledPix = ((uint16_t)constrain((imageArea[128 * y + x] - colorLow) / (colorHigh - colorLow) * 31.9, 0.0, 31.0)) << 11;  // Temperature
          // Invert the colors for distance - I just prefer closer = brighter (and hotter = brighter, which is default), so this has to be inverted
          scaledPix += 31.0 - (uint16_t)constrain((imageArea2[128 * y + x] - colorLow53) / (colorHigh53 - colorLow53) * 31.9, 0.0, 31.0);
          
          // fillRect(x0, y0, w, h, color); (x0, y0) is the upper left corner.
          //arcada.display->fillRect(140 - x, 92 - y * 1, 1, 1, (uint16_t)scaledPix);  // Filled rectangles, bottom up
          //original, works as expected scaledPix = constrain((imageArea[128 * y + x] - colorLow) / (colorHigh - colorLow) * 255.9, 0.0, 255.0);
          //original, works as expected arcada.display->fillRect(140 - x, 92 - y * pxSize, 1, pxSize, colorPal[(uint16_t)scaledPix]);  // Filled rectangles, bottom up
          //arcada.display->fillRect(140 - x, 92 - y * pxSize, 1, pxSize, scaledPix);  // Filled rectangles, bottom up
          arcada.display->drawPixel(140 - x, 95 - y, scaledPix);  // Fill pixels from the bottom up
          //arcada.display->fillRect(140 - x, 92 - y, 1, 1, scaledPix);  // Filled rectangles, bottom up
          // TODO: Replace all fillRects for single pixels with drawPixel(140 - x, 92 - y, (uint16_t)scaledPix); <- presumably it's faster than fillRect
    
        }
      }
    } else {  // Not mirrored
      for(int y = 0; y < 96; ++y) {
        for(int x = 0; x < 128; x++) {
  
          scaledPix = ((uint16_t)constrain((imageArea[128 * y + x] - colorLow) / (colorHigh - colorLow) * 31.9, 0.0, 31.0)) << 11;
          // Invert the colors for distance - I just prefer closer = brighter (and hotter = brighter, which is default), so this has to be inverted
          scaledPix += 31.0 - (uint16_t)constrain((imageArea2[128 * y + x] - colorLow53) / (colorHigh53 - colorLow53) * 31.9, 0.0, 31.0);
          // fillRect(x0, y0, w, h, color); (x0, y0) is the upper left corner.
          //arcada.display->fillRect(16 + x, 92 - y * 1, 1, 1, (uint16_t)scaledPix);
          //original, works as expected scaledPix = constrain((imageArea[128 * y + x] - colorLow) / (colorHigh - colorLow) * 255.9, 0.0, 255.0);
          //original, works as expected arcada.display->fillRect(16 + x, 92 - y * pxSize, 1, pxSize, colorPal[(uint16_t)scaledPix]);
          //arcada.display->fillRect(16 + x, 92 - y * pxSize, 1, pxSize, scaledPix);
          //arcada.display->drawPixel(16 + x, 95 - y, colorPal[(uint16_t)scaledPix]);  // Woops! We don't want to use the color palette here, but if we do...it looks kinda neat actually
          arcada.display->drawPixel(16 + x, 95 - y, scaledPix);  // Fill pixels from the bottom up
          //pxSize in 'y0' does in fact change where pixels are drawn - if I make it 1 when it expects 4, the pixels get squished down into the lower left corner (1/4 of the screen vertically and horizontally)
          //pxSize in 'h' does control the height of the pixels - set to 1 it only draws every 4th line
          //Now, what's going on that I have height set by pxSize, but not width? How does it draw correctly across the width dimension?
          //arcada.display->fillRect(16 + x, 92 - y, 1, 1, scaledPix);
          
        }
      }
    }
    
  } else {
    // Single channel, use the color palette setting
    // Draw the image from the single-pixel array
    if(mirrorFlag) {                 // Mirrored display (selfie mode)?
      for(int y = 0; y < 96; ++y) {  // Rows count from bottom up
        for(int x = 0; x < 128; x++) {
        
          scaledPix = constrain((imageArea[128 * y + x] - colorLow) / (colorHigh - colorLow) * 255.9, 0.0, 255.0);
          // Invert the colors for distance - I just prefer closer = brighter (and hotter = brighter, which is default), so this has to be inverted
          if (sensorMode == 1 || sensorMode == 2) {
            scaledPix = 255.0 - scaledPix;
          }
          // fillRect(x0, y0, w, h, color); (x0, y0) is the upper left corner.
          arcada.display->drawPixel(140 - x, 95 - y, colorPal[(uint16_t)scaledPix]);  // Fill pixels from the bottom up
    
        }
      }
    } else {  // Not mirrored
      for(int y = 0; y < 96; y++) {
        for(int x = 0; x < 128; x++) {
          scaledPix = constrain((imageArea[128 * y + x] - colorLow) / (colorHigh - colorLow) * 255.9, 0.0, 255.0);
          // Invert the colors for distance - I just prefer closer = brighter
          if (sensorMode == 1 || sensorMode == 2) {
            scaledPix = 255.0 - scaledPix;
          }
          // fillRect(x0, y0, w, h, color); (x0, y0) is the upper left corner. Origin (0, 0) is the upper left corner of the screen.
          //arcada.display->drawPixel(16 + x, 95 - y, colorPal[(uint16_t)(abs(abs(y%2 - x%2)*255 - (x+y)))]);  // DEBUG
          arcada.display->drawPixel(16 + x, 95 - y, colorPal[(uint16_t)scaledPix]);  // Fill pixels from the bottom up
        }
      }
    }
  }

  // Post pass: Screen print the lowest, center, and highest temperatures
  arcada.display->fillRect(  0, 96, 53, 12, colorPal[0]);                  // Contrasting mini BGs for cold temp
  arcada.display->fillRect(107, 96, 53, 12, colorPal[255]);                // and for hot temperature texts
  scaledPix = constrain((mlx90640To[400] - colorLow) / (colorHigh - colorLow) * 255.9, 0.0, 255.0);
  arcada.display->fillRect(53, 96, 54, 12, colorPal[(uint16_t)scaledPix]); // Color coded mini BG for center temp

  arcada.display->setTextSize(1);
  arcada.display->setCursor(10, 99);
  arcada.display->setTextColor(0xFFFF ^ colorPal[0]);   // Contrasting text color for coldest/nearest value
  if (sensorMode == 0 | sensorMode > 2) {
    arcada.display->print(celsiusFlag ? lowPix : lowPix * 1.8 + 32.0);  // Print Celsius or Fahrenheit
  } else {
    arcada.display->print(lowPix53);
  }

  arcada.display->setCursor(120, 99);
  arcada.display->setTextColor(0xFFFF ^ colorPal[255]); // Contrast text for hottest/furthest value
  if (sensorMode == 0 | sensorMode > 2) {
    arcada.display->print(celsiusFlag ? highPix : highPix * 1.8 + 32.0);  // Print Celsius or Fahrenheit
  } else {
    arcada.display->print(highPix53);
  }

  arcada.display->setCursor(65, 99);
  if((mlx90640To[400] < (colorLow + colorHigh) * 0.5) == (paletteNum < 3))
    arcada.display->setTextColor(0xFFFF);               // A contrasting text color for center temp/distance
  else
    arcada.display->setTextColor(0x0000);
    
  if (sensorMode == 0 | sensorMode > 2) {
    arcada.display->print(celsiusFlag ? mlx90640To[400] : mlx90640To[400] * 1.8 + 32.0);  // Pixel 12 * 32 + 16
  } else {
    arcada.display->print(vl53Data.distance_mm[32*targetsPerZone]);
  }

  markColor = 0x0600;    // Deep green color to draw onscreen cross markers
  if(markersOn) {        // Show markers?
    if(mirrorFlag) {     // ...over a mirrored display?
      arcada.display->drawFastHLine(156 - (( lowAddr % 32) * 4 + 16), 93 - 4 * ( lowAddr / 32), 4, markColor); // Color crosses mark cold pixel,
      arcada.display->drawFastVLine(159 - (( lowAddr % 32) * 4 + 17), 92 - 4 * ( lowAddr / 32), 4, markColor);
      arcada.display->drawFastHLine(156 - ((highAddr % 32) * 4 + 16), 93 - 4 * (highAddr / 32), 4, markColor); // hot pixel,
      arcada.display->drawFastVLine(159 - ((highAddr % 32) * 4 + 17), 92 - 4 * (highAddr / 32), 4, markColor);
      arcada.display->drawFastHLine(76, 45, 4, markColor);                                                     // and center pixel
      arcada.display->drawFastVLine(78, 44, 4, markColor);
    } else {             // Not mirrored
      arcada.display->drawFastHLine(( lowAddr % 32) * 4 + 16, 93 - 4 * ( lowAddr / 32), 4, markColor); // Color crosses mark cold pixel,
      arcada.display->drawFastVLine(( lowAddr % 32) * 4 + 17, 92 - 4 * ( lowAddr / 32), 4, markColor);
      arcada.display->drawFastHLine((highAddr % 32) * 4 + 16, 93 - 4 * (highAddr / 32), 4, markColor); // hot pixel,
      arcada.display->drawFastVLine((highAddr % 32) * 4 + 17, 92 - 4 * (highAddr / 32), 4, markColor);
      arcada.display->drawFastHLine(80, 45, 4, markColor);                                             // and center pixel
      arcada.display->drawFastVLine(81, 44, 4, markColor);
    }
  }

// Print the frame count on the left sidebar
  arcada.display->setRotation(0);    // Vertical printing
  arcada.display->setCursor(48, 4);
  arcada.display->setTextColor(0xFFFF, backColor); // White text, current BG
  arcada.display->print("FRM ");
  arcada.display->print(++frameCounter);
  arcada.display->setCursor(96, 4);
  arcada.display->print("UP ");
  arcada.display->print(upsample);
  arcada.display->setRotation(1);    // Back to horizontal

// Handle any button presses
  if(!buttonActive && clickFlagMenu) {         // Was B:MENU button pressed?
    buttonActive = true;                       // Set button flag
    deBounce = millis() + DE_BOUNCE;           // and start debounce timer
    menuLoop(backColor);                       // Execute menu routine until finished
    clickFlagSelect = recordingInProg = false; // Clear unneeded flags
    nextFrameSequence = 1;
    setBackdrop(backColor, buttonRfunc);       // Repaint current BG & button labels
  }
  
  if(!buttonActive && clickFlagUpsample) {
    buttonActive = true;                       // Set button flag
    deBounce = millis() + DE_BOUNCE;           // and start debounce timer
    if (upsample >= 4) {
      upsample = 1;
    } else if (upsample >= 2) {
      upsample = 4;  // Skip 3 because it doesn't really fit in the screen space we have alotted
    } else {
      upsample++;
    }
  }

  if(!buttonActive && clickFlagSmoothstep) {
    buttonActive = true;                       // Set button flag
    deBounce = millis() + DE_BOUNCE;           // and start debounce timer
    if (useSmoothstep) {
      useSmoothstep = false;
    } else {
      useSmoothstep = true;
    }
  }

  if (!buttonActive && clickFlagSwitchMode) {
    buttonActive = true;                       // Set button flag
    deBounce = millis() + DE_BOUNCE;           // and start debounce timer
    sensorMode++;
    if (sensorMode > 4) {
      sensorMode = 0;
    }
  }

  if(!buttonActive && clickFlagSelect) { // Was the A button pressed?
    buttonActive = true;                 // Set button flag
    deBounce = millis() + DE_BOUNCE;     // and start debounce timer

    if(buttonRfunc == 0) {                                           // Freeze requested?
      arcada.display->drawBitmap(146, 48, snowIcon, 16, 12, 0xC61F); // Freeze icon on
      while(buttonBits & ARCADA_BUTTONMASK_A)                        // Naive freeze: loop until button released
        delay(10);                                                   // Short pause
      deBounce = millis() + DE_BOUNCE;                               // Restart debounce timer
      arcada.display->fillRect(146, 48, 12, 12, backColor);          // Freeze icon off
      
    } else if(buttonRfunc == 1) {                         // Capture requested?
      // Get the next available index, taking into consideration CSV and BMP files
      nextFileIndex = availableFileNumber(nextFileIndex, BOTTOM_DIR + String(BMP_FORMAT));
      uint16_t nextFileIndexTemp = availableFileNumber(nextFileIndex, BOTTOM_DIR_CSV + String(CSV_FORMAT));
      if (nextFileIndexTemp > nextFileIndex) {
        nextFileIndex = nextFileIndexTemp;
      }
      if(nextFileIndex != 0) { // Serialized filename available?
        save1frame = true;                                // Set the flag to save file(s)
        arcada.display->fillRect(0, 96, 160, 12, 0x0600); // Display a green strip
        arcada.display->setTextColor(0xFFFF);             // with white capture message text
        arcada.display->setCursor(16, 99);
        arcada.display->print("Saving frame ");
        arcada.display->print(nextFileIndex);
      } else {
        arcada.display->fillRect(0, 96, 160, 12, 0xF800);    // Red error signal
        arcada.display->setTextColor(0xFFFF);                // with white text
        arcada.display->setCursor(20, 99);
        arcada.display->print("No filename available!");
      }
    } else {                          // Begin or halt recording a sequence of BMP files
      if(!recordingInProg) {            // "A:START RECORDING" was pressed
        nextDirIndex = availableFileNumber(nextDirIndex, BOTTOM_DIR + String(DIR_FORMAT));
        uint16_t nextDirIndexTemp = availableFileNumber(nextDirIndex, BOTTOM_DIR_CSV + String(DIR_FORMAT));
        if (nextDirIndexTemp > nextDirIndex) {
          nextDirIndex = nextDirIndexTemp;
        }
        if(nextDirIndex != 0) { // Serialized directory name available?
          // Make the directory
          if(newDirectory()) {          // Success in making a new sequence directory?
            recordingInProg = true;     // Set the flag for saving BMP files
            nextFrameSequence = 1;        // ...numbered starting with 00001
            setBackdrop(backColor, 3);  // Show "A:STOP RECORDING" label
          } else                        // Couldn't make the new directory, so
            nextDirIndex = 0;           // disable further sequences
        }
      } else {                          // "A:STOP RECORDING" was pressed
        recordingInProg = false;
        setBackdrop(backColor, 2);      // Clear "A:STOP RECORDING" label
      }
    }
  }

  // Saving any BMP images to flash media happens here
  if(save1frame || recordingInProg) {      // Write a file to SD?
    arcada.display->drawBitmap(146, 32, SDicon, 16, 12, 0x07E0); // Flash storage activity icon on

    prepForSave();                         // Save to flash.  Use global values for parameters
    nextFrameSequence += recordingInProg ? 1 : 0;  // If recording a series, increment frame count
    save1frame = false;                    // If one frame saved, clear the flag afterwards

    arcada.display->fillRect(146, 32, 12, 12, backColor);        // Flash storage activity icon off
  }

  if(showLastCap) {                      // Redisplay the last BMP saved?
    buttonActive = true;                 // Set button flag
    deBounce = millis() + DE_BOUNCE;     // and start debounce timer
    recallLastBMP(backColor);            // Redisplay last bitmap from buffer until finished
    setBackdrop(backColor, buttonRfunc); // Repaint current BG & button labels
    showLastCap = false;
  }

// Here we protect against button bounces while the function loops
  if(buttonActive && millis() > deBounce && (buttonBits
     & (ARCADA_BUTTONMASK_B | ARCADA_BUTTONMASK_A)) == 0)  // Has de-bounce wait expired & all buttons released?
    buttonActive = false;                // Clear flag to allow another button press

  clickFlagMenu = clickFlagSelect = clickFlagUpsample = clickFlagSmoothstep = clickFlagSwitchMode = false; // End of the loop, clear all interrupt flags
}

// Compute and fill an array with 256 16-bit color values
void loadPalette(uint16_t palNumber) {
  uint16_t x, y;
  float fleX, fleK;

  switch(palNumber) {
    case 1:  // Compute ironbow palette
      for(x = 0; x < 256; ++x) {
        fleX = (float)x / 255.0;

        // fleK = 65535.9 * (1.02 - (fleX - 0.72) * (fleX - 0.72) * 1.96);
        // fleK = (fleK > 65535.0) || (fleX > 0.75) ? 65535.0 : fleK;  // Truncate red curve
        fleK = 63487.0 * (1.02 - (fleX - 0.72) * (fleX - 0.72) * 1.96);
        fleK = (fleK > 63487.0) || (fleX > 0.75) ? 63487.0 : fleK;  // Truncate red curve
        colorPal[x] = (uint16_t)fleK & 0xF800;                      // Top 5 bits define red

        // fleK = fleX * fleX * 2047.9;
        fleK = fleX * fleX * 2015.0;
        colorPal[x] += (uint16_t)fleK & 0x07E0;  // Middle 6 bits define green

        // fleK = 31.9 * (14.0 * (fleX * fleX * fleX) - 20.0 * (fleX * fleX) + 7.0 * fleX);
        fleK = 30.9 * (14.0 * (fleX * fleX * fleX) - 20.0 * (fleX * fleX) + 7.0 * fleX);
        fleK = fleK < 0.0 ? 0.0 : fleK;          // Truncate blue curve
        colorPal[x] += (uint16_t)fleK & 0x001F;  // Bottom 5 bits define blue
      }
      break;
    case 2:  // Compute quadratic "firebow" palette
      for(x = 0; x < 256; ++x) {
        fleX = (float)x / 255.0;

        // fleK = 65535.9 * (1.00 - (fleX - 1.0) * (fleX - 1.0));
        fleK = 63487.0 * (1.00 - (fleX - 1.0) * (fleX - 1.0));
        colorPal[x] = (uint16_t)fleK & 0xF800;                      // Top 5 bits define red

        // fleK = fleX < 0.25 ? 0.0 : (fleX - 0.25) * 1.3333 * 2047.9;
        fleK = fleX < 0.25 ? 0.0 : (fleX - 0.25) * 1.3333 * 2015.0;
        colorPal[x] += (uint16_t)fleK & 0x07E0;  // Middle 6 bits define green

        // fleK = fleX < 0.5 ? 0.0 : (fleX - 0.5) * (fleX - 0.5) * 127.9;
        fleK = fleX < 0.5 ? 0.0 : (fleX - 0.5) * (fleX - 0.5) * 123.0;
        colorPal[x] += (uint16_t)fleK & 0x001F;  // Bottom 5 bits define blue
      }
      break;
    case 3:  // Compute "alarm" palette
      for(x = 0; x < 256; ++x) {
        fleX = (float)x / 255.0;

        fleK = 65535.9 * (fleX < 0.875 ? 1.00 - (fleX * 1.1428) : 1.0);
        colorPal[x] = (uint16_t)fleK & 0xF800;                      // Top 5 bits define red

        fleK = 2047.9 * (fleX < 0.875 ? 1.00 - (fleX * 1.1428) : (fleX - 0.875) * 8.0);
        colorPal[x] += (uint16_t)fleK & 0x07E0;  // Middle 6 bits define green

        fleK = 31.9 * (fleX < 0.875 ? 1.00 - (fleX * 1.1428) : 0.0);
        colorPal[x] += (uint16_t)fleK & 0x001F;  // Bottom 5 bits define blue
      }
      break;
    case 4:  // Compute negative gray palette, black hot
      for(x = 0; x < 256; ++x)
        colorPal[255 - x] = (((uint16_t)x << 8) & 0xF800) + (((uint16_t)x << 3) & 0x07E0) + (((uint16_t)x >> 3) & 0x001F);
      break;
    default:  // Compute gray palette, white hot
      for(x = 0; x < 256; ++x)
        colorPal[x] = (((uint16_t)x << 8) & 0xF800) + (((uint16_t)x << 3) & 0x07E0) + (((uint16_t)x >> 3) & 0x001F);
      break;
  }
}

void setColorRange(int presetIndex) { // Set coldest/hottest values in color range
  switch(presetIndex) {
    case 1:  // Standard range, from FLIR document: 50F to 90F
      colorLow = 10.0;
      colorHigh = 32.22;
      break;
    case 2:  // Cool/warm range, for detecting mammals outdoors
      colorLow = 5.0;
      colorHigh = 32.0;
      break;
    case 3:  // Warm/warmer range, for detecting mammals indoors
      colorLow = 20.0;
      colorHigh = 32.0;
      break;
    case 4:  // Hot spots, is anything hotter than it ought to be?
      colorLow = 20.0;
      colorHigh = 50.0;
      break;
    case 5:  // Fire & ice, extreme temperatures only!
      colorLow = -10.0;
      colorHigh = 200.0;
      break;
    default:  // Default is autorange, so these values will change with every frame
      colorLow = 0.0;
      colorHigh = 100.0;
      break;
  }
}

// Draw the stationary screen elements behind the live camera window
void setBackdrop(uint16_t bgColor, uint16_t buttonFunc) {
  arcada.display->fillScreen(bgColor);

  for(int x = 0; x < 160; ++x)   // Paint current palette across bottom
    arcada.display->drawFastVLine(x, 110, 6, colorPal[map(x, 0, 159, 0, 255)]);

  arcada.display->setCursor(16, 120);
  arcada.display->setTextColor(0xFFFF, bgColor);    // White text, current BG for button labels
  switch(buttonFunc) {
    case 0:
      arcada.display->print("B:MENU        A:FREEZE");
      break;
    case 1:
      arcada.display->print("B:MENU       ");
      if(nextFileIndex == 0)                         // No room to store a file in flash media?
        arcada.display->setTextColor(GRAY_33 >> 1); // Grayed button label
      arcada.display->print("A:CAPTURE");
      break;
    case 2:
      arcada.display->print("B:MENU    ");
      if(nextDirIndex == 0)                         // Has flash storage no room for a new directory?
        arcada.display->setTextColor(GRAY_33 >> 1); // Grayed button label
      arcada.display->print("A:START RECORD");
      break;
    case 3:
      arcada.display->print("B:MENU    ");
      arcada.display->setTextColor(0xFFFF, 0xF800);  // White text, red BG recording indicator
      arcada.display->print("A:STOP RECORD");
      break;
    case 4:
      arcada.display->print("               A:EXIT"); // Use for bitmap redisplay only
      break;
  }
}

void prepForSave() {
  for(int x = 0; x < 768; ++x)
    pixelArray[3 * x + 2] = pixelArray[3 * x + 1] = pixelArray[3 * x];  // Copy each blue byte into R & G for 256 grays in 24 bits

  if (sensorMode == 0 || sensorMode > 2) {
    if(!writeBMP()) {                                      // Did BMP write to flash fail?
      arcada.display->fillRect(0, 96, 160, 12, 0xF800);    // Red error signal
      arcada.display->setTextColor(0xFFFF);                // with white text
      arcada.display->setCursor(20, 99);
      arcada.display->print("Storage error (bmp)!");
    }
    if(!writeThermalCSV()) {                                      // Did csv write to flash fail?
      arcada.display->fillRect(0, 96, 160, 12, 0xF800);    // Red error signal
      arcada.display->setTextColor(0xFFFF);                // with white text
      arcada.display->setCursor(20, 99);
      arcada.display->print("Storage error (csv)!");
    }
  }

  if (sensorMode > 0) {
    // Write the vl53All array to file as a CSV
    if(!writeCSV()) {                                      // Did CSV write to flash fail?
      arcada.display->fillRect(0, 96, 160, 12, 0xF800);    // Red error signal
      arcada.display->setTextColor(0xFFFF);                // with white text
      arcada.display->setCursor(20, 99);
      arcada.display->print("Storage error (csv)!");
    }
  }
}

boolean newDirectory() { // Create a subdirectory, converting the name between char arrays and string objects
  char fileArray[64];
  String fullPath;

  sprintf(fileArray, DIR_FORMAT, nextDirIndex); // Generate subdirectory name
  fullPath = BOTTOM_DIR + String(fileArray);    // Make a filepath out of it, then
  return arcada.mkdir(fullPath.c_str());        // try to make a real subdirectory from it
}

boolean writeCSV() {
  String fullPath;
  char fileArray[64];

  // First, figure out a name and path for our new CSV
  fullPath = BOTTOM_DIR_CSV;                          // Build a filepath starting with the base subdirectory
  if(buttonRfunc == 2) {                              // CSV sequence recording in progress?
    sprintf(fileArray, DIR_FORMAT, nextDirIndex);     // Generate subdirectory name
    fullPath += String(fileArray);                    // Add it to the path
    sprintf(fileArray, CSV_FORMAT, nextFrameSequence);  // Generate a sequential filename
    fullPath += String(fileArray);                    // Complete the filepath string
    
  } else {                                            // Not a sequence, solitary CSV file
    sprintf(fileArray, CSV_FORMAT, nextFileIndex);     // Generate a serial filename
    fullPath += String(fileArray);                    // Complete the filepath string
  }

  myFile = arcada.open(fullPath.c_str(), FILE_WRITE); // Only one file can be open at a time

  if(myFile) {                      // If the file opened okay, write to it:
    for (int target=0; target < 64; target++) {
      for (int zone=0; zone < targetsPerZone; zone++) {
        // https://stackoverflow.com/questions/41394063/how-to-simply-convert-a-float-to-a-string-in-c
        int len = snprintf(NULL, 0, "%f", vl53All[target*targetsPerZone+zone]);  // Determine the length of the value in floats
        char *result = (char*)malloc(sizeof(char) * ( len + 1 ));  // Allocate memory for a character array
        snprintf(result, len + 1, "%f", vl53All[target*targetsPerZone+zone]);  // Convert the float and copy to the character array
        myFile.write(result, len);  // I think this is myFile.write(array of data, size in bytes);
        myFile.write(",", 1);
      }
      myFile.write("\n", 1);
    }
    myFile.close();
    return true;
  } else {          // The file didn't open, return error
    return false;
  }

}

boolean writeThermalCSV() {
  // Write a csv file of the data from the MLX
  String fullPath;
  char fileArray[64];

  // First, figure out a name and path for our new CSV
  fullPath = BOTTOM_DIR;                          // Build a filepath starting with the base subdirectory
  if(buttonRfunc == 2) {                              // CSV sequence recording in progress?
    sprintf(fileArray, DIR_FORMAT, nextDirIndex);     // Generate subdirectory name
    fullPath += String(fileArray);                    // Add it to the path
    sprintf(fileArray, CSV_FORMAT, nextFrameSequence);  // Generate a sequential filename
    fullPath += String(fileArray);                    // Complete the filepath string
    
  } else {                                            // Not a sequence, solitary CSV file
    sprintf(fileArray, CSV_FORMAT, nextFileIndex);     // Generate a serial filename
    fullPath += String(fileArray);                    // Complete the filepath string
  }

  myFile = arcada.open(fullPath.c_str(), FILE_WRITE); // Only one file can be open at a time

  if(myFile) {                      // If the file opened okay, write to it:
    for(int y = 0; y < 24; ++y) {  // Rows count from bottom up
      for(int x = 0; x < 32; x++) {
        // https://stackoverflow.com/questions/41394063/how-to-simply-convert-a-float-to-a-string-in-c
        int len = snprintf(NULL, 0, "%f", mlx90640To[32 * y + x]);  // Determine the length of the value in floats
        char *result = (char*)malloc(sizeof(char) * ( len + 1 ));  // Allocate memory for a character array
        snprintf(result, len + 1, "%f", mlx90640To[32 * y + x]);  // Convert the float and copy to the character array
        myFile.write(result, len);  // I think this is myFile.write(array of data, size in bytes);
        myFile.write(",", 1);
      }
      myFile.write("\n", 1);
    }
    myFile.close();
    return true;
  } else {          // The file didn't open, return error
    return false;
  }
}

// Here we write the actual bytes of a BMP file (plus extras) to flash media
boolean writeBMP() {
  uint16_t counter1, shiftedFloats[14]; // A buffer for the appended floats and uint16_t's
  uint32_t timeStamp;
  float shiftAssist;
  char fileArray[64];
  String fullPath;

  // First, figure out a name and path for our new BMP
  fullPath = BOTTOM_DIR;                              // Build a filepath starting with the base subdirectory
  if(buttonRfunc == 2) {                              // BMP sequence recording in progress?
    sprintf(fileArray, DIR_FORMAT, nextDirIndex);     // Generate subdirectory name
    fullPath += String(fileArray);                    // Add it to the path
    sprintf(fileArray, BMP_FORMAT, nextFrameSequence);  // Generate a sequential filename
    fullPath += String(fileArray);                    // Complete the filepath string
  } else {                                            // Not a sequence, solitary BMP file
    sprintf(fileArray, BMP_FORMAT, nextFileIndex);     // Generate a serial filename
    fullPath += String(fileArray);                    // Complete the filepath string
  }

  myFile = arcada.open(fullPath.c_str(), FILE_WRITE); // Only one file can be open at a time

  if(myFile) {                      // If the file opened okay, write to it:
    myFile.write(BmpPSPHead, 14);   // BMP header 1
    myFile.write(DIBHeadPSP1, 40);  // BMP header 2
    myFile.write(pixelArray, 2304); // Array of 768 BGR byte triples
    myFile.write(PSPpad, 2);        // Pad with 2 zeros 'cause Photoshop does it.

    // My BMP hack - append 5 fixed-point temperature values as 40 extra bytes
    for(counter1 = 0; counter1 < 5; ++counter1) {    // Shift 5 floats
      shiftAssist = sneakFloats[counter1] + 1000.0;  // Offset MLX90640 temps to positive
      shiftedFloats[counter1 * 2] = (uint16_t)shiftAssist;
      shiftAssist = (shiftAssist - (float)shiftedFloats[counter1 * 2]) * 49152.0; // Scale up fraction
      shiftedFloats[counter1 * 2 + 1] = (uint16_t)shiftAssist;
    }

    shiftedFloats[10] = lowAddr;   // Two more appended numbers, the 2 extreme pixel addresses
    shiftedFloats[11] = highAddr;

    timeStamp = millis();         // Recycle this variable to append a time stamp
    lowAddr = timeStamp & 0xFFFF;
    highAddr = timeStamp >> 16;
    shiftedFloats[12] = lowAddr;
    shiftedFloats[13] = highAddr;

    myFile.write(shiftedFloats, 28);  // Write appended uint16_t's

    myFile.close();
    return true;
  } else {          // The file didn't open, return error
    return false;
  }
}

void recallLastBMP(uint16_t bgColor) {  // Display 8-bit values left in buffer from the last BMP save
  int counter1, counter2;
  boolean exitFlag = false;

  setBackdrop(bgColor, 4);  // Clear screen, just a color palette & "A:EXIT" in the BG

  for(int counter1 = 0; counter1 < 24; ++counter1) {  // Redraw using leftover red byte values, not yet overwritten
    for(int counter2 = 0 ; counter2 < 32 ; ++counter2) {
      arcada.display->fillRect(16 + counter2 * 4, 92 - counter1 * 4, 4, 4,
                   colorPal[(uint16_t)pixelArray[3 * (32 * counter1 + counter2) + 2]]);
    }
  }

  while(!exitFlag) {  // Loop here until exit button
    if(!buttonActive && (buttonBits & ARCADA_BUTTONMASK_A)) { // "A:EXIT" button freshly pressed?
      exitFlag = true;
      buttonActive = true;
      deBounce = millis() + DE_BOUNCE;
    }

    if(buttonActive && millis() > deBounce
       && (buttonBits & (ARCADA_BUTTONMASK_A | ARCADA_BUTTONMASK_B)) == 0)  // Has de-bounce wait expired & all buttons released?
      buttonActive = false;               // Clear flag to allow another button press
  }
}

uint16_t availableFileNumber(uint16_t startNumber, String formatBase) { // Find unclaimed serial number for file series
  uint16_t counter1;
  char nameArray[80];

  for(counter1 = startNumber; counter1 % MAX_SERIAL != 0; ++counter1) { // Start counting
    sprintf(nameArray, formatBase.c_str(), counter1);                   // Generate a serialized filename
    if(!arcada.exists(nameArray))                                       // If it doesn't already exist
      return counter1;                                                  // return the number as available
  }
  return 0; // Loop finished, no free number found, return fail
}

boolean menuLoop(uint16_t bgColor) {  // Lay out a menu screen, interact to change values
  int counter1 = 0, scrollPosition = 0;
  boolean exitFlag = false, settingsChanged = false;
  uint32_t menuButtons;

  arcada.display->fillScreen(bgColor);
  arcada.display->fillRect(0, 12 * (counter1 + scrollPosition) + MENU_VPOS - 2, 160, 12, 0x0000);  // Black stripe cursor on menu

  arcada.display->setTextColor(0xFFFF);             // White text
  arcada.display->setCursor(16, 120);               // at screen bottom
  arcada.display->print("B:ADVANCE     A:CHANGE");  // for button labels

  for(counter1 = 0; counter1 < MENU_ROWS; ++counter1) {  // Display menu texts
    menuLines(counter1, scrollPosition);
  }
  counter1 = 0;

  while(!exitFlag) { // Loop until exit is activated
    if(!buttonActive && (buttonBits & ARCADA_BUTTONMASK_B)) {  // Fresh press of B:ADVANCE button?
      buttonActive = true;                                     // Set button flag
      deBounce = millis() + DE_BOUNCE;                         // and start debounce timer.

      arcada.display->fillRect(0, 12 * (counter1 - scrollPosition) + MENU_VPOS - 2, 160, 12, bgColor); // Erase cursor & text
      menuLines(counter1, scrollPosition);                     // Refresh menu text line
      counter1 = (counter1 + 1) % MENU_LEN;                    // Advance menu counter

      if(counter1 == 0) {                                      // Have we cycled around to the menu top?
        scrollPosition = 0;
        for(int counter2 = 0; counter2 < MENU_ROWS; ++counter2) {  // Redisplay all menu texts
          arcada.display->fillRect(0, 12 * counter2 + MENU_VPOS - 2, 160, 12, bgColor); // Erase old text
          menuLines(counter2 + scrollPosition, scrollPosition);    // Redraw each text line
        }
      } else if((counter1 + 1 < MENU_LEN) && (counter1 - scrollPosition == MENU_ROWS - 1)) { // Should we scroll down 1 menu line?
        ++scrollPosition;
        for(int counter2 = 0; counter2 < MENU_ROWS; ++counter2) {  // Redisplay all menu texts
          arcada.display->fillRect(0, 12 * counter2 + MENU_VPOS - 2, 160, 12, bgColor); // Erase old text
          menuLines(counter2 + scrollPosition, scrollPosition);    // Redraw each text line
        }
      }

      arcada.display->fillRect(0, 12 * (counter1 - scrollPosition) + MENU_VPOS - 2, 160, 12, 0x0000);  // New black cursor
      menuLines(counter1, scrollPosition);                     // Refresh text line
      deBounce = millis() + DE_BOUNCE;                         // Restart debounce timer, just for safety
    }

    if(!buttonActive && (buttonBits & ARCADA_BUTTONMASK_A)) {  // Fresh press of A:CHANGE button?
      buttonActive = true;                                     // Set button flag
      deBounce = millis() + DE_BOUNCE;                         // and start debounce timer.
      bool setSharp = false;
      int tmp = 25;
      
      switch(counter1) {       // Change whichever setting is currently hilighted
        case 0:
          showLastCap = true;  // Set flag to display the last frame captured to SD
          exitFlag = true;     // and exit
          break;
        case 1:
          celsiusFlag = !celsiusFlag; // Toggle Celsius/Fahrenheit
          break;
        case 2:
          buttonRfunc = (buttonRfunc + 1) % 3; // Step through button functions
          break;
        case 3:
          loadPalette(paletteNum = (paletteNum + 1) % 5); // Step through various color palettes
          break;
        case 4:
          thermRange = (thermRange + 1) % 6; // Step through various temp range presets
          break;
        case 5:
          markersOn = !markersOn; // Toggle hot/cold marker visibility
          break;
        case 6:
          mirrorFlag = !mirrorFlag; // Toggle mirrored display
          break;
        case 7:
          switch(frameRate = (frameRate + 1) % 6) {              // 6 frame rates, 0.5 to 16 in powers of 2
            case 0: mlx.setRefreshRate(MLX90640_0_5_HZ); break;
            case 1: mlx.setRefreshRate(MLX90640_1_HZ); break;
            case 2: mlx.setRefreshRate(MLX90640_2_HZ); break;
            case 3: mlx.setRefreshRate(MLX90640_4_HZ); break;
            case 4: mlx.setRefreshRate(MLX90640_8_HZ); break;
            default: mlx.setRefreshRate(MLX90640_16_HZ); break;
          }
          break;
        case 8:
          emissivity = (emissivity + 90) % 100; // Step from 95% to 5% by -10%
          break;
        case 9:
          smoothing = !smoothing; // Toggle pixel smoothing
          break;
        case 10:
          arcada.setBacklight((screenDim = !screenDim) ? 64 : 255); // Change backlight LED
          break;
        case 11:
          tmp = sharpener;
          sharpener += 5;
          if (sharpener == 100) {
            sharpener = 99;
          } else if (sharpener > 100) {
            sharpener = 0;
          }
          setSharp = myImager.setSharpenerPercent(sharpener);
          if (!setSharp) {
            sharpener = tmp;
          }
          break;
        default:
          exitFlag = true;
          break;
      }
      if((counter1 > 0) && (counter1 < MENU_LEN - 1))    // Was any setting just changed?
        settingsChanged = true;

      arcada.display->fillRect(0, 12 * (counter1 - scrollPosition) + MENU_VPOS - 2, 160, 12, 0x0000); // Erase hilit menu line
      menuLines(counter1, scrollPosition);   // Retype hilit menu line
    }

    if(buttonActive && millis() > deBounce
       && (buttonBits & (ARCADA_BUTTONMASK_A | ARCADA_BUTTONMASK_B)) == 0)  // Has de-bounce wait expired & all buttons released?
      buttonActive = false;               // Clear flag to allow another button press
  }
  return(settingsChanged);
}

void menuLines(int lineNumber, int scrollPos) {  // Screen print a single line in the settings menu

  arcada.display->setTextColor(0xFFFF);               // White text
  arcada.display->setCursor(10, 12 * (lineNumber - scrollPos) + MENU_VPOS); // Menu lines 12 pixels apart

  if(lineNumber - scrollPos == 0 && scrollPos > 0) {  // Are any menu lines scrolled off screen top?
    arcada.display->print("           ^");            // Print a small up arrow indicator
  } else if(lineNumber - scrollPos == 8 && lineNumber + 1 < MENU_LEN) { // How about off the bottom?
    arcada.display->print("           v");            // Print a small down arrow indicator... yeah, it's a v
  } else {

    switch(lineNumber) {
      case 0:
        arcada.display->print("  Display last capture");
        break;
      case 1:
        arcada.display->print("     Scale - ");
        arcada.display->print(celsiusFlag ? "CELSIUS" : "FAHRENHEIT");
        break;
      case 2:
        arcada.display->print(" Rt button - ");
        switch(buttonRfunc) {
          case 1:
            arcada.display->print("CAPTURE"); break;
          case 2:
            arcada.display->print("RECORD"); break;
          default:
            arcada.display->print("FREEZE"); break;
        }
        break;
      case 3:
        arcada.display->print("   Palette - ");
        for(int xPos = 0; xPos < 72; ++xPos)   // Display the current heat spectrum colors
          arcada.display->drawFastVLine(xPos + 87, (lineNumber - scrollPos) * 12 + MENU_VPOS,
                                        8, colorPal[map(xPos, 0, 71, 0, 255)]);
        switch(paletteNum) {
          case 1:
            arcada.display->print("IRONBOW");
            break;
          case 2:
            arcada.display->print("FIREBOW");
            break;
          case 3:
            arcada.display->setTextColor(0x0000);    // Black text for reverse contrast
            arcada.display->print("ALARM");
            break;
          case 4:
            arcada.display->setTextColor(0x0000);    // Black text
            arcada.display->print("BLACK HOT");
            break;
          default:
            arcada.display->print("WHITE HOT");
            break;
        }
        break;
      case 4:
        arcada.display->print("Temp range - ");
        setColorRange(thermRange);
        switch(thermRange) {
          case 1:
            arcada.display->print("STANDARD"); break;
          case 2:
            arcada.display->print("COOL/WARM"); break;
          case 3:
            arcada.display->print("WARM/WARMER"); break;
          case 4:
            arcada.display->print("HOT SPOTS"); break;
          case 5:
            arcada.display->print("FIRE & ICE"); break;
          default:
            arcada.display->print("AUTO-RANGE"); break;
        }
        break;
      case 5:
        arcada.display->print("   Markers - ");
        arcada.display->print(markersOn ? "ON" : "OFF");
        break;
      case 6:
        arcada.display->print("     Image - ");
        arcada.display->print(mirrorFlag ? "MIRRORED" : "FORWARD");
        break;
      case 7:
        arcada.display->print("Frame rate - ");
        arcada.display->print((float)(1 << frameRate) * 0.5);
        arcada.display->print(" FPS");
        break;
      case 8:
        arcada.display->setTextColor(GRAY_33 << 1); // Grayed menu item
        arcada.display->print("Emissivity - ");
        arcada.display->print(emissivity);
        arcada.display->print("%");
        break;
      case 9:
        arcada.display->setTextColor(GRAY_33 << 1); // Grayed menu item
        arcada.display->print(" Smoothing - ");
        arcada.display->print(smoothing ? "ON" : "OFF");
        break;
      case 10:
        arcada.display->print(" Backlight - ");
        arcada.display->print(screenDim ? "DIM" : "FULL");
        break;
      case 11:
        arcada.display->print(" Sharpener - ");
        arcada.display->print(sharpener);
        arcada.display->print("%");
        break;
      case 12:
        arcada.display->print("       Exit menu");
    }
  }
}

// This is the function that substitutes for GPIO external interrupts
// It will check for A and B and Up button presses at 50Hz
void buttonCatcher(void) {
  buttonBits = arcada.readButtons();
  clickFlagMenu |= (buttonBits & ARCADA_BUTTONMASK_B) != 0;
  clickFlagSelect |= (buttonBits & ARCADA_BUTTONMASK_A) != 0;
  clickFlagUpsample |= (buttonBits & ARCADA_BUTTONMASK_UP) != 0;
  clickFlagSmoothstep |= (buttonBits & ARCADA_BUTTONMASK_RIGHT) != 0;
  clickFlagSwitchMode |= (buttonBits & ARCADA_BUTTONMASK_DOWN) != 0;
}

void drawtext(const char *text, uint16_t color) {
  arcada.display->setCursor(0, 0);
  arcada.display->setTextColor(color);
  arcada.display->setTextWrap(true);
  arcada.display->print(text);
}
