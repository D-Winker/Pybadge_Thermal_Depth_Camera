This project builds on Adafruit's "MLX90640 Thermal Camera." See this https://learn.adafruit.com/mlx90640-thermal-image-recording/ for their guide.  
  
This project adds a VL53L5CX 8x8 depth sensor to the original PyBadge + MLX90640 32x24 thermal camera. It also adds various features  
- 2x and 4x Bilinear interpolation
- Normal and smoothstep options for the bilinear interpolation
- View selection of thermal data, depth data, or a two-channel thermal + depth image
- Captured thermal and depth data are now stored as CSV files
- An exponentially weighted moving average, to smooth viewed data over time
- A frame rate readout
- Two options for displaying depth data. The VL53L5CX returns up to 4 range measurements per zone (of the 8x8 zones)
     - One mode shows the nearest return, the other attemps to 'intelligently' combine all of the information for a 3x upscaled image
     - In either mode, all available measurements are saved to CSV when an image is captured
     - I have only been able to get, at most, three returns per zone. Getting more than 1 requires modifying platform.h in the Sparkfun VL53LCX library files
  
Unfortunately, my changes have slowed things down considerably, it now maxes out around 3 frames per second.  
  
A note about connecting the VL53LCX: I initially had issues with the VL53LCX because the PyBadge puts VBat on the Qwiic connector instead of 3.3V. The Pybadge does have 3.3V available on a header; this requires some modification of the Qwiic connector, or not using the connector.  
