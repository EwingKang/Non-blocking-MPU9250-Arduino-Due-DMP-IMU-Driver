# arduino_pdc_i2c
This intent to ba a library for non-blocking I2C communication on Arduino Due (ATSAM3X).  
The function uses Peripheral DMA Controller, or PDC on the Two Wire Interface (TWI, can be seen as a alias for I2C). Basically, you only have to assign the data and volume to the PDC, and the hardware controller will transfer (transmit or receive) the data in the background. 

## Installation
1. Arduino IDE: [Linux](https://www.arduino.cc/en/guide/linux), [Windows]()
2. In IDE Tools->Board->Board manager, search "due" and install the Arduino SAM Boards (32-bit Cortex M3)
3. Change board to Arduinoi Due (programming port)
4. Clone this project 
    ```
    cd ~/Arduino/
    git clone https://github.com/EwingKang/arduino_pdc_i2c.git
    ```
5. Build the project with IDE

## Dev plan
Not in particular order
- [x] Remove static variable in class
- [x] 400Kbps support
- [ ] Remove all Serial.print within lib
- [x] Access guards for half-duplex dependent on status
- [ ] Comm queue? buffers? (use DMA copy?)

## Changelog
##### v0.4
  - Add I2C bus/deivce library
  - New testing .ino source
  - Tested SetGeg and GetGeg functions
  - Blocking version of the function call
  - Move old test file into sub directory
##### v0.3
  - Cleanup comm status handling
  - Fix single bye RX to copy to pointers
  - Test multi-read with 2 bytes
  - Disable all debug info during nominal operation
  - Test 400K communication (success!)
##### v0.2 
  - Add reset and manual SCK hard reset
  - Simplify comm state management
  - Clean up library comments
  - Create README
##### v0.1 First working version

## Reference
- Arduino Forum [I2C DMA](https://forum.arduino.cc/index.php?topic=605127.0)
- Arduino Forum [I2C + DMA](https://forum.arduino.cc/index.php?topic=152643.0)
- Arduino Forum [Interrupt misconception on Due (ATSAM3X)](https://forum.arduino.cc/index.php?topic=621506.0)
- Arduino Forum [Hanging I2C on DUE, SDA Low SCL High permanent](https://forum.arduino.cc/index.php?topic=288573.0)

## TODO
- Use IADAR method for internal address access, its much simpler and requires much less interrupts
    * https://github.com/arduino/ArduinoCore-sam/blob/9944119937a4a3341d2023f0a46c422c0da46298/libraries/Wire/src/Wire.cpp
    * https://github.com/arduino/ArduinoCore-sam/blob/9944119937a4a3341d2023f0a46c422c0da46298/libraries/Wire/src/Wire.cpp
    * https://github.com/brandonbraun653/SAM3X8E-Libraries/search?q=twi_master_read
