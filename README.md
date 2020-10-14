# Non-blocking MPU9250 Arduino Due DMP IMU Driver
This project reduces the communication time of a publicly available MPU9250 MPU IMU library from 1ms to 23 us.  

## Background
The default Wire library in Arduino always reads and writes to the I2C bus in a blocking fashion. The behavior is adapted for almost all MPU IMU driver for Arduino I can find online.  However, the communication can take up to 1ms for a MPU9250 simple sensor read. This is a very long time for higher-performance chip such as ATSAM3X and is often critical for time-sensitive application such as robotic control. Further more, the Wire library on Arduino Due is somewhat unstable, and will sometimes stuck without returning if the I2C bus hangs at the last transmission bit.  
To resolve the issue, the ATMEL MCU architecture come to the rescue. The chip ATSAM3X used in the Arduino Due came with a hardware I2C controller (they call it the two wire interface, or TWI for legal reasons). This hardware controller can handle the I2C communication all by itself even the complicated "Repeated Start" process during slave read communication. This functionality of hardware controlled communication is often refered as DMA, or "Diret Memory Access". And the ATMEL names the hardware "Peripheral DMA Controller", or PDC on the two wire interface.  
  
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
- [x] Comm queue? buffers? (use DMA copy?)

## Changelog
#### v1.0
  Complete the non-blocking communication between Arduino Due and MPU9250 IMU.
  - Main .ino code
      * Provide 2 version (blocking, non-blocking) of code, switching by macro
      * Non blocking code use timer controlled realtime style loop that checks the function regularly.
      * Benchmark: blocking read: 0.6 ms; non-blocking: 42us (minimal possible 23 us)
  - inv_mpu library
      * Provide two functions: mpu_hear_all_sensors() and mpu_parse_all_sensors() as primary interface for sending request for reading IMU data, and parsing data once request is done
      * Use arduino_pdci2c_ask() and arduino_pdci2c_hear() that interfaces with i2c_bus library.
      * The spining update is done by upper mpu9250 library
  - pdc_mpu9250_dmp library
      * Add additional state of INV_PENDING
      * Use updateAll() as blocking, chunck read of all IMU sensors
      * Use updateAllUnblocked() as unblocked version. This function replys INV_PENDING if any request has been send and is waiting for i2c_bus response.
      * updateAllUnblocked() should be called regularly to update I2C bus internal status.
#### v0.4
  - Add I2C bus/deivce library
  - New testing .ino source
  - Tested SetGeg and GetGeg functions
  - Blocking version of the function call
  - Move old test file into sub directory
#### v0.3
  - Cleanup comm status handling
  - Fix single bye RX to copy to pointers
  - Test multi-read with 2 bytes
  - Disable all debug info during nominal operation
  - Test 400K communication (success!)
#### v0.2 
  - Add reset and manual SCK hard reset
  - Simplify comm state management
  - Clean up library comments
  - Create README
#### v0.1 First working version

## Reference
- Arduino Forum [I2C DMA](https://forum.arduino.cc/index.php?topic=605127.0)
- Arduino Forum [I2C + DMA](https://forum.arduino.cc/index.php?topic=152643.0)
- Arduino Forum [Interrupt misconception on Due (ATSAM3X)](https://forum.arduino.cc/index.php?topic=621506.0)
- Arduino Forum [Hanging I2C on DUE, SDA Low SCL High permanent](https://forum.arduino.cc/index.php?topic=288573.0)
- Arduino github [Arduino Due Wire library](https://github.com/arduino/ArduinoCore-sam/blob/9944119937a4a3341d2023f0a46c422c0da46298/libraries/Wire/src/Wire.cpp)
- Github [Example code for WIT communication on SAM3X8E](https://github.com/brandonbraun653/SAM3X8E-Libraries/search?q=twi_master_read)
