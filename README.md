# Non-blocking MPU9250 Arduino Due DMP IMU Driver
This project reduces the communication time of a publicly available MPU9250 MPU IMU library from 1ms to 23 us.  

## Background
The default Wire library in Arduino always reads and writes to the I2C bus in a blocking fashion. The behavior is adapted for almost all MPU IMU driver for Arduino I can find online.  However, the communication can take up to 1ms for a MPU9250 simple sensor read. This is a very long time for higher-performance chip such as ATSAM3X and is often critical for time-sensitive application such as robotic control. Further more, the Wire library on Arduino Due is somewhat unstable, and will sometimes stuck without returning if the I2C bus hangs at the last transmission bit.  
To resolve the issue, the ATMEL MCU architecture come to the rescue. The chip ATSAM3X used in the Arduino Due came with a hardware I2C controller (they call it the two wire interface, or TWI for legal reasons). This hardware controller can handle the I2C communication all by itself even the complicated "Repeated Start" process during slave read communication. This functionality of hardware controlled communication is often refered as DMA, or "Diret Memory Access". And the ATMEL names the hardware "Peripheral DMA Controller", or PDC on the two wire interface.  
For a 

## Performance
The default behavior for a MPU-9250 full IMU sensor read (accelerometer, gyroscope, magnetometer, and temperature), that requires 25 packet exchange between I2C master and slave device, requires 562.5 microseconds of I2C communication time. Actual measurement of the function often exceeds 1 ms due to overheads caused by the library.
With this library, the CPU time spent within the function reduced to below 23 microseconds. Thats 1/40 of the original time required.  
This also make multiple sensor across different interface (e.g, I2C, serial) with accurate timestamping possible.
Also, this library enables maximum read rate of MPU-9250 of 1KHz full-suite read rate, which default library often crashes du to super heavy CPU-TWI loading. You can check the [MPU9250_PDC_HiSpdInterrupt.ino](examples/MPU9250_PDC_HiSpdInterrupt/MPU9250_PDC_HiSpdInterrupt.ino) in the example directory. 

**Note:** Transmission qually matters a lot if I2C is communicating at maximum rate of 1kHz. Some of the cables I have (~25cm) will always crash the communication after few dozens of seconds. Try to use shorter ( < 5cm), higher quality wires with lower resistance and better contact.
  
## Installation
1. Arduino IDE: [Linux](https://www.arduino.cc/en/guide/linux), [Windows]()
2. Install the dependency: [Due_I2C_PDC](https://github.com/EwingKang/Due_I2C_PDC)
2. In IDE Tools->Board->Board manager, search "due" and install the Arduino SAM Boards (32-bit Cortex M3)
3. Change board to Arduino Due (programming port)
4. Clone this project **into your library** (recommanding name: due_pdc_imu)
    ```
    cd ~/Arduino/library/
    git clone https://github.com/EwingKang/Non-blocking-MPU9250-Arduino-Due-DMP-IMU-Driver.git due_pdc_imu
    ```
5. Got to Arduino IDE, select File -> examples -> Due PDC MPU-9250 DMP library -> MPU9250_PDC_HiSpdInterrupt
6. Build the example with the IDE

## Dev plan
Not in particular order
- [x] Remove static variable in class
- [x] 400Kbps support
- [ ] Remove all Serial.print within lib
- [x] Access guards for half-duplex dependent on status
- [x] Comm queue? buffers? (use DMA copy?)
- [x] High speed example for MPU 9250
- [ ] Simultaneous high speed communication with BMP280

## Changelog  
#### v1.1
  - Add interface to the non-blocking read function: updateAllUnblocked() now accepts external micros() clock to further speed the code up.
  - Add MPU9250_PDC_HiSpdInterrupt.ino testing file for 1kHz communication
  - Add testing screenshot
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
