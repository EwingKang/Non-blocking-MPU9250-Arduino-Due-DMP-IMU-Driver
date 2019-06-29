# pdc_lib_v01
This intent to ba a library for non-blocking I2C communication on Arduino Due (ATSAM3X).  
The function uses Peripheral DMA Controller, or PDC on the Two Wire Interface (TWI, can be seen as a alias for I2C). Basically, you only have to assign the data and volume to the PDC, and the hardware controller will transfer (transmit or receive) the data in the background. 

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
