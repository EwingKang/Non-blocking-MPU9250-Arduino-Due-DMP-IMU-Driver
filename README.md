# pdc_lib_v01
This intent to ba a library for non-blocking I2C communication on Arduino Due (ATSAM3X).
The function uses Peripheral DMA Controller, or PDC on the Two Wire Interface (TWI, can be seen as a alias for I2C). Basically, you only have to assign the data and volume to the PDC, and the hardware controller will transfer (transmit or receive) the data in the background. 

## Dev plan
Not in particular order
- Remove static variable in class
- Access guards for half-duplex dependent on status
- Comm queue? buffers? (use DMA copy?)

## Changelog
##### v0.2 
  - Add reset and manual SCK hard reset
  - Simplify comm state management
  - Clean up library comments
##### v0.1 First working version
