# AGON 
 Arduino software for UConn's AIAA Propulsive Landing project.
 
 # dev environment setup
 1. Download Arduino IDE: https://www.arduino.cc/en/software
 2. Download Teensy Loader: https://www.pjrc.com/teensy/loader.html
 3. Clone this repo
 4. Navigate into the 'arduino_libraries' directory. 
 5. Copy 'BNO055-master' and 'SPIMemory-2.6.0'
 6. Navigate to your 'Documents/Arduino/libraries'
 7. Paste 'BNO055-master' and 'SPIMemory-2.6.0' in libraries
 8. Navigate back to 'AGON/main' and open 'main.ino'
 9. Connect computer to Teensy via a micro-USB cable
 10. Press the white pushbutton on the Teensy
 11. Navigate to the menu 'Tools->Boards->Teensyduino'. Select Teensy 3.2 / 3.1
 12. Verify (check icon) and upload (arrow icon) the sketch
 13. If working with the Spring version of the program, LEDs should soon alight

# TODO: add platformio instructions
