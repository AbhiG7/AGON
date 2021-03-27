#include "Wire.h"
#include <SPI.h>
#include <SPIFlash.h>
#include <Servo.h>

SPIFlash flash(1);

struct data {
    float _float;
    String _str;
    int _int;
};

data dataOut1 = {84, "hello max"};
data dataOut2 = {231.123, "test 123 a;lsdka;sldka;ewfi halfjnslefa;s;soaldkfa;ldksea;kasldfjas;eija;fjas;dkj;af;l;a;efa;sl;sefjsdkfjdfheurhoiqwurhoqwerqw21456w49876543er4q6we87r6asdfas3v41we68f4a6ef4as3f1", 15};

data dataIn;

uint32_t _address1;
uint32_t _address2;

void setup() {
    flash.begin(9600);
    flash.eraseChip();
    _address1 = flash.getAddress(sizeof(dataOut1));
    _address2 = flash.getAddress(sizeof(dataOut2));
    Serial.print(F("Address 1 = "));
    Serial.println(_address1);
    Serial.print(F("Address 2 = "));
    Serial.println(_address2);
    
    Serial.print(F("writeAnything()"));
    int t = micros();
    if (!flash.writeAnything(_address1, dataOut1)) { // Function is used to write the
        // address '_address'
        Serial.println(F("Failed"));
    }
    else {
        Serial.print("time elaspsed: ");
        Serial.println(micros()-t);
        Serial.println(F("Passed"));
        Serial.print("Address: ");
        Serial.println(_address1);
    }

    if (!flash.writeAnything(_address2, dataOut2)) { // Function is used to write the
        // address '_address'
        Serial.println(F("Failed"));
    }
    else {
        Serial.print("time elaspsed: ");
        Serial.println(micros()-t);
        Serial.println(F("Passed"));
        Serial.print("Address: ");
        Serial.println(_address2);
    }

    flash.readAnything(_address1, dataIn);
    Serial.println(dataIn._int);

    flash.readAnything(_address2, dataIn);
    Serial.println(dataIn._int);
}

void loop() {

}
