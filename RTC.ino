//Edited by Nicole Siron
//source: looking for original source 
//Note: Have to edit the time every time the code runs

#include "Wire.h"
#define DS1307_ADDRESS 0x68
byte zero = 0x00; 

void setup(){
  Wire.begin();
  Serial.begin(9600);
  setDateTime(); 
}

void loop(){
  printDate();
  delay(1000);
}

void setDateTime(){

  byte second =      0; //0-59
  byte minute =      11; //0-59
  byte hour =        14; //0-23
  byte weekDay =     7; //1-7
  byte monthDay =    30; //1-31
  byte month =       4; //1-12
  byte year  =       21; //0-99

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero); //stop Oscillator

  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));

  Wire.write(zero); //start 

  Wire.endTransmission();

}

byte decToBcd(byte val){
// Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)  {
// Convert binary coded decimal to normal decimal numbers
  return ( (val/16*10) + (val%16) );
}

void printDate(){

  // Reset the register pointer
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 7);

  int second = bcdToDec(Wire.read());
  int minute = bcdToDec(Wire.read());
  int hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
  int weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  int monthDay = bcdToDec(Wire.read());
  int month = bcdToDec(Wire.read());
  int year = bcdToDec(Wire.read());

  
  Serial.print(month);
  Serial.print("/");
  Serial.print(monthDay);
  Serial.print("/");
  Serial.print(year);
  Serial.print(" ");
  Serial.print(hour);
  Serial.print(":");
  Serial.print(minute);
  Serial.print(":");
  Serial.println(second);

}
