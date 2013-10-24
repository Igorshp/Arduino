

/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <XBee.h>

/*
This example is for Series 2 XBee
 Receives a ZB RX packet and sets a PWM value based on packet data.
 Error led is flashed if an unexpected packet is received
 */


XBee xbee = XBee();
uint8_t payload[] = { 0, 0 };
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
XBeeAddress64 addr64 = XBeeAddress64(0x0, 0x0);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

int statusLed = 13;
int errorLed = 13;
int dataLed = 13;
int data[1];
int xbee_data[1];
uint8_t relayPins[] = {7,8,9,10};
uint8_t switchPins[] = {2,3,4,5};
uint8_t switchStatus = 0;
uint8_t relayStatus = 0; // bitmask of the 4 relays
boolean success;
int dataLength;
uint8_t oldRelayStatus;

void flashLed(int pin, int times, int wait) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);
    if (i + 1 < times) {
      delay(wait);
    }
  }
}

void setup() {

  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(dataLed,  OUTPUT);

  // start serial
  Serial.begin(9600);
  xbee.begin(Serial);

  flashLed(statusLed, 3, 50);

  for(int i = 0; i < sizeof(switchPins); i++){
    pinMode(switchPins[i], INPUT);
    digitalWrite(switchPins[i], LOW);
  }
  for(int i = 0; i < sizeof(relayPins); i++){
    pinMode(relayPins[i], OUTPUT);
  }
}
// continuously reads packets, looking for ZB Receive or Modem Status

uint8_t* xbee_get_data(int &dataLength, boolean &success){
  xbee.readPacket();
  success = false;
  if (xbee.getResponse().isAvailable()) {
    // got something
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      // got a zb rx packet
      // now fill our zb rx class
      xbee.getResponse().getZBRxResponse(rx);
      if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
        // the sender got an ACK
        flashLed(statusLed, 5, 10);
      } 
      else {
        // we got it (obviously) but sender didn't get an ACK
        flashLed(errorLed, 2, 200);
      }
      //use data
      success = true;
      //uint8_t* data = rx.getData();
      uint8_t data[rx.getDataLength()];       
      for(int i = 0; i < rx.getDataLength(); i ++){
        data[i] = rx.getData(i);
      }
      dataLength = rx.getDataLength();
      return data;
      // end of using data
    } 
    else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      xbee.getResponse().getModemStatusResponse(msr);
      // the local XBee sends this response on certain events, like association/dissociation

      if (msr.getStatus() == ASSOCIATED) {
        // yay this is great.  flash led
        flashLed(statusLed, 10, 10);
      } 
      else if (msr.getStatus() == DISASSOCIATED) {
        // this is awful.. flash led to show our discontent
        flashLed(errorLed, 10, 10);
      } 
      else {
        // another status
        flashLed(statusLed, 5, 10);
      }
    } 
    else {
      // not something we were expecting
      flashLed(errorLed, 1, 25);    
    }
  } 
  else if (xbee.getResponse().isError()) {
    //nss.print("Error reading packet.  Error code: ");  
    //nss.println(xbee.getResponse().getErrorCode());
  }
} // end of xbee_get_data()



void loop() {

  oldRelayStatus = relayStatus;
  for(int i = 0; i < sizeof(switchPins); i++){
    int value = digitalRead(switchPins[i]);
    int oldValue = (switchStatus >> i) & 1;  // returns 1,2,4,8 depending on pin hmmm

    if(oldValue == value){  // needs fixing
      switchStatus ^= 1 << i;
      relayStatus ^= 1 << i;
  
    }
  }

  uint8_t* xbee_data = xbee_get_data(dataLength,success);
  if(success == true){
    
    if (xbee_data[0] == 'r'){
           payload[0] = switchStatus;
      payload[1] = relayStatus;
      xbee.send(zbTx);
    }else if(dataLength > 1){
      relayStatus = (xbee_data[1]+(xbee_data[0]*10)) & 0x0F;
    }else{
      relayStatus = xbee_data[0] & 0x0F;
    }
  }

  if(oldRelayStatus ^ relayStatus){
       payload[0] = switchStatus;
      payload[1] = relayStatus;
      xbee.send(zbTx);
  }

  //send data


  //end of send data



  //toggle the relays
  for(int i = 0; i < sizeof(relayPins); i++){
    if(relayStatus & (1 << i)){
      digitalWrite(relayPins[i], LOW); 
    }
    else{
      digitalWrite(relayPins[i], HIGH); 
    }
  }
}


