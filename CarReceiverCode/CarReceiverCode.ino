#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(7, 8);

const uint64_t writing_pipe =  0xF0F0F0F0E1LL;

//DRV-8833 inputs
// Left motor
int in1 = 3;
int in2 = 5;
// Right motor
int in3 = 6;
int in4 = 9;

int speedLeft = 0;
int speedRight = 0;

void setup() {
  Serial.begin(9600);
  printf_begin();
  Serial.println("ROLE: receive\n\r");

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  radio.begin();
  //radio.setAutoAck(false);
  radio.setRetries(15, 15);
  radio.openReadingPipe(1, writing_pipe);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  radio.printDetails();
}

void loop() {
  if (radio.available())
  {
    float yprDegrees[3];
    bool done = false;
    while (!done)
    {
      done = radio.read(&yprDegrees, sizeof(yprDegrees));
      //Serial.println("Received data");

      float pitch = yprDegrees[1];
      float roll = yprDegrees[2];
      bool moving = false;
      bool forward = false;
      bool inplace = false;

//      Serial.print("ypr\t");
//      Serial.print(yprDegrees[0]);
//      Serial.print("\t");
//      Serial.print(yprDegrees[1]);
//      Serial.print("\t");
//      Serial.println(yprDegrees[2]);
      //uint16_t t1 = micros();

      if ( pitch < -10 ) // Forward
      {
        moving = true;
        forward = true;
        speedLeft = map(pitch, -10, -30, 0, 255);
        speedRight = map(pitch, -10, -30, 0, 255);

      }
      else if ( pitch > 10 )
      {
        moving = true;
        forward = false;
        speedLeft = map(pitch, 10, 30, 0, 255);
        speedRight = map(pitch, 10, 30, 0, 255);
      }
      else
      {
        moving = false; // Redundant but helps understand
        speedLeft = 0;
        speedRight = 0;
      }

      if ( roll > 20 ) // Right
      {
        moving = true;
        
        int turn = map(roll, 20, 50, 0, 255);
        speedLeft = speedLeft + turn;
        speedRight = speedRight - turn;

        if ( speedLeft > 255 ) speedLeft = 255;
        if ( speedRight < 0 ) speedRight = 0;

      }
      else if ( roll < -20 ) // Left
      {
        moving = true;
        
        int turn = map(roll, -20, -50, 0, 255);
        speedLeft = speedLeft - turn;
        speedRight = speedRight + turn;

        if ( speedLeft < 0 ) speedLeft = 0;
        if ( speedRight > 255 ) speedRight = 255;
      }

      if ( pitch > -10 && pitch < 10 ) inplace = true;
      if ( speedLeft < 80 && moving  ) speedLeft = 80;
      if ( speedRight < 80 && moving ) speedRight = 80;
      if ( speedLeft > 255 ) speedLeft = 255;
      if ( speedRight > 255) speedRight = 255;

      if ( inplace )
      {
        if ( roll > 0 )
        {
          digitalWrite(in1, LOW);
          analogWrite(in2, speedLeft);
          digitalWrite(in3, LOW);
          analogWrite(in4, speedRight);
        }
        else
        {
          analogWrite(in1, speedLeft);
          digitalWrite(in2, LOW);
          analogWrite(in3, speedRight);
          digitalWrite(in4, LOW);
        }
      }
      else
      {
        if ( forward )
        {
          digitalWrite(in1, LOW);
          analogWrite(in2, speedLeft);
          analogWrite(in3, speedRight);
          digitalWrite(in4, LOW);
        } 
        else
        {
          analogWrite(in1, speedLeft);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
          analogWrite(in4, speedRight);
        }
      }
      //uint16_t t2 = micros();
      //Serial.println(t2-t1);
      
      
      Serial.println(speedLeft);
      Serial.println(speedRight);
      //Serial.println(turn);

      //delay(50);
    }
    //delay(150);
  }
}

//if ((pitch < 10) && (pitch > -10))
//      {
//        if (roll > 20) {
//          forward = false; backwards = false, right_in_place = true, left_in_place = false;
//          turnLeft = map(roll, 20, 50, 0, 255);
//          turnRight = map(roll, 20, 50, 0, 255);; //map(roll, 20, 50, 0, -255);
//        }
//        else if (roll < -20) {
//          forward = false; backwards = false, right_in_place = false, left_in_place = true;
//          turnLeft = map(roll, -20, -50, 0, 255); //map(roll, 20, 50, 0, -255);
//          turnRight = map(roll, -20, -50, 0, 255);
//        }
//      }
//      else if (roll > 60 || roll < -60)
//      {
//        if(forward)
//        {
//          if (roll > 0) {
//            turnLeft = 255;
//            turnRight = 0; //map(roll, 20, 50, 0, -255);
//          }
//          else if (roll < 0) {
//            turnLeft = 0; //map(roll, 20, 50, 0, -255);
//            turnRight = 255;
//          }
//        }
//        else if (backwards)
//        {
//          if (roll > 0) {
//            turnLeft = 0; //map(roll, 20, 50, 0, -255);
//            turnRight = 255;
//          }
//          else if (roll < 0) {
//            turnLeft = 255;
//            turnRight = 0; //map(roll, 20, 50, 0, -255);
//          }
//        }
//      }
//      else if (roll > 20 && roll < 60) // Right
//      {
//        forward = true; // Overwritten if its not turning in place
//        turnLeft = map(roll, 20, 50, 0, 125);
//        turnRight = -turnLeft; //map(roll, 20, 50, 0, -255);
//      }
//      else if (roll > -60 && roll < -20) //Left
//      {
//        forward = true; // Overwritten if its not turning in place
//        turnRight = map(roll, -20, -50, 0, 125);
//        turnLeft = -turnRight; //map(roll, 20, 50, 0, -255);
//      }
//
//      if (pitch > 60 || pitch < -60)
//      {
//        if (pitch > 0) {forward = true; backwards = false; right_in_place = false, left_in_place = false;}
//        else if (pitch < 0) {forward = false; backwards = true; right_in_place = false, left_in_place = false;}
//        speed = 125;
//      }
//      else if (pitch > 10 && pitch < 60) // Forward
//      {
//        forward = true;
//        backwards = false;
//        speed = map(pitch, 10, 60, 0, 125);
//      }
//      else if (pitch < -10 && pitch > -60) // Backward
//      {
//        forward = false;
//        backwards = true;
//        speed = map(pitch, -10, -60, 0, 125);
//      }
//
//      if (forward)
//      {
//        analogWrite(in2, speed + turnRight); // Left motor
//        analogWrite(in3, speed + turnLeft); // Right motor
//      }
//      else if (backwards)
//      {
//        analogWrite(in1, speed + turnRight); // Left motor
//        analogWrite(in4, speed + turnLeft); // Right motor
//      }
//      else if (right_in_place)
//      {
//        analogWrite(in2, speed + turnRight); // Left motor forwards
//        analogWrite(in4, speed + turnLeft); // Right motor backwards
//      }
//      else if (left_in_place)
//      {
//        analogWrite(in1, speed + turnRight); // Left motor backwards
//        analogWrite(in3, speed + turnLeft); // Right motor forwards
//      }
