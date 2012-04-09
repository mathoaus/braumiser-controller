/*
brauduino semi automated single vessel RIMS
 created by s.mathison
 Copyright (C) 2012  Stephen Mathison
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.*/



//libraries
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <PID_v1.h>
OneWire ds(11);
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);


// push buttons
const int Button_up = A3;
const int Button_dn = A2;
const int Button_prev = A1;
const int Button_nxt = A0;


// outputs
const int Heat = 9;
const int Pump = 8;
const int Buzz = 10;


// global variables
unsigned int WindowSize;

unsigned long windowStartTime;
unsigned long start;

double Setpoint, Input, Output,eepromKp, eepromKi, eepromKd;
boolean autoLoop = false; 
boolean manuLoop = false;
boolean waterAdd = false;
boolean Conv_start = false;
boolean mpump = false;
boolean mheat = false;
boolean wtBtn = false;
boolean autoEnter = false;
boolean tempReached = false;
boolean pumpRest = false;
boolean boilLoop = false;
boolean resume = false;
float mset_temp = 35;
float Temp_c, stageTemp,pumptempError;

int temp;
int x;
int  stageTime,hopTime;

byte mainMenu = 0;
byte pumpTime;
byte data[2];
byte second;
byte minute;
byte i;
byte Busy = 0;
byte nmbrStgs;
byte nmbrHops;
byte tempHAddr;
byte tempLAddr;
byte timeAddr;
byte blhpAddr;
byte hopAdd;

char* stgName[] ={
  "MashIn","Stage1","Stage2","Stage3","Stage4","Stage5","Stage6","Stage7","Stage8"};


// degree c sybmol 
byte degc[8] =
{
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00100,
  B00011,
  B00000,
};


//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,100,20,5, DIRECT);


void setup()
{
  // Start up the library
  lcd.begin(16,2);
  pinMode (Button_up,INPUT);
  pinMode (Button_dn,INPUT);
  pinMode (Button_prev,INPUT);
  pinMode (Button_nxt,INPUT);
  pinMode (Heat,OUTPUT);
  pinMode (Pump,OUTPUT);
  pinMode (Buzz,OUTPUT);
  windowStartTime = millis();

  //tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);

  // write custom symbol to LCD
  lcd.createChar(0,degc);
}

void loop()
{
  switch (mainMenu){
    case (0):
    digitalWrite(Heat,LOW);
    digitalWrite(Pump,LOW);  
    Temperature();
    lcd.setCursor(0,0);
    lcd.print(" The Brauduino  ");
    lcd.setCursor(0,1);
    lcd.print("  Temp=");
    lcd.print(Temp_c);
    lcd.write(0);
    //read the buttons for menu selection

    if (digitalRead(Button_dn) == 0){
      delay(1000);
      if (digitalRead(Button_dn) == 0){
        lcd.setCursor(0,0);
        lcd.print("    Manual      ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        mainMenu = 1;
        while(digitalRead(Button_dn) == 0){
        } 
      }
    }  
    if (digitalRead(Button_prev) == 0){
      delay(1000);
      if (digitalRead(Button_prev) == 0){
        lcd.setCursor(0,0);
        lcd.print("     Auto       ");
        lcd.setCursor(0,1);
        lcd.print("                "); 
        mainMenu = 2;
        while(digitalRead(Button_prev) == 0){
        } 
      }
    }
    if (digitalRead(Button_nxt) == 0){
      delay(1000);
      if (digitalRead(Button_nxt) == 0){         
        lcd.setCursor(0,0);
        lcd.print("     Setup      ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        delay(500);        
        mainMenu = 3;
        while(digitalRead(Button_nxt)==0){
        } 
      }
    }
    break;

    case(1): //manual mode
    delay(500);
    eepromKp = word(EEPROM.read(0),EEPROM.read(1));// read the PID settings from the EEPROM
    eepromKi = word(EEPROM.read(2),EEPROM.read(3));
    eepromKd = word(EEPROM.read(4),EEPROM.read(5));

    myPID.SetTunings(eepromKp,eepromKi,eepromKd); // send the PID settings to the PID
    WindowSize = word(EEPROM.read(33),EEPROM.read(34));
    myPID.SetOutputLimits(0, WindowSize);

    lcd.setCursor(0,0);           // prompt to add water
    lcd.print("  Water added?  ");
    Buzzer2();
    lcd.setCursor(0,1);
    lcd.print("       Yes  Quit");
    wtBtn = true;

    while (wtBtn){               // wait for comfirmation 
      if (digitalRead(Button_prev)==0){
        manuLoop = true;
        wtBtn = false;
        lcd.clear();
        while(digitalRead(Button_prev)==0){
        }
      }
      if (digitalRead(Button_nxt)==0){
        mainMenu = 0;
        wtBtn = false;
        lcd.clear();
        while(digitalRead(Button_nxt)==0){
        }
      }
    }

    while (manuLoop){            // manual loop
      Temperature();
      Setpoint = mset_temp;
      Input = Temp_c;

      lcd.setCursor(0,0);
      lcd.print("  Manual Mode   ");
      lcd.setCursor(0,1);
      lcd.print("S/A=");
      lcd.print(mset_temp);
      lcd.print("/");
      lcd.print(Temp_c);
      lcd.write(0);

      // Increase set temp or quit
      if (digitalRead(Button_up)==0){
        delay(200);
        if (digitalRead(Button_dn)==0){
          digitalWrite(Heat,LOW);
          digitalWrite(Pump,LOW);
          mainMenu=0;
          manuLoop = false;
          lcd.clear();
        }
        else{
          mset_temp= mset_temp+0.25;
        }
        while (digitalRead(Button_up)==0){
        }
      }

      // decrease temp or quit
      if (digitalRead(Button_dn)==0) 
      {
        delay(200);
        if (digitalRead(Button_up)==0){
          digitalWrite(Heat,LOW);
          digitalWrite(Pump,LOW);
          mainMenu=0;
          manuLoop = false;
          lcd.clear();
        }
        else{
          mset_temp= mset_temp-0.25;
        }
        while (digitalRead(Button_dn)==0){
        }
      }

      //turns heat on or off      
      if (digitalRead(Button_prev)==0){
        while(digitalRead(Button_prev)==0){
        }
        if (mheat==false){
          mheat = true;
          windowStartTime = millis();
        }
        else{
          mheat = false;
          digitalWrite(Heat,LOW);
        }
      }

      //turns the pump on or off
      if (digitalRead(Button_nxt)==0){
        if (mpump == false){
          mpump = true;
          digitalWrite(Pump,HIGH);
        }
        else{
          mpump = false;
          digitalWrite(Pump,LOW);
        }
        while(digitalRead(Button_nxt)==0){
        }
      }
      if (mheat){
        PID_HEAT(); 
      }        
    }
    break;

    /* This is the Auto process, it reads the number of stages and it the starts a loop for the number of stages and executes the stages*
     * at the start of each stage it reads the time and temp of that stage and once the temp is reached it starts counting down         */


    case(2): 
    delay(500);
    tempHAddr = 6; // setup intitial stage addresses 
    tempLAddr = 7;
    timeAddr = 8;
    nmbrStgs = EEPROM.read(38);// read the number of steps
    nmbrHops = EEPROM.read(39);//read the number of hop additions
    eepromKp = word(EEPROM.read(0),EEPROM.read(1));// get the PID settings
    eepromKi = word(EEPROM.read(2),EEPROM.read(3));
    eepromKd = word(EEPROM.read(4),EEPROM.read(5));
    myPID.SetTunings(eepromKp,eepromKi,eepromKd);// set the PID
    WindowSize =word(EEPROM.read(33),EEPROM.read(34));
    myPID.SetOutputLimits(0, WindowSize);


    if(EEPROM.read(35)){ // read the auto started byte to see if it has been set and if so ask to resume
      lcd.clear();
      lcd.print("Resume Process?");
      lcd.setCursor(0,1);
      lcd.print("        Yes   No");
      wtBtn = true;

      while (wtBtn){  // wait for comfirmation
        if (digitalRead(Button_prev)==0){
          resume = true;
          tempHAddr = (EEPROM.read(36)*3)+6;
          tempLAddr = tempHAddr+1;
          timeAddr = tempHAddr+2;
          wtBtn = false;
          autoEnter = true;
          lcd.clear();
        }
        if (digitalRead(Button_nxt)==0){
          wtBtn = false;
          resume = false;

        }
      }
    }

    if(!(resume)){  // if starting a new process prompt for water
      lcd.clear();
      lcd.print("  Water added?  ");
      Buzzer2();
      lcd.setCursor(0,1);
      lcd.print("       Yes  Quit");
      wtBtn = true;

      while (wtBtn){
        if (digitalRead(Button_prev)==0){
          autoEnter = true;
          wtBtn = false;
          EEPROM.write(35,1);
          lcd.clear();
          lcd.print("  Pump  Prime  ");  // priming the pump
          digitalWrite(Pump,HIGH);
          delay (1000);
          digitalWrite(Pump,LOW);
          delay(200);
          digitalWrite(Pump,HIGH);
          delay (1000);
          digitalWrite(Pump,LOW);
          delay(200);
          digitalWrite(Pump,HIGH);
          delay (1000);
          digitalWrite(Pump,LOW);  
          lcd.clear();      
          while(digitalRead(Button_prev)==0){
          }
        }
        if (digitalRead(Button_nxt)==0){
          mainMenu = 0;
          wtBtn = false;
          autoEnter = false;
          lcd.clear();
          while(digitalRead(Button_nxt)==0){
          }
        }
      }
    }
    if (autoEnter){     // mash steps
      if(resume){
        x = EEPROM.read(36); // reads the last step of the interrupted process
      }
      else{
        x = 0;
      }

      for (int i = x;i < nmbrStgs;i++){
        EEPROM.write(36,lowByte(x)); // stores the stage number for the resume
        x++;                         // used to count the stages for the resume 
        tempReached = false;
        stageTemp = word(EEPROM.read(tempHAddr),EEPROM.read(tempLAddr));
        stageTemp = stageTemp/16.0;
        if (resume){                 // on the start of resume gets saved time
          stageTime=EEPROM.read(37);
          resume = false;            // clears resume for next loop
        }
        else{
          stageTime = EEPROM.read(timeAddr); // gets stage time
          EEPROM.write(37,lowByte(stageTime));// saves the intial stage time
        }
        start = millis();
        // windowStartTime = millis();
        second = 0;
        minute = 0;
        delay(50);

        while ((stageTime>0)&&(autoEnter)){
          if ((millis()-start)>1000){  // timing routine
            start = millis();
            second++;
            if(!(tempReached))second=0;// starts counting down when temp reached
            if (second>59){
              lcd.setCursor(10,0);
              lcd.print("      ");
              second = 0;
              pumpTime++;
              stageTime--;
              EEPROM.write(37,lowByte(stageTime));// saves stage time incase of interuption
            }
          }
          Temperature();// get temp
          Setpoint = stageTemp;//
          Input = Temp_c;
          if (pumpRest){
            lcd.setCursor(0,0);
            lcd.print("   Pump  Rest   ");  // display Pump rest during rest period
            lcd.setCursor(0,1);
            lcd.print("                ");
          }
          else{
            lcd.setCursor(0,0);       // display stage name, time left , set temp and actual temp
            lcd.print(stgName[i]);
            lcd.print("    ");
            lcd.setCursor(10,0);
            lcd.print("T=");
            lcd.print(stageTime);
            lcd.setCursor(0,1);
            lcd.print("S/A=");
            lcd.print(stageTemp);
            lcd.print("/");
            lcd.print(Temp_c);
            lcd.write(0);
          }
          if(i == 0)pumpTime = 0;
          pumptempError = stageTemp-Temp_c;
          if (pumptempError <= 0)tempReached = true;
          if ((pumpTime < 10) || (pumptempError > 1.0)){ // starts pumps and heat
            digitalWrite(Pump,HIGH);
            PID_HEAT();
            pumpRest =false;
          }
          if ((pumpTime >= 10) && (pumptempError < 1.0)){ // pump rest
            digitalWrite(Pump,LOW);
            digitalWrite(Heat,LOW);
            pumpRest = true;
            if(pumpTime>=12)pumpTime = 0;
          } 
          if (digitalRead(Button_dn) == 0) // quit pressing up and down button
          {
            delay(200);
            if (digitalRead(Button_up) == 0){
              digitalWrite(Heat,LOW);
              digitalWrite(Pump,LOW);
              lcd.clear();
              delay(50);
              mainMenu=0;
              autoEnter = false;
              i = 10;
            }
          }
        }

        if( i==0 ){    // at the end of the mashIn step pauses to add grain
          lcd.clear();
          digitalWrite(Pump,LOW);
          digitalWrite(Heat,LOW);
          lcd.print("    Add Malt   ");
          Buzzer2();
          lcd.setCursor(0,1);
          lcd.print("         Confirm");
          wtBtn = true;
          while (wtBtn){
            if (digitalRead(Button_nxt)==0){
              wtBtn = false;
              lcd.clear();
              while(digitalRead(Button_nxt)==0){
              }
            }
            if (digitalRead(Button_dn)==0) 
            {
              delay(200);
              if (digitalRead(Button_up)==0){
                digitalWrite(Heat,LOW);
                digitalWrite(Pump,LOW);
                lcd.clear();
                delay(50);
                mainMenu=0;
                autoEnter = false;
                i = 10;
              }
            }
          }
        }
        if(i==(nmbrStgs-1)){   // at the end of the last step pauses to remove the malt pipe before the boil
          lcd.clear();
          x = nmbrStgs;               // used add to stage count on the final stage for the resume 
          EEPROM.write(36,lowByte(x)); // stores the stage number for the resume
          digitalWrite(Pump,LOW);
          digitalWrite(Heat,LOW);
          lcd.print("  Remove  Malt  ");
          lcd.setCursor(0,1);
          Buzzer2();
          lcd.print("         Confirm");
          wtBtn = true;
          while (wtBtn){
            if (digitalRead(Button_nxt) == 0){
              wtBtn = false;
              lcd.clear();
              while(digitalRead(Button_nxt)==0){
              }
            }
            if (digitalRead(Button_dn) == 0) 
            {
              delay(200);
              if (digitalRead(Button_up) == 0){
                digitalWrite(Heat,LOW);
                digitalWrite(Pump,LOW);
                lcd.clear();
                delay(50);
                mainMenu=0;
                autoEnter = false;
                i=10;
              }
            }
          }

        }          

        Buzzer1(); 
        tempHAddr +=3; // increase stage addresses 
        tempLAddr +=3;
        timeAddr +=3;
        lcd.clear(); 
      }
    }
    // start of the boil
    if(autoEnter){
      start = millis();
      stageTemp= 98.0; // set the intital boil temp to 98 deg c
      boilLoop = true;
      tempReached = false;  
      nmbrHops = EEPROM.read(39);
      if (resume){
        stageTime= EEPROM.read(37);
      }
      else{
        stageTime = EEPROM.read(40);
        EEPROM.write(37,lowByte(stageTime));
      }
      hopAdd = EEPROM.read(49);
      blhpAddr = hopAdd+41;
      lcd.clear();
      hopTime = EEPROM.read(blhpAddr);


      while(boilLoop){
        if(hopAdd <= nmbrHops){
          if (stageTime == hopTime){
            Buzzer2();
            lcd.clear();
            lcd.print("    Add  Hops");
            delay(2000);
            Buzzer2();
            hopAdd++;
            EEPROM.write(49,hopAdd);
            blhpAddr++;
            hopTime = EEPROM.read(blhpAddr);
            lcd.clear();
          }
        }


        if(stageTime == 0 )boilLoop = false;  
        Temperature();
        Setpoint = stageTemp;
        Input = Temp_c;
        PID_HEAT();
        if(Temp_c >= stageTemp)tempReached =true;

        lcd.setCursor(0,0);
        lcd.print("Boil");
        lcd.setCursor(10,0);
        lcd.print("T=");
        lcd.print(stageTime);        
        lcd.setCursor(0,1);
        lcd.print("S/A=");
        lcd.print(int(stageTemp));
        lcd.print("/");
        lcd.print(Temp_c);
        lcd.write(0);


        if (digitalRead(Button_up)==0){ // quit or increase temp
          delay(200);
          if (digitalRead(Button_dn)==0){
            digitalWrite(Heat,LOW);
            digitalWrite(Pump,LOW);
            mainMenu=0;
            autoEnter = false;
            boilLoop = false;
            lcd.clear();
          }
          else{
            stageTemp++;
            if (stageTemp>120)stageTemp=120;
          }
          while (digitalRead(Button_up)==0){
          }
        }


        if (digitalRead(Button_dn)==0) // quit or decrease temp
        {
          delay(200);
          if (digitalRead(Button_up)==0){
            digitalWrite(Heat,LOW);
            digitalWrite(Pump,LOW);
            mainMenu=0;
            autoEnter = false;
            boilLoop = false;
            lcd.clear();
          }
          else{
            stageTemp--;
            if (stageTemp < 95.0)stageTemp = 95.0;
          }
          while (digitalRead(Button_dn) == 0){
          }
        }
        if ((millis() - start) > 1000){  // timing routine
          start = millis();
          second++;
          if(!(tempReached)) second = 0;// starts counting down when temp reached
          if (second > 59){
            lcd.setCursor(10,0);
            lcd.print("      ");
            second = 0;
            stageTime--;
            EEPROM.write(37,lowByte(stageTime));
          }
        }

      }
    }
    if(autoEnter){    // finishes the brewing process
      lcd.clear();
      lcd.print("    Brewing     ");
      lcd.setCursor(0,1);
      lcd.print("   Finished     ");
      Buzzer2();
      delay(2000);
      EEPROM.write(35,0); // sets auto start byte to 0 for resume
      EEPROM.write(49,0); // sets hop count to 0
      mainMenu=0;
      autoEnter =false;
      resume =false;
    }


    break;
    case(3):// Setup 
    SetupFun();
    lcd.clear();
    mainMenu=0;
    break; 

  }
}



// reads the DS18B20 temerature probe 

void Temperature(void){
  ds.reset();
  ds.skip();
  // start conversion and return
  if (!(Conv_start)){
    ds.write(0x44,0);
    Conv_start = true;
    return;
  }
  // check for conversion if it isn't complete return if it is then convert to decimal
  if (Conv_start){
    Busy = ds.read_bit();
    if (Busy == 0){
      return;
    }
    ds.reset();
    ds.skip();
    ds.write(0xBE);  
    for ( i = 0; i < 2; i++) {           // we need 2 bytes
      data[i] = ds.read();
    } 
    unsigned int raw = (data[1] << 8) + data[0];
    Temp_c = (raw & 0xFFFC) * 0.0625; 
    Conv_start = false;
    return;
  } 
  return;
}

/************************************************
 * turn the output pin on/off based on pid output
 ************************************************/

void PID_HEAT (void){
  myPID.Compute();

  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  {                                     //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) digitalWrite(Heat,HIGH);
  else digitalWrite(Heat,LOW);

  return;
}

// setup function to set the Brauduino and Auto parameters

void SetupFun(void){
  byte pidMenu = 0;
  byte autoMenu = 0;
  byte setupMenu = 0;
  tempHAddr = 6;
  tempLAddr = 7;
  timeAddr = 8;
  blhpAddr = 40;
  int kpSet,kiSet,kdSet;
  double stgtmpSet;
  word stgtmpSetword;
  word windowSizeSet;
  int stgtimSet;
  byte hopSet;
  boolean setupLoop = true;
  boolean pidLoop = false;
  boolean hopLoop = false;
  boolean autotempLoop = false;
  boolean autotimeLoop = false;
  boolean eepromread = false;


  while (setupLoop){
    switch (setupMenu){ // to select between PID and Auto menu
      case(0):
      lcd.setCursor(0,0); 
      lcd.print("Unit Parameters ");
      if (digitalRead(Button_up)==0){
        delay(100);
        if (digitalRead(Button_dn)==0){
          setupLoop = false;
          return;
        }
        else{
          setupMenu = 1;
        }
      }
      if (digitalRead(Button_nxt)==0){   // set PID parameters
        kpSet=word(EEPROM.read(0),EEPROM.read(1));
        kiSet=word(EEPROM.read(2),EEPROM.read(3));
        kdSet=word(EEPROM.read(4),EEPROM.read(5));
        windowSizeSet=word(EEPROM.read(33),EEPROM.read(34));
        nmbrStgs = EEPROM.read(38);
        nmbrHops = EEPROM.read(39);
        while(digitalRead(Button_nxt)==0){
        }
        pidLoop = true;
        while(pidLoop){
          switch(pidMenu){
            case(0): //kP setting
            lcd.setCursor(0,1);
            lcd.print("Kp = ");
            lcd.print(kpSet);
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                kpSet++;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                kpSet--;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");                
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }
              EEPROM.write(0,highByte(kpSet));
              EEPROM.write(1,lowByte(kpSet));
              lcd.setCursor(0,1);
              lcd.print("                ");
              pidMenu = 1;
            }
            break;
            case(1)://ki setting
            lcd.setCursor(0,1);
            lcd.print("Ki = ");
            lcd.print(kiSet);
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                kiSet++;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");              
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                kiSet--;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");              
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }
              EEPROM.write(2,highByte(kiSet));
              EEPROM.write(3,lowByte(kiSet));
              lcd.setCursor(0,1);
              lcd.print("                ");             
              pidMenu = 2;
            }
            break;
            case(2)://kd setting
            lcd.setCursor(0,1);
            lcd.print("Kd = ");
            lcd.print(kdSet);
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                kdSet++;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                kdSet--;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }
              EEPROM.write(4,highByte(kdSet));
              EEPROM.write(5,lowByte(kdSet));
              lcd.setCursor(0,1);
              lcd.print("                ");    
              pidMenu = 3;

            }
            break;
            case(3)://window size setting
            lcd.setCursor(0,1);
            lcd.print("Window = ");
            lcd.print(windowSizeSet);
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                windowSizeSet+=500;
                if (windowSizeSet>5000) windowSizeSet = 5000;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                windowSizeSet-=500;
                if (windowSizeSet<500) windowSizeSet =500;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }
              EEPROM.write(33,highByte(windowSizeSet));
              EEPROM.write(34,lowByte(windowSizeSet));
              lcd.setCursor(0,1);
              lcd.print("                ");
              pidMenu = 4;

            }
            break;
            case(4)://number of stages setting
            lcd.setCursor(0,1);
            lcd.print("Num of steps = ");
            lcd.print(int(nmbrStgs));
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                nmbrStgs++;
                if(nmbrStgs>9)nmbrStgs=9;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                nmbrStgs--;
                if (nmbrStgs<1)nmbrStgs =1;
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }
              EEPROM.write(38,nmbrStgs);
              lcd.setCursor(0,1);
              lcd.print("                ");    
              pidMenu = 5;

            }
            break;
            case(5)://number of hop additons setting
            lcd.setCursor(0,1);
            lcd.print("Num of Hops = ");
            lcd.print(int(nmbrHops));
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                nmbrHops++;
                if (nmbrHops>8)nmbrHops =8;               
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                setupLoop = false;
                pidLoop = false;
                return;
              }
              else{
                nmbrHops--;
                if (nmbrHops>9)nmbrHops =0;                
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }
              EEPROM.write(39,nmbrHops);
              lcd.setCursor(0,1);
              lcd.print("                ");
              pidLoop = false;    
              pidMenu = 0;

            }
            break;            


          }
        } 
      }
      break;

      case(1):
      lcd.setCursor(0,0);
      lcd.print(" Auto Parameters");
      if (digitalRead(Button_dn)==0){
        delay(100);
        if(digitalRead(Button_up)==0){
          setupLoop = false;
          return;
        }
        else{
          setupMenu = 0;
        }
      }
      if (digitalRead(Button_nxt)==0){ // set auto parameters
        while(digitalRead(Button_nxt)==0){
        }
        nmbrStgs = EEPROM.read(38);
        tempHAddr = 6;
        tempLAddr = 7;
        timeAddr = 8;
        blhpAddr = 40;

        for (int i=0; i<nmbrStgs;i++){ // loops for the number of stages 
          stgtmpSet = word(EEPROM.read(tempHAddr),EEPROM.read(tempLAddr));
          stgtmpSet = stgtmpSet/16.0;
          autotempLoop = true;
          while (autotempLoop){  // loops for temp adjust
            lcd.setCursor(0,1);
            lcd.print(stgName[i]);
            lcd.print("Temp=");
            lcd.print(stgtmpSet);
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                autotempLoop = false;
                lcd.setCursor(0,1);
                lcd.print("               ");
                return;
              }
              else{
                stgtmpSet = stgtmpSet+0.25;
                if(stgtmpSet>100.0){
                  stgtmpSet=100.0;
                }
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                lcd.setCursor(0,1);
                lcd.print("               ");                
                setupLoop = false;
                autotempLoop = false;
                return;
              }
              else{
                stgtmpSet = stgtmpSet-0.25;
                if(stgtmpSet<20.0){
                  stgtmpSet=20.0;
                }
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }
              stgtmpSet = stgtmpSet*16;
              stgtmpSetword =word(stgtmpSet);
              EEPROM.write(tempHAddr,highByte(stgtmpSetword));
              EEPROM.write(tempLAddr,lowByte(stgtmpSetword));
              lcd.setCursor(0,1);
              lcd.print("                ");   
              autotempLoop = false; 
            }
          }
          autotimeLoop = true;
          stgtimSet = EEPROM.read(timeAddr);
          while (autotimeLoop){ // loops to adjust time setting

            lcd.setCursor(0,1);
            lcd.print(stgName[i]);
            lcd.print("time=");
            lcd.print(stgtimSet);
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                autotimeLoop = false;
                lcd.setCursor(0,1);
                lcd.print("               ");
                return;
              }
              else{
                stgtimSet = stgtimSet+1;
                if(stgtimSet>=120){
                  stgtimSet=120;
                }
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                lcd.setCursor(0,1);
                lcd.print("                ");                
                setupLoop = false;
                autotimeLoop = false;
                return;
              }
              else{
                stgtimSet = stgtimSet-1;
                if(stgtimSet<=0){
                  stgtimSet=0;
                }

                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }

              EEPROM.write(timeAddr,lowByte(stgtimSet));
              lcd.setCursor(0,1);
              lcd.print("                ");    
              autotimeLoop = false;
            }

          }
          tempHAddr+= 3;
          tempLAddr+= 3;
          timeAddr+= 3;
        }
        nmbrHops = EEPROM.read(39);
        nmbrHops+=1;
        for(int i =0;i<nmbrHops;i++){
          hopLoop = true;
          hopSet = EEPROM.read(blhpAddr);
          while (hopLoop){
            if (i==0){
              lcd.setCursor(0,1);
              lcd.print("Boil time = ");
              lcd.print(int (hopSet));
            }
            else{
              lcd.setCursor(0,1);
              lcd.print("Hop ");
              lcd.print(i);
              lcd.print(" time = ");
              lcd.print(int(hopSet));

            }
            if (digitalRead(Button_up)==0){
              delay(100);
              if (digitalRead(Button_dn)==0){
                setupLoop = false;
                hopLoop = false;
                lcd.setCursor(0,1);
                lcd.print("               ");
                return;
              }
              else{
                hopSet = hopSet+1;
                if(hopSet>=180){
                  hopSet=180;
                }
                delay(100);
                lcd.setCursor(0,1);
                lcd.print("               ");
              }
            }
            if (digitalRead(Button_dn)==0){
              delay(100);
              if(digitalRead(Button_up)==0){
                lcd.setCursor(0,1);
                lcd.print("                ");                
                setupLoop = false;
                hopLoop = false;
                return;
              }
              else{
                hopSet = hopSet-1;
                if(hopSet<=0){
                  hopSet=0;
                }

                delay(100);
                lcd.setCursor(0,1);
                lcd.print("                ");
              }
            }      
            if (digitalRead(Button_nxt)==0){
              while (digitalRead(Button_nxt)==0){
              }

              EEPROM.write(blhpAddr,hopSet);
              lcd.setCursor(0,1);
              lcd.print("                ");    
              hopLoop = false;
            }
          }
          blhpAddr+= 1;
        }


      }
    }
  }

}

void Buzzer1(void){
  digitalWrite (Buzz,HIGH);
  delay (1000);
  digitalWrite(Buzz,LOW);
  return;
}
void Buzzer2(void){
  digitalWrite(Buzz,HIGH);
  delay(600);
  digitalWrite(Buzz,LOW);
  delay(200);
  digitalWrite(Buzz,HIGH);
  delay(600);
  digitalWrite(Buzz,LOW);
  delay(200);
  digitalWrite(Buzz,HIGH);
  delay(600);
  digitalWrite(Buzz,LOW);
}







































