/*
brauduino semi automated single vessel RIMS
 created by s.mathison
 Copyright (C) 2012  Stephen Mathison
 
 compiled on Arduino V1.0
 
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
const char Button_up = A3;
const char Button_dn = A2;
const char  Button_prev = A1;
const char  Button_nxt = A0;


// outputs
const int Heat = 9;
const int Pump = 8;
const int Buzz = 10;


// global variables
unsigned int WindowSize;

unsigned long windowStartTime;
unsigned long start;

double Setpoint, Input, Output,eepromKp, eepromKi, eepromKd;
//boolean autoLoop = false; 
boolean manualLoop = false;
boolean waterAdd = false;
boolean Conv_start = false;
boolean mpump = false;
boolean mheat = false;
boolean wtBtn = false;
boolean autoEnter = false;
boolean tempReached = false;
boolean pumpRest = false;
//boolean boilLoop = false;
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
//byte minute;
//byte i;
byte Busy = 0;
byte nmbrStgs;
byte nmbrHops;
byte tempHAddr;
byte tempLAddr;
byte timeAddr;
byte blhpAddr;
byte hopAdd;

char* stgName[] ={
  "MashIn","Stage1","Stage2","Stage3","Stage4","Stage5","Stage6","Stage7","Stage8","Boil  "};

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


//****** start of the funtions************** 


void Buzzer(int number)
{
  for (int i=0; i < number; i++)
  {
    digitalWrite (Buzz,HIGH);
    delay (500);
    digitalWrite(Buzz,LOW);
    delay(100);
  }

}


void pause_stage(void){
  boolean stage_pause = false;
  if (Button_1sec_press(Button_prev)){
    Buzzer(1);
    stage_pause = true;
    digitalWrite(Heat,LOW);
    digitalWrite(Pump,LOW); 
      display_lcd(0,0,"     Paused    " );   
    while (stage_pause)
    {
      if (Button_1sec_press(Button_prev))stage_pause=false;
    }

  } 
}


void display_lcd (int pos , int line ,const char* lable){
  lcd.setCursor(pos,line);
  lcd.print(lable);


}





// 1 second button press
int Button_1sec_press (int Button_press){
  if (digitalRead(Button_press)==0){
    delay (1000);
    if (digitalRead(Button_press)==0){
      lcd.clear();
      while(digitalRead(Button_press)==0){
      }
      return 1;
    }
  }
  return 0;

}





// repeat button press
int Button_repeat (int Button_press){
  if (digitalRead(Button_press)==0){
    delay(200);
    return 1;
  }
  return 0;
}




// holds whilst button pressed 
int Button_hold_press (int Button_press){
  if (digitalRead (Button_press)==0){
    delay(50);
    while (digitalRead (Button_press)==0){
    }
    return 1;
  }
  return 0;
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
    for ( int i = 0; i < 2; i++) {           // we need 2 bytes
      data[i] = ds.read();
    } 
    unsigned int raw = (data[1] << 8) + data[0];
    Temp_c = (raw & 0xFFFC) * 0.0625; 
    Conv_start = false;
    return;
  } 

}




void PID_HEAT (void){
  myPID.Compute();

  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  {                                     //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) digitalWrite(Heat,HIGH);
  else digitalWrite(Heat,LOW);


}




void load_pid_settings (void)
{
  eepromKp = word(EEPROM.read(0),EEPROM.read(1));// read the PID settings from the EEPROM
  eepromKi = word(EEPROM.read(2),EEPROM.read(3));
  eepromKd = word(EEPROM.read(4),EEPROM.read(5));

  myPID.SetTunings(eepromKp,eepromKi,eepromKd); // send the PID settings to the PID
  WindowSize = word(EEPROM.read(33),EEPROM.read(34));
  myPID.SetOutputLimits(0, WindowSize);

}  





boolean wait_for_confirm (boolean& test)
{ 
  wtBtn = true;
  while (wtBtn){               // wait for comfirmation 
    if (Button_hold_press(Button_prev)){
      test = true;
      wtBtn = false;
      lcd.clear();
    }
    if (Button_hold_press(Button_nxt)){
      test = false;
      wtBtn = false;
      lcd.clear();

    }
  }
}





float change_temp(float& temp_change,int upper_limit,int lower_limit)
{
  // Increase set temp
  if (Button_repeat(Button_up)){
    if (temp_change>=100){
      temp_change++; 
    }
    else{  
      temp_change+=0.25;
    }
    if (temp_change > upper_limit)temp_change = upper_limit;
  }
  // decrease temp
  if (Button_repeat(Button_dn)) 
  {
    if(temp_change>=100){
      temp_change--;
    }
    else{
      temp_change-=0.25;
    }
    if ( temp_change < lower_limit) temp_change = lower_limit;
  }

}




void quit_mode (boolean& processLoop)
{
  if ((digitalRead(Button_dn)==0) && (digitalRead(Button_up)==0)){
    digitalWrite(Heat,LOW);
    digitalWrite(Pump,LOW);
    processLoop = false;
    lcd.clear();
  }

}





void heat_control(void)
{
  //turns heat on or off      
  if (Button_hold_press(Button_prev)){
    if (mheat==false){
      mheat = true;
      windowStartTime = millis();
    }
    else{
      mheat = false;
      digitalWrite(Heat,LOW);
    }
  }

}





void pump_control(void)
{
  //turns the pump on or off
  if (Button_hold_press(Button_nxt)){
    if (mpump == false){
      mpump = true;
      digitalWrite(Pump,HIGH);
    }
    else{
      mpump = false;
      digitalWrite(Pump,LOW);
    }
  }

}




void prompt_for_water (void){
  display_lcd(0,0,"  Water added?  ");
  Buzzer(3);
  display_lcd(0,1,"       Ok   Quit");

}



void pump_prime(void)
{
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

}


void pump_rest (int stage)
{
  if (stage==9){
    if (Temp_c<94.0) digitalWrite(Pump,HIGH);
    else digitalWrite(Pump,LOW);
    if (Temp_c >= 95)tempReached = true; 
  }
  else{
    pumptempError = stageTemp-Temp_c;
    if (pumptempError <= 0)tempReached = true;
    if ((pumpTime < 10)){ // starts pumps and heat
      digitalWrite(Pump,HIGH);
      pumpRest =false; 
    }
    if ((pumpTime >= 10)){ // pump rest
      digitalWrite(Pump,LOW);
      digitalWrite(Heat,LOW);
      pumpRest = true;
      if(pumpTime>=12 || (pumptempError > 1.0))pumpTime = 0;
    } 
  }
}

void check_for_resume(void){
  if(EEPROM.read(35)){ // read the auto started byte to see if it has been set and if so ask to resume

    display_lcd (0,0," Resume Process?");
    display_lcd (0,1,"        Yes   No");
    wait_for_confirm(resume);
    if(resume==true){
      tempHAddr = (EEPROM.read(36)*3)+6;
      tempLAddr = tempHAddr+1;
      timeAddr = tempHAddr+2;
      x = EEPROM.read(36);
      autoEnter = true;
      lcd.clear();

    }
  } 
}



void load_stage_settings (void){
  tempHAddr = 6; // setup intitial stage addresses 
  tempLAddr = 7;
  timeAddr = 8;
  nmbrStgs = EEPROM.read(38);// read the number of steps
  nmbrHops = EEPROM.read(39);//read the number of hop additions

}



void start_time (void)
{
  start = millis();
  // windowStartTime = millis();
  second = 0;
  // minute = 0;

}




void stage_timing (int stage)
{
  if ((millis()-start)>1000){  // timing routine
    start = millis();
    second++;
    if(!(tempReached))second=0;// starts counting down when temp reached
    if (second>59){
      display_lcd(10,0,"      ");
      second = 0;
      pumpTime++;
      if(stage == 0)pumpTime = 0;
      stageTime--;
      EEPROM.write(37,lowByte(stageTime));// saves stage time incase of interuption
    }
  }

}




void hop_add (void)
{
  if(hopAdd <= nmbrHops){
    if (stageTime == hopTime){
      Buzzer(3);
      lcd.clear();
      lcd.print("    Add  Hops");
      delay(2000);
      Buzzer(3);
      hopAdd++;
      EEPROM.write(49,hopAdd);
      blhpAddr++;
      hopTime = EEPROM.read(blhpAddr);
      lcd.clear();
    }
  } 
}



void stage_loop (int stage, float H_temp=80, float L_temp=30){
  int lastminute;
  while ((stageTime>0)&&(autoEnter)){
    lastminute=stageTime;
    stage_timing(stage);
    Temperature();// get temp
    Setpoint = stageTemp;//
    Input = Temp_c;
    pause_stage();
    if (pumpRest){
      display_lcd(0,0,"   Pump  Rest   ");
      display_lcd(0,1,"                ");
    }
    else{
      display_lcd(0,0,stgName[stage]);      
      display_lcd(6,0,"     ");
      display_lcd(11,0,"T=");
      lcd.print(stageTime);
      display_lcd(0,1,"S/A=");
      lcd.print(stageTemp);
      display_lcd(9,1,"/");
      lcd.print(Temp_c);
      lcd.write((uint8_t)0);
    }

    change_temp(stageTemp,H_temp,L_temp);

    pump_rest(stage);

    if (pumpRest==false)PID_HEAT();

    if (stage==9){
      if(stageTime<lastminute){
        hop_add();
      }
    }

    quit_mode (autoEnter);
  }


}




void get_stage_settings (void)
{
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

}




void add_malt (void)
{
  boolean malt;
  lcd.clear();
  digitalWrite(Pump,LOW);
  digitalWrite(Heat,LOW);
  display_lcd(0,0,"    Add Malt    ");
  Buzzer(3);
  display_lcd(0,1,"        Ok  Quit");
  wait_for_confirm(malt);
  if (malt==false){
    digitalWrite(Heat,LOW);
    digitalWrite(Pump,LOW);
    lcd.clear();
    delay(50);
    mainMenu=0;
    autoEnter = false;
  }
}


void remove_malt (void)
{
  boolean malt;
  lcd.clear();
  x = 9;               // used add to stage count on the final stage for the resume 
  EEPROM.write(36,lowByte(x)); // stores the stage number for the resume
  digitalWrite(Pump,LOW);
  digitalWrite(Heat,LOW);
  display_lcd(0,0,"  Remove  Malt  ");
  Buzzer(3);
  display_lcd(0,1,"        Ok  Quit");
  wait_for_confirm(malt);
  if (malt==false){
    stageTime = EEPROM.read(40);
    EEPROM.write(37,lowByte(stageTime));
    digitalWrite(Heat,LOW);
    digitalWrite(Pump,LOW);
    lcd.clear();
    delay(50);
    mainMenu=0;
    autoEnter = false;
  }
}






void get_boil_settings (void)
{
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

}



void manual_mode (void)
{

  load_pid_settings();

  prompt_for_water();

  wait_for_confirm(manualLoop);

  while (manualLoop){            // manual loop
    Temperature();
    Setpoint = mset_temp;
    Input = Temp_c;

    display_lcd(0,0,"  Manual Mode   ");
    display_lcd(0,1,"S/A=");
    lcd.print(mset_temp);
    display_lcd(9,1,"/");
    lcd.print(Temp_c);
    lcd.write((uint8_t)0);
    change_temp(mset_temp,120,20);
    quit_mode(manualLoop);
    heat_control();
    pump_control();
    if (mheat){
      PID_HEAT(); 
    }        
  }

}


void auto_mode (void)
{
  load_stage_settings();
  load_pid_settings();
  check_for_resume();
  if(!(resume)){  // if starting a new process prompt for water
    prompt_for_water();
    wait_for_confirm(autoEnter);
    if(!(autoEnter))return;
    pump_prime();
    x = 0;
  }
  if (autoEnter){     // mash steps
    EEPROM.write(35,1);// auto mode started
    for (int i = x;i < nmbrStgs;i++){
      EEPROM.write(36,lowByte(x)); // stores the stage number for the resume
      x++;                         // used to count the stages for the resume 
      tempReached = false;
      get_stage_settings();
      start_time(); 
      stage_loop(i);
      if (!(autoEnter)) break;
      if( i==0 && autoEnter){    // at the end of the mashIn step pauses to add grain
        add_malt();
        if (!(autoEnter))break;
      }
      if(i==(nmbrStgs-1)&& autoEnter){   // at the end of the last step pauses to remove the malt pipe before the boil
        remove_malt();
        if (!(autoEnter))break;
      }          
      Buzzer(1); 
      tempHAddr +=3; // increase stage addresses 
      tempLAddr +=3;
      timeAddr +=3;
      lcd.clear(); 
    }
  }
  // start of the boil
  if(autoEnter){
    start_time(); 
    stageTemp= 98.0; // set the intital boil temp to 98 deg c
    tempReached = false;  
    get_boil_settings();
    stage_loop(x,120,94);

    if(autoEnter){    // finishes the brewing process
      display_lcd(0,0,"    Brewing     ");
      display_lcd(0,1,"   Finished     ");
      Buzzer(3);
      delay(2000);
      EEPROM.write(35,0); // sets auto start byte to 0 for resume
      EEPROM.write(49,0); // sets hop count to 0
      mainMenu=0;
      autoEnter =false;
      resume =false;
    }


  }
}



void save_settings (int addr,int data)
{
  EEPROM.write(addr,highByte(data));
  EEPROM.write((addr+1),lowByte(data));

}  



void save_settings (int addr,byte data){

  EEPROM.write(addr,data);

}




int change_set(int& set_change,int upper_limit,int lower_limit,int step_size)
{
  // Increase set temp
  if (Button_repeat(Button_up)){
    set_change+=step_size;
    display_lcd(0,1,"                ");
  }
  if (set_change > upper_limit)set_change = upper_limit;

  // decrease temp
  if (Button_repeat(Button_dn))
  {
    set_change-=step_size;
    display_lcd(0,1,"                ");    
  }
  if ( set_change < lower_limit) set_change = lower_limit;
}


int change_set(byte& set_change,int upper_limit,int lower_limit,int step_size)
{
  // Increase set temp
  if (Button_repeat(Button_up)){
    set_change+=step_size;
    display_lcd(0,1,"                ");
  }
  if (set_change > upper_limit)set_change = upper_limit;

  // decrease temp
  if (Button_repeat(Button_dn))
  {
    set_change-=step_size;
    display_lcd(0,1,"                ");    
  }
  if ( set_change < lower_limit) set_change = lower_limit;
}




void unit_set (void)
{
  int param[] ={
    200,-200,1,200,-200,1,200,-200,1,5000,500,500,9,1,1,8,0,1                };
  int a = 0;
  boolean pidLoop = false;
  int pidSet,setaddr;
  int windowSizeSet;
  char* setName[] ={
    "Kp = ","Ki = ","Kd = ","Windowsize= ","Num of Stages=","Num of Hops="                };

  setaddr = 0;
  for(int i=0;i<6;i++){
    if((i>=0) && (i<=3)){
      if (i==3) setaddr = 33;  
      pidSet=word(EEPROM.read(setaddr),EEPROM.read((setaddr+1)));
    }
    if (i==4)setaddr = 38;
    if((i>=4) && (i<6)){
      pidSet= EEPROM.read(setaddr);
    }
    pidLoop= true;
    display_lcd(0,1,"                ");
    while (pidLoop){
      display_lcd(0,1,setName[i]); 
      lcd.print(pidSet);
      change_set(pidSet,param[a],param[a+1],param[a+2]);
      quit_mode(pidLoop);
      if (!(pidLoop))i=6;

      if(Button_hold_press(Button_nxt)){
        if (i >= 4){
          save_settings(setaddr,lowByte(pidSet));
          pidLoop = false;
        }
        else{
          save_settings(setaddr,pidSet);
          pidLoop = false;
        }
      }
    }
    if (i>=4){
      setaddr+=1;
    }
    else{
      setaddr+=2;
    }
    a+=3;
  }

}


void set_stages (void)
{
  boolean autotempLoop = false;
  boolean autotimeLoop = false;
  tempHAddr = 6;
  tempLAddr = 7;
  timeAddr = 8;  
  float stgtmpSet;
  int stgtmpSetword;
  int stgtimSet;
  nmbrStgs = EEPROM.read(38);

  for (int i=0; i<nmbrStgs;i++){ // loops for the number of stages 
    stgtmpSet = word(EEPROM.read(tempHAddr),EEPROM.read(tempLAddr));
    stgtmpSet = stgtmpSet/16.0;
    autotempLoop = true;
    while (autotempLoop){  // loops for temp adjust
      display_lcd(0,1,stgName[i]);
      lcd.print("Temp=");
      lcd.print(stgtmpSet);
      quit_mode(autotempLoop);
      if (autotempLoop == false){
        return;
      }
      change_temp(stgtmpSet,85,20);
      if (Button_hold_press(Button_nxt)){
        stgtmpSet = stgtmpSet*16;
        stgtmpSetword =word(stgtmpSet);
        save_settings(tempHAddr,stgtmpSetword);
        display_lcd(0,1,"                ");  
        autotempLoop = false; 
      }
    }
    autotimeLoop = true;
    stgtimSet = EEPROM.read(timeAddr);
    while (autotimeLoop){ // loops to adjust time setting
      display_lcd(0,1,stgName[i]);
      lcd.print(" time=");
      lcd.print(stgtimSet);
      quit_mode(autotimeLoop);
      if (autotimeLoop == false){
        return;
      }
      change_set(stgtimSet,120,0,1);

      if (Button_hold_press(Button_nxt)){
        save_settings(timeAddr,lowByte(stgtimSet));
        display_lcd(0,1,"                ");  
        autotimeLoop = false;
      }
    }
    tempHAddr+= 3;
    tempLAddr+= 3;
    timeAddr+= 3;
  }

}  

void set_hops (void)
{
  boolean hopLoop = false;
  blhpAddr = 40;
  byte hopSet;

  nmbrHops = EEPROM.read(39);
  nmbrHops+=1;

  for(int i =0;i<nmbrHops;i++){
    hopLoop = true;
    hopSet = EEPROM.read(blhpAddr);
    while (hopLoop){
      if (i==0){
        display_lcd(0,1,"Boil time = ");
        lcd.print(int (hopSet));
      }
      else{
        display_lcd(0,1,"Hop ");
        lcd.print(i);
        lcd.print(" time = ");
        lcd.print(int(hopSet));
      }
      quit_mode(hopLoop);
      if( hopLoop == false){
        return;
      }
      change_set(hopSet,180,0,1);

      if (Button_hold_press(Button_nxt)){
        save_settings(blhpAddr,hopSet);
        lcd.setCursor(0,1);
        lcd.print("                ");    
        hopLoop = false;
      }
    }
    blhpAddr+= 1;
  }

}



void auto_set(void)
{
  set_stages();
  set_hops();

}



void setup_mode (void)
{
  byte setupMenu = 0;
  boolean setupLoop = true;
  while (setupLoop){
    switch (setupMenu){ // to select between PID and Auto menu
      case(0):
      display_lcd(0,0,"Unit Parameters ");
      display_lcd(0,1,"                ");
      quit_mode(setupLoop);
      if (Button_hold_press(Button_up))setupMenu = 1;
      if (Button_hold_press(Button_nxt))unit_set();  
      break;

      case(1):
      display_lcd(0,0," Auto Parameters");
      display_lcd(0,1,"                ");
      quit_mode(setupLoop);
      if (Button_hold_press(Button_dn))setupMenu = 0;
      if (Button_hold_press(Button_nxt))auto_set();
      break;
    }
  }

}   






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
  switch(mainMenu){
    case (1):
    display_lcd(0,0,"  Manual Mode   ");
    display_lcd(0,1,"                ");
    delay (1000);
    lcd.clear();
    manual_mode();
    mainMenu = 0;

    break;

    case (2): 

    display_lcd(0,0,"   Auto Mode    ");
    display_lcd(0,1,"                ");
    delay (1000);
    lcd.clear();
    auto_mode();
    mainMenu = 0;  

    break;

    case (3):   

    display_lcd(0,0,"   Setup Mode   ");
    display_lcd(0,1,"                ");
    delay (1000);
    lcd.clear();
    setup_mode();
    mainMenu = 0;    

    break;

  default:      

    digitalWrite(Heat,LOW);
    digitalWrite(Pump,LOW);  
    Temperature();
    display_lcd(0,0," The Brauduino  ");
    display_lcd(0,1,"  Temp=");
    lcd.print(Temp_c);
    lcd.write((uint8_t)0);


    if (Button_1sec_press(Button_dn))mainMenu = 1;
    if (Button_1sec_press(Button_prev))mainMenu = 2;
    if (Button_1sec_press(Button_nxt))mainMenu = 3;
    break;    
  }


}














































