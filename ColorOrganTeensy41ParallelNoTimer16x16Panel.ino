#include <Arduino.h>
#include <SPI.h>
#include <arduinoFFT.h>
#include <FastLED.h>
#include <time.h>
#include <stdlib.h>
#include <limits.h>
#include "fl/assert.h"

FASTLED_USING_NAMESPACE

struct hexagonVerticeInfo
{
  int verticeRows[6];
  int verticeCols[6];
};

enum PrintType {FFT_Mag,AnalogSamples};
enum eUpDown{e_up,e_down};
enum eRightLeft{e_right,e_left};
enum eudrlBinMode{e_all,e_firstFour,e_secondFour,e_everyOtherOdd,e_everyOtherEven};

void setup();
void loop();
char* ReadSerialString(char* str);
void fillBins(int* bins);
int MapLeds_Strips(int row,int col);
int MapLeds(int row,int col);
void PrintDebug(enum PrintType pType);
void spectrumCorners(int* bins);
void spectrumCenterSideToSide(int* bins);
void spectrumCenterBottomToTop(int* bins);
void drawSquare(int ULRow,int ULCol, int Size, int hue, int fill);
void drawRect(int ULRow,int ULCol, int rowSize,int colSize, int hue, int fill);
void Squares(int* bins, int constrain);
void drawStar(int centerRow,int centerCol, int rsize, int hue);
void Stars(int* bins);
void drawCircle(int centerRow,int centerCol, int rsize, int hue);
void Circles(int* bins);
void drawColorCircle(int centerRow,int centerCol, int rsize, int* bins );
//void preCalcColorCircle(void);
void ColorCircles(int* bins);
void drawTriangle(int startRow,int startCol,int size,int direction,int binVal);
void drawTrianglesLR(int* bins);
void drawTrianglesUD(int* bins);
void drawLine(int startRow,int stopRow,int startCol,int stopCol,CHSV color);
void drawLinesUpDown(eUpDown upDown,enum eudrlBinMode udrlBinMode, int* bins);
void drawLinesLeftRight(enum eRightLeft LeftRight,enum eudrlBinMode udrlBinMode, int* bins);
void drawSparkles(int* bins);
void drawSpiral(int centerRow,int centerCol, int binNum,int binVal);
void drawSpirals(int* bins);
void drawSpinners(void);
void drawPersistentSpinners(int* bins);
void drawPersistentCircles(int* bins);
void drawPersistentSquares(int* bins);
void drawPersistentSpinningSquares(int* bins);
void drawPersistentSpinningSquares2(int* bins);
void drawPersistentSpinningSquare(int bin, int binval, int dir, int centerRow, int centerCol,struct VerticeInfo VIBase,int fnCtr);
void drawPersistentWiggles(int* bins);
void drawTimeDomain(int* bins);
void drawPersistentSpinningCircles(int* bins);
void drawPersistentBreathingCircles(int* bins);
void drawPersistentSpinningCircle(int bin, int binval, int dir, int centerRow, int centerCol,int radiusSize,int fnCtr);
void drawPersistentBreathingCircle(int bin, int binval, int dir, int centerRow, int centerCol,int radiusSize,int fnCtr);
void drawHexagons(int* bins);
void mapHexagon(int a, int hexrow, int hexcol, int* prow, int* pcol);
void setupHexagonVerticeInfoBase(hexagonVerticeInfo* VIBase, int a);
void drawPersistentHexagons(int* bins);
void drawPersistentSpinningHexagons(int* bins);
void drawPersistentSpinningHexagon(int bin, int binval, int dir, int centerRow, int centerCol,struct hexagonVerticeInfo VIBase,int fnCtr);
void drawDiagonals(int* bins);
//void drawDiagonalsVariableLength(int* bins);
void drawPersistentPulsatingSpinningDiagonals(int* bins);
//void drawPersistentPulsatingDiagonal(int bin, int binval, int dir, int centerRow, int centerCol,int fnCtr);
void drawPersistentPulsatingSpinningDiagonal(int bin, int binval, int dir, int centerRow, int centerCol,int fnCtr);
void fillTrigTables(void);
int checkTable(void);
int checkRowAndCol(int row, int col);
void collectAnalog(void);
void setup_i2s(void);

#define FFT_N 128 // set to 128 point fht
#define SAMPLINGFREQ 20000
#define USESERIAL 0

#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

#define NUM_ROWS_OF_PANELS 2
#define NUM_COLS_OF_PANELS 2
#define NUM_ROWS_PER_PANEL 16
#define NUM_COLS_PER_PANEL 16
#define NUM_LEDS_PER_COLUMN (NUM_COLS_OF_PANELS*NUM_COLS_PER_PANEL)
#define NUM_LEDS_PER_ROW (NUM_ROWS_OF_PANELS*NUM_ROWS_PER_PANEL)
#define NUM_LEDS_PER_PANEL (NUM_ROWS_PER_PANEL*NUM_COLS_PER_PANEL)
#define NUM_COLUMNS 32
#define NUM_SETS 2
#define NUM_ROWS_PER_SET 16
#define NUM_COLUMNS_PER_SET 32
#define NUM_LEDS_PER_SET (NUM_COLUMNS_PER_SET * NUM_ROWS_PER_SET)


#define NUM_AUDIO_BINS 8
#define DEFAULT_HUE 0
#define DEFAULT_SAT 255
#define DEFAULT_LUM 255
#define BRIGHTNESS          15

#define TIMINGSIG1 15
#define TIMINGSIG2 16
#define TIMINGSIG3 17
#define analogPin A0
#define LED_DATA_PIN 2

// defines for pins to use for LEDs
#define PIN_FIRST 0
#define PIN_SECOND 1
#define PIN_THIRD 2
#define PIN_FOURTH 3
#define IS_RGBW false

/* Variables for FFT and FFT timing */
int AnalogVals1[FFT_N];
volatile int val = 0;  // variable to store the value read
volatile int aCt=0;
volatile int CollectingFlag=1;
volatile boolean ledOn;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
float vReal[FFT_N];
float vImag[FFT_N];

/* LED array */
CRGB leds[NUM_LEDS_PER_COLUMN*NUM_COLUMNS];

CRGB *Set0=leds+NUM_LEDS_PER_SET*0;
CRGB *Set1=leds+NUM_LEDS_PER_SET*1;

/* Create FFT object */
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FFT_N, SAMPLINGFREQ);

float sinTable11p25Deg[32];
float cosTable11p25Deg[32];
float sinTable15Deg[24];
float cosTable15Deg[24];

unsigned long loopCtr=0;
unsigned long mode=0;

int persistentCirclesResetFlag=1;
int persistentSpinningSquaresResetFlag=1;
int persistentWigglesResetFlag=1;
int persistentSpinningCircles1ResetFlag=1;
int persistentSpinningCircles2ResetFlag=1;
int persistentHexagonsResetFlag=1;
int persistentSpinningHexagonsResetFlag=1;
int persistentPulsatingSpinningDiagonalsResetFlag=1;

int semiGlobalThreshold=250;

struct VerticeInfo
{
  int verticeRows[4];
  int verticeCols[4];
};

//******************************************************************************
void setup() 
{
  fillTrigTables();
  
  Serial.begin(9600); // use the serial port
  if(USESERIAL) delay(10000);
  Serial.println("Teensy Color Organ");

  delay(6000);  // The long reset time here is to make it easier to flash the device during the development process


  //Set Gain of MCP 6S28
  pinMode(10, OUTPUT); // set the CS pin as an output
  SPI.begin();         // initialize the SPI library
  digitalWrite(10, LOW);   // set the CS pin to LOW
  SPI.transfer(0x40);   // write to register
  // 0 = Gain of +1 (Default)
  // 1 = Gain of +2
  // 2 = Gain of +4
  // 3 = Gain of +5
  // 4 = Gain of +8
  // 5 = Gain of +10
  SPI.transfer(0x1);
  digitalWrite(10, HIGH);  // set the CS pin HIGH
  
  pinMode(TIMINGSIG1,OUTPUT);
  digitalWrite(TIMINGSIG1,LOW);

  pinMode(TIMINGSIG2,OUTPUT);
  digitalWrite(TIMINGSIG2,LOW);  

  pinMode(TIMINGSIG3, OUTPUT);
  digitalWrite(TIMINGSIG3,LOW);  
  //TimerStart(TC1, 0, TC3_IRQn, 20000);  

  CLEDController& c1 = FastLED.addLeds<WS2812, PIN_FIRST, GRB>(leds, NUM_LEDS_PER_SET);
  CLEDController& c2 = FastLED.addLeds<WS2812, PIN_SECOND, GRB>(leds+NUM_LEDS_PER_SET*1, NUM_LEDS_PER_SET);
  //CLEDController& c3 = FastLED.addLeds<WS2812, PIN_THIRD, GRB>(leds+NUM_LEDS_PER_SET*2, NUM_LEDS_PER_SET);
  //CLEDController& c4 = FastLED.addLeds<WS2812, PIN_FOURTH, GRB>(leds+NUM_LEDS_PER_SET*3, NUM_LEDS_PER_SET);

  if (IS_RGBW) {
    c1.setRgbw();
    c2.setRgbw();
    //c3.setRgbw();
    //c4.setRgbw();                
}

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  for(int i=0;i<NUM_LEDS_PER_ROW;++i)
  {
    for(int j=0;j<NUM_LEDS_PER_COLUMN;++j)
    {
        leds[MapLeds(i,j)]=CHSV(i*40,DEFAULT_SAT,DEFAULT_LUM);
    }
  }
  
  FastLED.show();  
  delay(5000);

  for(int i=0;i<NUM_LEDS_PER_COLUMN*NUM_COLUMNS;++i)
  {
    leds[i]=0;
  }
  FastLED.show();  

  for(int i=0;i<NUM_LEDS_PER_ROW;++i)
  {
    for(int j=0;j<NUM_LEDS_PER_COLUMN;++j)
    {
        leds[MapLeds(i,j)]=CHSV(40,DEFAULT_SAT,DEFAULT_LUM);
        FastLED.show();
        delay(10);
    }
  }
 
  FastLED.show();  
  delay(5000);

}
//******************************************************************************
void loop() 
{
  char str[40];
  int bins[NUM_AUDIO_BINS];
  static int combinedEffectList[20]={19,20,2,3,4,5,6,7,8,9,10,11,12,13,14,0,1,2,3,4};
  static int combinedEffectListSize=3;
  int randomMode=1;
  int numModes=27;
  //uint32_t T0,T1,T2,T3;

  digitalWrite(TIMINGSIG1,HIGH);
  delay(1);
  digitalWrite(TIMINGSIG1,LOW);
  //T0=micros();
  
  if(USESERIAL==1)
  {
    ReadSerialString(str);
  }
  
  digitalWrite(TIMINGSIG2,HIGH);  
  collectAnalog();
  digitalWrite(TIMINGSIG2,LOW);

  //T1=micros();

  //digitalWrite(TIMINGSIG2,HIGH);
  for(int i=0;i<FFT_N;++i)
  {
    //Serial.println(AnalogVals1[i]);
    vReal[i]=AnalogVals1[i];
    vImag[i]=0.0;
  }  
     
  //if(USESERIAL==1)PrintDebug(AnalogSamples);   
  
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  /* Weigh data */
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
  FFT.complexToMagnitude(); /* Compute magnitudes */
  
  //digitalWrite(TIMINGSIG2,LOW);

  // These are custom tweaks for the analog environment using the original analog hardware. They eliminate "spurious" signals.
  vReal[0]=0;
  vReal[1]=0;
  for(int i=0;i<FFT_N;++i)
  {
    vReal[i]-=150;
    if(vReal[i]<0)vReal[i]=0;
  }  
  
  // for(int i=6;i<12;++i)
  // {
  //   vReal[i]-=100;
  //   if(vReal[i]<0)vReal[i]=0;
  // }  
    
  if(USESERIAL==1)PrintDebug(FFT_Mag);

  for(int i=0;i<NUM_LEDS_PER_COLUMN*NUM_COLUMNS;++i)
  {
    leds[i]=0;
  }

  fillBins(bins);

  //digitalWrite(TIMINGSIG2,HIGH);
  for(int combinedEffectListIndex=0;combinedEffectListIndex<combinedEffectListSize;++combinedEffectListIndex)
  {
    mode=combinedEffectList[combinedEffectListIndex];
    switch(mode)
    {
      case 0:
        spectrumCorners(bins); 
      break;
      case 1:
        spectrumCenterSideToSide(bins);
      break;
      case 2:
        spectrumCenterBottomToTop(bins);
      break;
      case 3:
        Squares(bins,0);
      break;
      case 4:
        Stars(bins);
      break;
      case 5:
        Circles(bins);
      break;  
      case 6:
        ColorCircles(bins);    
      break;  
      case 7:
        spectrumCorners(bins);    
        spectrumCenterSideToSide(bins);
        spectrumCenterBottomToTop(bins);
      break;
      case 8:
        drawLinesUpDown(e_up,e_all,bins);    
        drawLinesUpDown(e_down,e_all,bins);    
      break;   
      case 9:
        drawLinesLeftRight(e_left,e_all,bins);    
        drawLinesLeftRight(e_right,e_all,bins);    
      break;  
      case 10: 
        drawTrianglesLR(bins);  
      break; 
      case 11: 
        drawTrianglesUD(bins);        
      break;
      case 12:
        drawSparkles(bins);
      break;
      case 13:
        drawSpirals(bins);
      break;
      case 14:
        drawPersistentSpinners(bins);
      break;
      case 15:
        drawPersistentCircles(bins);
      break;
      case 16:
        drawPersistentSquares(bins);
      break;
      case 17:
        drawPersistentSpinningSquares2(bins);
      break;   
      case 18:
        drawPersistentWiggles(bins);   
      break;
      case 19:
        drawTimeDomain(bins);
      break;
      case 20:
        drawPersistentSpinningCircles(bins);
      break;
      case 21:
        drawPersistentBreathingCircles(bins);
      break;
      case 22:
        drawHexagons(bins);
      break;
      case 23:
        drawPersistentHexagons(bins);
      break;      
      case 24:
        drawPersistentSpinningHexagons(bins);
      break;
      case 25:
        drawDiagonals(bins);
      break;
      case 26:
        drawPersistentPulsatingSpinningDiagonals(bins);
      break;
    }
  }
  //digitalWrite(TIMINGSIG2,LOW);
  
  //digitalWrite(TIMINGSIG2,HIGH);
  FastLED.show(); 
  //digitalWrite(TIMINGSIG2,LOW);
  //delay(8);

  // char printStr[30];
  // printStr[0]=0;

  ++loopCtr;
  if(loopCtr%40==0 && randomMode)
  {
    int newMode=random(numModes);
    combinedEffectList[random(combinedEffectListSize)]=newMode;
    // for(int i=0;i<3;++i)
    // {
    //   char tStr[5];
    //   itoa(combinedEffectList[i],tStr,10);
    //   strcat(printStr,tStr);
    //   strcat(printStr,"\t");
    // }
    // Serial.println(printStr);
    switch(newMode)
    {
      case 15:
        persistentCirclesResetFlag=1;
      break;
      case 17:
        persistentSpinningSquaresResetFlag=1;
      break;
      case 18:
        persistentWigglesResetFlag=1;
      break;
      case 20:
        persistentSpinningCircles1ResetFlag=1;
      break;
      case 21:
        persistentSpinningCircles2ResetFlag=1;
      break; 
      case 23:
        persistentHexagonsResetFlag=1;   
      break;
      case 24:
        persistentSpinningHexagonsResetFlag=1;  
      case 26:
        persistentPulsatingSpinningDiagonalsResetFlag =1;
      break;
    }
  }
  //Serial.println(T1-T0);
  //digitalWrite(TIMINGSIG1,LOW);
  delay(10);
}
//******************************************************************************
char* ReadSerialString(char* str)
{
  char c;
  int charCt=0;
  str[0]=0;
  while (!Serial.available());
  c = Serial.read();
  while (c!='\n')
  {
    //Serial.println(c);
    str[charCt]=c;
    ++charCt;
    while (!Serial.available());
    c = Serial.read();
  }
  str[charCt]=0;
  return(str);
}
//*******************************************************************************
void collectAnalog(void)
{
  uint32_t startTime;
  uint32_t period=(1.0/SAMPLINGFREQ)*1e6;
  int timingsigval=0;

  //Serial.print("");

  startTime=micros();
  if (startTime >=ULONG_MAX-period*FFT_N-10000)
  {
    delay(1000);
    startTime=micros();
  }
  AnalogVals1[0]=analogRead(analogPin); //This seems to stabilize the timing, results discarded
  startTime=micros();
  for(int i=0;i<FFT_N;++i)
  {
    timingsigval=!timingsigval;
    //digitalWrite(TIMINGSIG2,timingsigval);
    AnalogVals1[i]=analogRead(analogPin)-512;
    while(micros()-startTime<=(i+1)*period);
  }

//  for(int i=0;i<FFT_N;++i)
//  {
//    AnalogVals1[i]=analogRead(analogPin)/4-512;
//    delayMicroseconds(50);
//  }
}
//******************************************************************************
// void TC3_Handler()
// {
//    TC_GetStatus(TC1, 0);
//    digitalWrite(TIMINGSIG3, ledOn = !ledOn);
//    val=analogRead(analogPin); 
//    //AnalogVals1[aCt]=(val << 6)-32767; // form into a 16b signed int;
//    AnalogVals1[aCt]=val-511;
//    aCt=aCt+1;
//    if (aCt==FFT_N)
//    {
//      aCt=0;
//      NVIC_DisableIRQ(TC3_IRQn);
//      CollectingFlag=0;
// //    Serial.println("End Collection");
//    }
// }
//******************************************************************************
// void TimerStart(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq)
// {
//    pmc_set_writeprotect(false);
//    pmc_enable_periph_clk(irq);
//    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |
//                              TC_CMR_TCCLKS_TIMER_CLOCK4);
//    uint32_t rc = VARIANT_MCK / 128 / freq;
//    //TC_SetRA(tc, channel, rc >> 1); // 50% duty cycle square wave
//    TC_SetRC(tc, channel, rc);
//    TC_Start(tc, channel);
//    tc->TC_CHANNEL[channel].TC_IER=  TC_IER_CPCS | TC_IER_CPAS;
//    tc->TC_CHANNEL[channel].TC_IDR=~(TC_IER_CPCS | TC_IER_CPAS);
//    NVIC_EnableIRQ(irq);
// }
//******************************************************************************
void fillBins(int* bins)
{
  for(int i=0;i<NUM_AUDIO_BINS;++i)
  {
    bins[i]=0;
    for(int j=0;j<FFT_N/2/NUM_AUDIO_BINS;++j)
    {
      int vRealIndex=i*FFT_N/2/NUM_AUDIO_BINS+j;
      //Serial.print(vRealIndex);
      //Serial.print("\t");
      if(vReal[vRealIndex]>bins[i])
        bins[i]=vReal[vRealIndex];
    }
    //Serial.println();
  }  
}
//******************************************************************************
int MapLeds_Strips(int row,int col)
{
  int index;
  if(col%2==0)
    index=col*NUM_LEDS_PER_COLUMN+row;
  else
    index=col*NUM_LEDS_PER_COLUMN+NUM_LEDS_PER_COLUMN-row-1;

  return index;
}
//******************************************************************************
int MapLeds(int row,int col)
{
  int index;

  int base=col/NUM_COLS_PER_PANEL*NUM_LEDS_PER_PANEL+row/NUM_ROWS_PER_PANEL*NUM_LEDS_PER_PANEL;
  if(row%2==0)
  {
    index=row*NUM_ROWS_PER_PANEL;
    index=base+index+col%NUM_COLS_PER_PANEL;
  }
  else
  {
    index=row*NUM_ROWS_PER_PANEL+NUM_ROWS_PER_PANEL-1;
    index=base+index-col%NUM_COLS_PER_PANEL;
  }
  return index;
}
//******************************************************************************
void PrintDebug(enum PrintType pType)
{
  int i;
  switch (pType)
  {

    case FFT_Mag:
      for(i=0;i<FFT_N/2;++i)
      {
        Serial.println(vReal[i]);
      }
      break;

    case AnalogSamples:
      for(i=0;i<FFT_N;++i)
      {
        Serial.println(AnalogVals1[i]);
      }    
      break;
    
  }  
}


//******************************************************************************
void spectrumCorners(int* bins)
{
  //float ledStepVal=6000/NUM_LEDS_PER_COLUMN;
  float ledStepVal=125;
  int MaxLed;
  for(int b=0;b<NUM_AUDIO_BINS;++b)
  {
    MaxLed=bins[b]/ledStepVal;
    if(MaxLed>NUM_LEDS_PER_COLUMN)MaxLed=NUM_LEDS_PER_COLUMN-1;
    for(int j=0;j<MaxLed;++j)
    {
      leds[MapLeds(j,b)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      leds[MapLeds(j,NUM_COLUMNS-b-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      leds[MapLeds(NUM_LEDS_PER_COLUMN-j-1,b)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      leds[MapLeds(NUM_LEDS_PER_COLUMN-j-1,NUM_COLUMNS-b-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);         
    }
  }
}

//******************************************************************************
void spectrumCenterSideToSide(int* bins)
{
  //float ledStepVal=6000/NUM_COLUMNS/2;
  float ledStepVal=125;
  for(int b=0;b<NUM_AUDIO_BINS;++b)
  {
    int MaxLed=bins[b]/ledStepVal;
    if(MaxLed>NUM_COLUMNS/2)MaxLed=NUM_COLUMNS/2;
    for(int j=0;j<MaxLed;++j)    
    {
      leds[MapLeds(b+NUM_LEDS_PER_COLUMN/2,j+NUM_COLUMNS/2)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      leds[MapLeds(NUM_LEDS_PER_COLUMN/2-1-b,j+NUM_COLUMNS/2)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM); 
      leds[MapLeds(b+NUM_LEDS_PER_COLUMN/2,NUM_COLUMNS/2-j-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM); 
      leds[MapLeds(NUM_LEDS_PER_COLUMN/2-1-b,NUM_COLUMNS/2-j-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);         
    }
  }    
}

//******************************************************************************
void spectrumCenterBottomToTop(int* bins)
{
  //float ledStepVal=6000/64;
  float ledStepVal=125;
  int MaxLed;
  for(int b=0;b<NUM_AUDIO_BINS;++b)
  {
    MaxLed=bins[b]/ledStepVal;
    if(MaxLed>NUM_LEDS_PER_COLUMN/4)MaxLed=NUM_LEDS_PER_COLUMN/4;

    for(int j=0;j<MaxLed;++j)
    {
      // leds[MapLeds(45+j,b+NUM_COLUMNS/2)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      // leds[MapLeds(25-j,b+NUM_COLUMNS/2)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      // leds[MapLeds(45+j,NUM_COLUMNS/2-b-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      // leds[MapLeds(25-j,NUM_COLUMNS/2-b-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);     
      
      leds[MapLeds(NUM_LEDS_PER_COLUMN/2+j,b+NUM_COLUMNS/2)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      leds[MapLeds(NUM_LEDS_PER_COLUMN/2-1-j,b+NUM_COLUMNS/2)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      leds[MapLeds(NUM_LEDS_PER_COLUMN/2+j,NUM_COLUMNS/2-b-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);
      leds[MapLeds(NUM_LEDS_PER_COLUMN/2-1-j,NUM_COLUMNS/2-b-1)]=CHSV(30*b+100,DEFAULT_SAT,DEFAULT_LUM);

    }
  }
}
//******************************************************************************
void drawSquare(int ULRow,int ULCol, int Size, int hue, int fill)
{
 
  for(int col=ULCol;col<ULCol+Size;++col)
  {
    for(int row=ULRow;row>ULRow-Size;--row)
    {

      //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
      if(checkRowAndCol(row,col))
      {   
        if(fill)
          leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);      
        else
          if(col==ULCol || col==ULCol+Size-1)
            leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);
          else
            if(row==ULRow || row==ULRow-Size+1)
              leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);         
      }
    }
  }
}
//******************************************************************************
void drawRect(int ULRow,int ULCol, int rowSize,int colSize, int hue, int fill)
{
 
  for(int col=ULCol;col<ULCol+colSize;++col)
  {
    for(int row=ULRow;row>ULRow-rowSize;--row)
    {
      //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
      if(checkRowAndCol(row,col))
      {
        if(fill)
          leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);      
        else
          if(col==ULCol || col==ULCol+colSize-1)
            leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);
          else
            if(row==ULRow || row==ULRow-rowSize+1)
              leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);         
      }
    }
  }
}
//******************************************************************************
void Squares(int* bins, int constrain)
{

  int randRow,randCol,randRowSize,randColSize;
  for(int b=0;b<NUM_AUDIO_BINS;++b)
  {
    if(bins[b]>semiGlobalThreshold) //was 300
    {
      // depending on random 0 or 1 row size is random and column size is based on b amplitude
      // or vise versa
      if(random(2))
      {
        randRowSize=random(12)+2;
        randColSize=bins[b]/semiGlobalThreshold;       // was 150
      }
      else
      {
        randColSize=random(8)+2;
        randRowSize=bins[b]/semiGlobalThreshold;             // was 150
      }
        
      randRow=random(71);
      randCol=random(NUM_COLUMNS);

      if(constrain)  // this may need a relook
      {
        if(randRow-randRowSize+1<0)
          randRow=randRowSize-1;
        if(randCol+randColSize-1>=NUM_COLUMNS)
           randCol=NUM_COLUMNS-randColSize;
      }
  
      drawRect(randRow,randCol,randRowSize,randColSize,30*b,0);
    }
  }
}
//******************************************************************************
void drawStar(int centerRow,int centerCol, int rsize, int hue)
{
  int row,col;

  for(col=centerCol-rsize;col<=centerCol+rsize;++col)
  {
    //if(!(centerRow>=NUM_LEDS_PER_COLUMN || centerRow < 0 || col >= NUM_COLUMNS || col<0 ))
    if(checkRowAndCol(centerRow,col))
        leds[MapLeds(centerRow,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);
  }
          
  for(row=centerRow-rsize;row<=centerRow+rsize;++row)
  {
      //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || centerCol >= NUM_COLUMNS || centerCol<0 ))
      if(checkRowAndCol(row,centerCol))
        leds[MapLeds(row,centerCol)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);  
  }

  for(int i=-rsize;i<rsize+1;++i)
  {
    row= centerRow+i;
    col= centerCol+i;

    //if(!(col>=NUM_COLUMNS || col<0 || row>=NUM_LEDS_PER_COLUMN || row<0))
    if(checkRowAndCol(row,col))
      leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);
       
    row= centerRow-i;
    col= centerCol+i;
    //if(!(col>=NUM_COLUMNS || col<0 || row>=NUM_LEDS_PER_COLUMN || row<0))
    if(checkRowAndCol(row,col))
      leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);       
  }
}
//******************************************************************************
void Stars(int* bins)
{
  int randRow,randCol,randSize;
  for(int b=0;b<NUM_AUDIO_BINS;++b)
  {
    if(bins[b]>semiGlobalThreshold) //was 300
    {
      randCol=random(NUM_COLUMNS);
      randRow=random(NUM_LEDS_PER_COLUMN);
      randSize=bins[b]/semiGlobalThreshold;  //was 300
      drawStar(randRow,randCol,randSize,30*b);
    }
  }
}
//******************************************************************************
void drawCircle(int centerRow,int centerCol, int rsize, int hue)
{
  int row,col; 

  for(int tableIndex=0;tableIndex<32;tableIndex=tableIndex+1)
  {
    row=round(sinTable11p25Deg[tableIndex]*(rsize*1.25))+centerRow;
    col=round(cosTable11p25Deg[tableIndex]*(rsize))+centerCol;
    if(checkRowAndCol(row,col))
      leds[MapLeds(row,col)]=CHSV(hue,DEFAULT_SAT,DEFAULT_LUM);
  }
}
//******************************************************************************
void Circles(int* bins)
{
  int randRow,randCol,randSize;
  for(int b=0;b<NUM_AUDIO_BINS;++b)
  {
    if(bins[b]>semiGlobalThreshold)
    {
      // basic random
      randCol=random(NUM_COLUMNS);
      randRow=random(NUM_LEDS_PER_COLUMN);
      randSize=bins[b]/semiGlobalThreshold; // was 300
      drawCircle(randRow,randCol,randSize,30*b);
    }
  }
}
//******************************************************************************
void drawColorCircle(int centerRow,int centerCol, int rsize, int* bins )
{
  int row,col; 
  for(int tableIndex=0;tableIndex<32;++tableIndex)
  {
    for(int radius=1;radius<=rsize;++radius)
    {
      //if(bins[(radius-1)%8]>semiGlobalThreshold) //was 300
      if(bins[(radius-1)%8]>400) //was 300
      {
        row=round(sinTable11p25Deg[tableIndex]*(radius+radius/2))+centerRow;
        col=round(cosTable11p25Deg[tableIndex]*radius)+centerCol;
        //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
        if(checkRowAndCol(row,col))
          leds[MapLeds(row,col)]=CHSV(((radius-1)%8)*30+100,DEFAULT_SAT,DEFAULT_LUM);
      }
    }
  }
}
//******************************************************************************
void ColorCircles(int* bins)
{
  int randRow,randCol,randSize;

  for(int circleCt=0;circleCt<8;++circleCt)
  {
    randCol=random(NUM_COLUMNS);
    randRow=random(NUM_LEDS_PER_COLUMN);
//    randRow=35+(circleCt-3)*7;
//    randCol=8;
    randSize=8;
    drawColorCircle(randRow,randCol,randSize,bins);
  }
}
//******************************************************************************
void drawTriangle(int startRow,int startCol,int size,int direction,int binVal)
{  
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  
  int hue=(binVal)%256;
  //for(int i=0;i<size;++i)
  for(int i=0;i<size;++i)  
  {
    color.hue=(hue+i*10)%256;
    //color.val=random(201)+55;
   
    if(direction==0)
    {

      drawLine(startRow+i,startRow+size-1,startCol,startCol+size-i-1,color);      
      drawLine(startRow-1+size,startRow+(size-1)*2-i,startCol+size-i-1,startCol,color); 

    }
    else if(direction==1)
    {
      drawLine(startRow+i,startRow+size-1,startCol,startCol-size+i+1,color); 
      drawLine(startRow+size-1,startRow+(size-1)*2-i,startCol-size+1+i,startCol,color);
 
    }
    else if(direction==2)
    {
      drawLine(startRow,startRow+size-1-i,startCol+i,startCol+size-1,color);      
      drawLine(startRow,startRow+size-1-i,startCol+(size-1)*2,startCol+size-1,color);      
    }    
    else if(direction==3)
    {
      drawLine(startRow,startRow-size+1+i,startCol+i,startCol+size-1,color);      
      drawLine(startRow-size+1,startRow,startCol+size-1,startCol+(size-1)*2-i,color);      
    }        
  } 
}

//******************************************************************************
void drawTrianglesLR(int* bins)
{
  //char str[40];
  
  for(int i=0;i<NUM_AUDIO_BINS;++i)
  {
    if(bins[i]>semiGlobalThreshold) //was 250
    {
      drawTriangle(random(72),random(NUM_COLUMNS),bins[i]/250+1,1,bins[i]);
      drawTriangle(random(72),random(NUM_COLUMNS),bins[i]/250+1,2,bins[i]);            
    }
  }
}
//******************************************************************************
void drawTrianglesUD(int* bins)
{
  //char str[40];
  
  for(int i=0;i<NUM_AUDIO_BINS;++i)
  {
    if(bins[i]>semiGlobalThreshold) //was 250
    {
      drawTriangle(random(72),random(NUM_COLUMNS),bins[i]/250+1,2,bins[i]);
      drawTriangle(random(72),random(NUM_COLUMNS),bins[i]/250+1,3,bins[i]);            

    }
  }
}
//*******************************************************************************
void drawLine(int startRow,int stopRow,int startCol,int stopCol,CHSV color)
{
  int col,row,deltaRow,deltaCol;
  float m;   
  
  //x=col y=row
  deltaRow=stopRow-startRow;
  deltaCol=stopCol-startCol;
  
  m=(deltaRow)/((float)deltaCol);

  if(isnan(m)) m=0;
  
  if(abs(deltaRow)>abs(deltaCol))
  {
    if(startRow>stopRow)
    {
      int temp=startRow;
      startRow=stopRow;
      stopRow=temp;
      temp=startCol;
      startCol=stopCol;
      stopCol=temp;
    }
   
    for(row=startRow;row<=stopRow;row=row+1)
    {
      int col=round((row-startRow)/m+startCol);
      //color.hue=color.hue+(row-startRow)*2;
      //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
      if(checkRowAndCol(row,col))
        leds[MapLeds(row,col)]=CHSV(color);     
    }
  }
  else
  {
    if(startCol>stopCol)
    {
      int temp=startRow;
      startRow=stopRow;
      stopRow=temp;
      temp=startCol;
      startCol=stopCol;
      stopCol=temp;
    }
    for(col=startCol;col<=stopCol;col=col+1)
    {
      int row=round(m*(col-startCol)+startRow); 
           
      //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
      //color.hue=color.hue+(row-startRow)*2;
      if(checkRowAndCol(row,col))
        leds[MapLeds(row,col)]=CHSV(color); 
    }
  }
}
//******************************************************************************
void drawLinesUpDown(eUpDown upDown,enum eudrlBinMode udrlBinMode, int* bins)
{
  //startRow,stopRow,startCol,stopCol

  int rowLen,colLen;
  int startRow, startCol;
  int stopRow, stopCol;
  int startBinIndex,stopBinIndex,incBinIndex;

  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};

  incBinIndex=0;
  startBinIndex=0;
  stopBinIndex=0;

  switch (udrlBinMode)
  {
    case e_all:
      startBinIndex=0;
      stopBinIndex=8;
      incBinIndex=1;
      break;      
    case e_firstFour:
      startBinIndex=0;
      stopBinIndex=4;
      incBinIndex=1;
      break;
    case e_secondFour:
      startBinIndex=4;
      stopBinIndex=8;
      incBinIndex=1;
      break;      
    case e_everyOtherEven:
      startBinIndex=0;
      stopBinIndex=8;
      incBinIndex=2;
      break;        
   case e_everyOtherOdd:
      startBinIndex=1;
      stopBinIndex=8;
      incBinIndex=2;
      break;                
  }  
  
  

  // Bottom up
  if(upDown==e_up)
  {
    for(int bin=startBinIndex;bin<stopBinIndex;bin+=incBinIndex)
    {
      if(bins[bin]>semiGlobalThreshold)  //was 150
      {
        startRow=0;
        startCol=random(NUM_COLUMNS);
        rowLen=bins[bin]/200+1;
        colLen=random(3)*(random(2)? -1:1);  
        for(int segment=0;segment<8;++segment)
        {     
          stopRow=startRow+rowLen;
          stopCol=startCol+colLen;
          color.hue=bin*30;
          drawLine(startRow,stopRow,startCol,stopCol,color);
          startRow=stopRow;
          startCol=stopCol;
          colLen=random(3)*(random(2)? -1:1);
    
        }       
      }
    }
  }
  if(upDown==e_down)
  {
    // top down
    for(int bin=startBinIndex;bin<stopBinIndex;bin+=incBinIndex)
    {
      if(bins[bin]>semiGlobalThreshold)  //was 150
      {
        //Serial.println();
        startRow=70;
        startCol=random(NUM_COLUMNS);
        rowLen=-bins[bin]/200-1;
        colLen=random(3)*(random(2)? -1:1);  
        for(int segment=0;segment<8;++segment)
        {
          stopRow=startRow+rowLen;
          stopCol=startCol+colLen;   

          color.hue=bin*30;
          drawLine(startRow,stopRow,startCol,stopCol,color);
          
          startRow=stopRow;
          startCol=stopCol;
  
          colLen=random(3)*(random(2)? -1:1);
    
        }       
      }
    }  
  }
}

//******************************************************************************
void drawLinesLeftRight(enum eRightLeft LeftRight,enum eudrlBinMode udrlBinMode, int* bins)
{
  //startRow,stopRow,startCol,stopCol
  enum eudrlBinMode{e_all,e_firstFour,e_secondFour,e_everyOtherOdd,e_everyOtherEven};

  int rowLen,colLen;
  int startRow, startCol;
  int stopRow, stopCol;
  int startBinIndex,stopBinIndex,incBinIndex;
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};

  incBinIndex=0;
  startBinIndex=0;
  stopBinIndex=0;

  switch (udrlBinMode)
  {
    case e_all:
     startBinIndex=0;
      stopBinIndex=8;
      incBinIndex=1;
      break;      
    case e_firstFour:
      startBinIndex=0;
      stopBinIndex=4;
      incBinIndex=1;
      break;
    case e_secondFour:
      startBinIndex=4;
      stopBinIndex=8;
      incBinIndex=1;
      break;      
    case e_everyOtherEven:
      startBinIndex=0;
      stopBinIndex=8;
      incBinIndex=2;
      break;        
   case e_everyOtherOdd:
      startBinIndex=1;
      stopBinIndex=8;
      incBinIndex=2;
      break;                
  }  

  // Bottom up
  if(LeftRight==e_right)
  {
    for(int bin=startBinIndex;bin<stopBinIndex;bin+=incBinIndex)
    {
      if(bins[bin]>semiGlobalThreshold) //was 150
      {
        startRow=random(71);
        startCol=0;
        colLen=bins[bin]/800+1;//was 200
        rowLen=random(5)*(random(2)? -1:1);  
        for(int segment=0;segment<8;++segment)
        {     
          stopRow=startRow+rowLen;
          stopCol=startCol+colLen;
          color.hue=bin*30;     
          drawLine(startRow,stopRow,startCol,stopCol,color);
          startRow=stopRow;
          startCol=stopCol;
          rowLen=random(5)*(random(2)? -1:1);
    
        }       
      }
    }
  }
  if(LeftRight==e_left)
  {
    // top down
    for(int bin=startBinIndex;bin<stopBinIndex;bin+=incBinIndex)
    {
      if(bins[bin]>semiGlobalThreshold)  // was 150
      {
        //Serial.println();
        startRow=random(71);
        startCol=NUM_COLUMNS-1;
        colLen=-bins[bin]/800-1;//was200
        rowLen=random(5)*(random(2)? -1:1);  
        for(int segment=0;segment<8;++segment)
        {
          stopRow=startRow+rowLen;
          stopCol=startCol+colLen;   
          
          color.hue=bin*30;
          drawLine(startRow,stopRow,startCol,stopCol,color);
          
          startRow=stopRow;
          startCol=stopCol;
  
          rowLen=random(5)*(random(2)? -1:1);
    
        }       
      }
    }  
  }
}
//******************************************************************************
void drawSparkles(int* bins)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  int row,col;
  
  for(int i=0;i<NUM_AUDIO_BINS;++i)
  {
    if(bins[i]>semiGlobalThreshold) //was 250
    {
      color.hue=i*30;
      for(int j=0;j<bins[i]/125;++j)
      {
        row=random(71);
        col=random(NUM_COLUMNS);
        //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
        if(checkRowAndCol(row,col))
          leds[MapLeds(row,col)]=CHSV(color);  
      }          
    }
  }
}
//******************************************************************************
void drawSpiral(int centerRow,int centerCol, int binNum,int binVal)
{
  //r = aθ; polar form of equation of a spiral
  
  float thetaPart1=2*PI/16;
  float a=.1;  
  
  for(int i=0;i<64;++i)
  {
    float theta=thetaPart1*i;
    //float r=3+a*theta;
    float r=binVal/250+a*theta;        
    int x=cosTable11p25Deg[(i*2)%32]*r;
    int y=sinTable11p25Deg[(i*2)%32]*r;    
    int row=y+centerRow;
    int col=x+centerCol;
    //if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
    if(checkRowAndCol(row,col))
      leds[MapLeds(row,col)]=CHSV(i*2+binNum*30,DEFAULT_LUM,DEFAULT_SAT);  
  }
}
//******************************************************************************
void drawSpirals(int* bins)
{

  int centerRow=random(72);
  int centerCol=random(NUM_COLUMNS);
  
  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {
    if(bins[bin]>semiGlobalThreshold)  //was 250
    {
      drawSpiral(centerRow,centerCol,bin,bins[bin]);      
    }
  }
}
//******************************************************************************
void drawPersistentSpinners(int* bins)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  static int angleCt=0;
  static int centerRow[NUM_AUDIO_BINS]={0,0,0,0,0,0,0,0};
  static int centerCol[NUM_AUDIO_BINS]={0,0,0,0,0,0,0,0};
  static int startAngleIndex[NUM_AUDIO_BINS]={0,0,0,0,0,0,0,0};


  if(loopCtr%16==0)
  {
    for(int i=0;i<NUM_AUDIO_BINS;++i)
    {
      centerRow[i]=random(71);
      centerCol[i]=random(NUM_COLUMNS);
      startAngleIndex[i]=random(8)*4;
    }
  }
//
  angleCt=(angleCt+4)%32;
  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {
    if(bins[bin]>semiGlobalThreshold) // was 125
    {
      if((loopCtr%1)==0)
      {
        color.hue=bin*30;
        int row=sinTable11p25Deg[(angleCt+startAngleIndex[bin])%32]*((bins[bin]/125)*2*1.4);
        int col=cosTable11p25Deg[(angleCt+startAngleIndex[bin])%32]*(bins[bin]/125)*2;
        drawLine(centerRow[bin],centerRow[bin]+row,centerCol[bin],centerCol[bin]+col,color);      
      }
    }
  }
}

//******************************************************************************
void drawPersistentCircles(int* bins)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  static int angleCt=0;
  static int pcCtr=0;
  //int trailLength=8;

  struct PCirclesInfo
  {
    int centerRow;
    int centerCol;
    int radius;
    int dir;
    int trailLength;
    int startAngleIndex;
    int lastRows[32];
    int lastCols[32];
  };

  static struct PCirclesInfo PCI[8]=
  {
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},   
  };

  // restart the persistent circles on reset flag of persistent circles counter %32==0
  //Serial.println(persistentCirclesResetFlag);
  if(persistentCirclesResetFlag==1  || pcCtr%32==0)
  {
    persistentCirclesResetFlag=0;
    for(int i=0;i<NUM_AUDIO_BINS;++i)
    {
      PCI[i].centerRow=random(71);
      PCI[i].centerCol=random(NUM_COLUMNS);
      PCI[i].startAngleIndex=random(8)*4;
      PCI[i].dir=random(2);
      PCI[i].radius=bins[i]/500+4;
      //PCI[i].radius=5;
      PCI[i].trailLength=random(8)+1;
      for(int j=0;j<PCI[i].trailLength;++j)
      {
        PCI[i].lastRows[j]=-1;
        PCI[i].lastCols[j]=-1;
      }

      pcCtr=0;   
    }
  }

//
  angleCt=(angleCt+2)%32;
  //int colorOffset=(pcCtr%trailLength)*20;
  int colorOffset=0;

  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {
    if(bins[bin]>semiGlobalThreshold) //was 125
    {
      if(loopCtr%1==0)
      {
        color.hue=bin*30+colorOffset;

        // CW or CCW circles
        int angleCt2;
        if(PCI[bin].dir==0)
          angleCt2=32-angleCt;
        else
          angleCt2=angleCt;

        // calculate row and column
        int radius=PCI[bin].radius;
        int row=round(sinTable11p25Deg[((angleCt2)+PCI[bin].startAngleIndex)%32]*radius*1.5)+PCI[bin].centerRow;
        int col=round(cosTable11p25Deg[((angleCt2)+PCI[bin].startAngleIndex)%32]*radius)+PCI[bin].centerCol;              

        // shift arrays containing previous rows and cols
        for(int i=0;i<PCI[bin].trailLength-1;++i)
        {
          PCI[bin].lastRows[i]=PCI[bin].lastRows[i+1];
          PCI[bin].lastCols[i]=PCI[bin].lastCols[i+1];            
        }
        // put new row and col values in arrays
        PCI[bin].lastRows[PCI[bin].trailLength-1]=row;
        PCI[bin].lastCols[PCI[bin].trailLength-1]=col; 

        // update led array
        for(int i=0;i<PCI[bin].trailLength;++i)
        {
          if(checkRowAndCol(PCI[bin].lastRows[i],PCI[bin].lastCols[i]))
            leds[MapLeds(PCI[bin].lastRows[i],PCI[bin].lastCols[i])]=CHSV(color);  
        } 
      }
    }
    // // update led array
    // for(int i=0;i<trailLength;++i)
    // {
    //   if(checkRowAndCol(PCI[bin].lastRows[i],PCI[bin].lastCols[i]))
    //     leds[MapLeds(PCI[bin].lastRows[i],PCI[bin].lastCols[i])]=CHSV(color);  
    // } 
  }
  ++pcCtr;
}
//******************************************************************************
void drawPersistentSquares(int* bins)
{
  static int row1=0;
  static int col1=0;
  static int row2=0;
  static int col2=0;
  int size=2;
  int rowSize=size*1.5;

  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};

  if(loopCtr%32==0)
  {
    row1=random(NUM_LEDS_PER_COLUMN);
    col1=random(NUM_COLUMNS);
    row2=random(NUM_LEDS_PER_COLUMN);
    col2=random(NUM_COLUMNS);
  }

  for(int i=0;i<NUM_AUDIO_BINS/2;++i)
  {
    if(bins[i]>125)
    {
      color.hue=30*i;
      drawLine(row1+rowSize+i,row1+rowSize+i,col1-size-i,col1+size+i,color);
      drawLine(row1+rowSize+i,row1-rowSize-i,col1+size+i,col1+size+i,color);
      drawLine(row1-rowSize-i,row1-rowSize-i,col1+size+i,col1-size-i,color);
      drawLine(row1+rowSize+i,row1-rowSize-i,col1-size-i,col1-size-i,color);  
    }
  }

   for(int i=NUM_AUDIO_BINS/2;i<NUM_AUDIO_BINS;++i)
  {
    if(bins[i]>125)
    {
      int offset=i-NUM_AUDIO_BINS/2;
      color.hue=30*i*2;
      drawLine(row2+rowSize+offset,row2+rowSize+offset,col2-size-offset,col2+size+offset,color);
      drawLine(row2+rowSize+offset,row2-rowSize-offset,col2+size+offset,col2+size+offset,color);
      drawLine(row2-rowSize-offset,row2-rowSize-offset,col2+size+offset,col2-size-offset,color);
      drawLine(row2+rowSize+offset,row2-rowSize-offset,col2-size-offset,col2-size-offset,color);  
    }
  }
}
//******************************************************************************
void drawPersistentSpinningSquares(int* bins)
{
  static int centerRows[8];
  static int centerCols[8];
  static int fnCtr=0;

  struct VerticeInfo
  {
    int verticeRows[4];
    int verticeCols[4];
  };

  static struct VerticeInfo VIBase=
  {
    {0,0,0,0},{0,0,0,0}, 
  };
  static struct VerticeInfo VINew;

  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};

  if(persistentSpinningSquaresResetFlag==1 || fnCtr%32==0)
  {
    for(int i=0;i<8;++i)
    {
      centerRows[i]=random(NUM_LEDS_PER_COLUMN);
      centerCols[i]=random(NUM_COLUMNS);
    } 
    persistentSpinningSquaresResetFlag=0;
    fnCtr=0;   
  }


  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {

    if(bins[bin]>125)
    {

      // To rotate a point (x, y) by an angle θ around the origin (0, 0), you'll use trigonometry to calculate the new coordinates (x', y').
      //     x' = x * cos(θ) - y * sin(θ)
      //     y' = x * sin(θ) + y * cos(θ)
      
      int sizeOffset=bins[bin]/125;
      if(sizeOffset>=16)sizeOffset=32-sizeOffset;
      color.hue=bin*30+sizeOffset*30;

      int dir=random(2);
      //dir=1;
      int trigTableIndex;
      if(dir==0)
        trigTableIndex=(16-(fnCtr%16))*2;
      else
        trigTableIndex=(fnCtr%16)*2;
      
      VINew.verticeRows[0]=round((VIBase.verticeCols[0]+sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[0]+sizeOffset)*cosTable11p25Deg[trigTableIndex]);
      VINew.verticeRows[1]=round((VIBase.verticeCols[1]-sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[1]+sizeOffset)*cosTable11p25Deg[trigTableIndex]);
      VINew.verticeRows[2]=round((VIBase.verticeCols[2]-sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[2]-sizeOffset)*cosTable11p25Deg[trigTableIndex]);
      VINew.verticeRows[3]=round((VIBase.verticeCols[3]+sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[3]-sizeOffset)*cosTable11p25Deg[trigTableIndex]);  

      VINew.verticeCols[0]=round((VIBase.verticeCols[0]+sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[0]+sizeOffset)*sinTable11p25Deg[trigTableIndex]);
      VINew.verticeCols[1]=round((VIBase.verticeCols[1]-sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[1]+sizeOffset)*sinTable11p25Deg[trigTableIndex]);
      VINew.verticeCols[2]=round((VIBase.verticeCols[2]-sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[2]-sizeOffset)*sinTable11p25Deg[trigTableIndex]);
      VINew.verticeCols[3]=round((VIBase.verticeCols[3]+sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[3]-sizeOffset)*sinTable11p25Deg[trigTableIndex]);                                                              
      
      drawLine(centerRows[bin]+VINew.verticeRows[0],centerRows[bin]+VINew.verticeRows[1],centerCols[bin]+VINew.verticeCols[0],centerCols[bin]+VINew.verticeCols[1],color);
      //color.hue=60;
      drawLine(centerRows[bin]+VINew.verticeRows[1],centerRows[bin]+VINew.verticeRows[2],centerCols[bin]+VINew.verticeCols[1],centerCols[bin]+VINew.verticeCols[2],color);  
      //color.hue=80; 
      drawLine(centerRows[bin]+VINew.verticeRows[2],centerRows[bin]+VINew.verticeRows[3],centerCols[bin]+VINew.verticeCols[2],centerCols[bin]+VINew.verticeCols[3],color);
      //color.hue=100;
      drawLine(centerRows[bin]+VINew.verticeRows[3],centerRows[bin]+VINew.verticeRows[0],centerCols[bin]+VINew.verticeCols[3],centerCols[bin]+VINew.verticeCols[0],color);
    }
  }
  ++fnCtr;
}
//******************************************************************************
void drawPersistentSpinningSquares2(int* bins)
{
  static int centerRows[8];
  static int centerCols[8];
  static int fnCtr=0;

  // struct VerticeInfo
  // {
  //   int verticeRows[4];
  //   int verticeCols[4];
  // };

  static struct VerticeInfo VIBase=
  {
    {0,0,0,0},{0,0,0,0}, 
  };

  //CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};

  if(persistentSpinningSquaresResetFlag==1 || fnCtr%32==0)
  {
    for(int i=0;i<8;++i)
    {
      centerRows[i]=random(NUM_LEDS_PER_COLUMN);
      centerCols[i]=random(NUM_COLUMNS);
    } 
    persistentSpinningSquaresResetFlag=0;
    fnCtr=0;   
  }


  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {

    if(bins[bin]>125)
    {

      int dir=random(2);

      drawPersistentSpinningSquare(bin, bins[bin],dir,centerRows[bin],centerCols[bin],VIBase,fnCtr);
    }
  }
  ++fnCtr;
}
//******************************************************************************
void drawPersistentSpinningSquare(int bin, int binval, int dir, int centerRow, int centerCol,struct VerticeInfo VIBase,int fnCtr)
{

  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  static struct VerticeInfo VINew;

  // To rotate a point (x, y) by an angle θ around the origin (0, 0), you'll use trigonometry to calculate the new coordinates (x', y').
  //     x' = x * cos(θ) - y * sin(θ)
  //     y' = x * sin(θ) + y * cos(θ)
  
  int sizeOffset=binval/125;
  if(sizeOffset>=16)sizeOffset=32-sizeOffset;
  color.hue=bin*30+sizeOffset*30;

  int trigTableIndex;
  if(dir==0)
    trigTableIndex=(15-(fnCtr%16))*2;
  else
    trigTableIndex=(fnCtr%16)*2;
  
  VINew.verticeRows[0]=round((VIBase.verticeCols[0]+sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[0]+sizeOffset)*cosTable11p25Deg[trigTableIndex]);
  VINew.verticeRows[1]=round((VIBase.verticeCols[1]-sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[1]+sizeOffset)*cosTable11p25Deg[trigTableIndex]);
  VINew.verticeRows[2]=round((VIBase.verticeCols[2]-sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[2]-sizeOffset)*cosTable11p25Deg[trigTableIndex]);
  VINew.verticeRows[3]=round((VIBase.verticeCols[3]+sizeOffset)*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[3]-sizeOffset)*cosTable11p25Deg[trigTableIndex]);  

  VINew.verticeCols[0]=round((VIBase.verticeCols[0]+sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[0]+sizeOffset)*sinTable11p25Deg[trigTableIndex]);
  VINew.verticeCols[1]=round((VIBase.verticeCols[1]-sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[1]+sizeOffset)*sinTable11p25Deg[trigTableIndex]);
  VINew.verticeCols[2]=round((VIBase.verticeCols[2]-sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[2]-sizeOffset)*sinTable11p25Deg[trigTableIndex]);
  VINew.verticeCols[3]=round((VIBase.verticeCols[3]+sizeOffset)*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[3]-sizeOffset)*sinTable11p25Deg[trigTableIndex]);                                                              
  
  drawLine(centerRow+VINew.verticeRows[0],centerRow+VINew.verticeRows[1],centerCol+VINew.verticeCols[0],centerCol+VINew.verticeCols[1],color);
  drawLine(centerRow+VINew.verticeRows[1],centerRow+VINew.verticeRows[2],centerCol+VINew.verticeCols[1],centerCol+VINew.verticeCols[2],color);  
  drawLine(centerRow+VINew.verticeRows[2],centerRow+VINew.verticeRows[3],centerCol+VINew.verticeCols[2],centerCol+VINew.verticeCols[3],color);
  drawLine(centerRow+VINew.verticeRows[3],centerRow+VINew.verticeRows[0],centerCol+VINew.verticeCols[3],centerCol+VINew.verticeCols[0],color);
}
//******************************************************************************
void drawPersistentWiggles(int* bins)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  static int angleCt=0;
  static int pwCtr=0;
  int row,col;

  struct PWigglesInfo
  {
    int startRow;
    int startCol;
    int radius;
    int dir;
    int trailLength;
    int startAngleIndex;
    int lastRows[32];
    int lastCols[32];
  };

  static struct PWigglesInfo PWI[8]=
  {
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},
    {0,0,0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},   
  };


  //if(persistentWigglesResetFlag==1  || pwCtr%32==0)
  //if(persistentWigglesResetFlag==1  || pwCtr%NUM_LEDS_PER_COLUMN==0)
  if(persistentWigglesResetFlag==1)
  {
    persistentWigglesResetFlag=0;
    angleCt=0;
    pwCtr=0;   
    for(int i=0;i<NUM_AUDIO_BINS;++i)
    {
      PWI[i].startRow=random(71);
      PWI[i].startCol=random(NUM_COLUMNS);
      PWI[i].startAngleIndex=(random(8)*4)%32;
      PWI[i].dir=random(2);
      PWI[i].radius=4; //bins[i]/125+1;
      PWI[i].trailLength=8; //random(8)+1;
      for(int j=0;j<PWI[i].trailLength;++j)
      {
        PWI[i].lastRows[j]=-1;
        PWI[i].lastCols[j]=-1;
      }
    }
  }

  angleCt=(angleCt+2)%32;
  //int colorOffset=((pwCtr*8)%256);
  int colorOffset=0;

  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {
    color.hue=bin*30+colorOffset;

    // CW or CCW circles
    // int angleCt2;
    // if(PWI[bin].dir==0)
    //   angleCt2=32-angleCt;
    // else
    //   angleCt2=angleCt;

    // calculate row and column
    //int radius=PWI[bin].radius;
    if(bins[bin]>semiGlobalThreshold) //was 125
    {
      row=((PWI[bin].startRow+pwCtr)%NUM_LEDS_PER_COLUMN);
      col=round(sinTable11p25Deg[row%32]*PWI[bin].radius+PWI[bin].startCol);
      col=round(sinTable11p25Deg[(PWI[bin].startAngleIndex+angleCt)%32]*PWI[bin].radius+PWI[bin].startCol);      
    }
    else
    {
      row=-1;
      col=-1;
    }

    // shift arrays containing previous rows and cols
    for(int i=0;i<PWI[bin].trailLength-1;++i)
    {
      PWI[bin].lastRows[i]=PWI[bin].lastRows[i+1];
      PWI[bin].lastCols[i]=PWI[bin].lastCols[i+1];            
    }
    // put new row and col values in arrays
    PWI[bin].lastRows[PWI[bin].trailLength-1]=row;
    PWI[bin].lastCols[PWI[bin].trailLength-1]=col; 

    int validsCt=0;
    for(int i=0;i<PWI[bin].trailLength;++i)
    {
      if(checkRowAndCol(PWI[bin].lastRows[i],PWI[bin].lastCols[i]))
        ++validsCt;
    }

    if(validsCt>6)
    {
      // update led array
      for(int i=0;i<PWI[bin].trailLength;++i)
      {
        if(checkRowAndCol(PWI[bin].lastRows[i],PWI[bin].lastCols[i]))
          leds[MapLeds(PWI[bin].lastRows[i],PWI[bin].lastCols[i])]=CHSV(color);  
      } 
    }
  }
  ++pwCtr;
}
//******************************************************************************
void drawTimeDomain(int* bins)
{
  //Serial.println("timedomain");
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};

  int max  = bins[0];
  int max_index=0;
  for (int i = 0; i < 8; i++)
  {
    if (bins[i] > max) 
    {
        max = bins[i];
        max_index = i;
    }
  }
  
  // Draws line from center column to point
  if(max>semiGlobalThreshold)
  {
    for(int i=0;i<FFT_N;i=i+2)
    {
      int row=i/2;
      int col=AnalogVals1[i]/32+16;
      Serial.println(col);
      color={(uint8_t)((max_index*30)%256),DEFAULT_SAT,DEFAULT_LUM};
      drawLine(row,row,16,col,color);
    }
  }  
}
//******************************************************************************
void drawPersistentSpinningCircles(int* bins)
{
  static int centerRows[NUM_AUDIO_BINS];
  static int centerCols[NUM_AUDIO_BINS];
  static int radiuss[NUM_AUDIO_BINS];
  static int fnCtr=0;
  int bin;
  int dir=0;

  if(persistentSpinningCircles1ResetFlag==1 || fnCtr%16==0)
  {
    for(bin=0;bin<NUM_AUDIO_BINS;++bin)
    {
      centerRows[bin]=random(NUM_LEDS_PER_COLUMN);
      centerCols[bin]=random(NUM_COLUMNS);
      radiuss[bin]=bins[bin]/125;
    } 
    persistentSpinningCircles1ResetFlag=0;
    fnCtr=0;   
  }
  
  for(bin=0;bin<NUM_AUDIO_BINS;++bin)
  {
    if(bins[bin]>semiGlobalThreshold)
    {
        drawPersistentSpinningCircle(bin, bins[bin],dir,centerRows[bin],centerCols[bin],radiuss[bin],fnCtr);
    }
  }

  ++fnCtr;
}
//******************************************************************************
void drawPersistentSpinningCircle(int bin, int binval, int dir, int centerRow, int centerCol,int radiusSize,int fnCtr)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  int circleCt;
  int radiusSizeRows;

  if(radiusSize<1) return;

  if((fnCtr%16)<8)
    circleCt=fnCtr%8;
  else
    circleCt=7-(fnCtr%8);

  color.hue=bin*30+circleCt*30;

  // adjust radius for rows to account for LED layout aspect ratio and circle rotation affect
  radiusSizeRows=radiusSize +radiusSize*.5-circleCt*radiusSize*1.5/8.0;

  for(int tableIndex=0;tableIndex<32;tableIndex=tableIndex+1)
  {
    int row=round(sinTable11p25Deg[tableIndex]*(radiusSizeRows))+centerRow;
    int col=round(cosTable11p25Deg[tableIndex]*(radiusSize))+centerCol;
    if(checkRowAndCol(row,col))
        leds[MapLeds(row,col)]=color;
  }
}
//******************************************************************************
void drawPersistentBreathingCircles(int* bins)
{
  static int centerRows[NUM_AUDIO_BINS];
  static int centerCols[NUM_AUDIO_BINS];
  static int fnCtr=0;
  int bin;
  int dir=0;

  if(persistentSpinningCircles2ResetFlag==1 || fnCtr%32==0)
  {
    for(bin=0;bin<NUM_AUDIO_BINS;++bin)
    {
      centerRows[bin]=random(NUM_LEDS_PER_COLUMN);
      centerCols[bin]=random(NUM_COLUMNS);
    } 
    persistentSpinningCircles2ResetFlag=0;
    fnCtr=0;   
  }
  
  //for(bin=0;bin<NUM_AUDIO_BINS;++bin)
  for(bin=0;bin<1;++bin)
  {
    if(bins[bin]>semiGlobalThreshold)
    {
      drawPersistentBreathingCircle(bin, bins[bin],dir,centerRows[bin],centerCols[bin],bins[bin]/125,fnCtr);
    }
  }

  ++fnCtr;
}

//******************************************************************************
void drawPersistentBreathingCircle(int bin, int binval, int dir, int centerRow, int centerCol,int radiusSize,int fnCtr)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  int circleCt;
  int radiusSizeRows;

  if(radiusSize<1) return;

  if((fnCtr%16)<8)
    circleCt=fnCtr%8;
  else
    circleCt=7-(fnCtr%8);

  color.hue=bin*30+circleCt*30;

  // adjust radius for rows to account for LED layout aspect ratio and circle rotation affect
  radiusSizeRows=radiusSize*1.25;

  for(int tableIndex=0;tableIndex<32;tableIndex=tableIndex+1)
  {
    int row=round(sinTable11p25Deg[tableIndex]*(radiusSizeRows))+centerRow;
    int col=round(cosTable11p25Deg[tableIndex]*(radiusSize))+centerCol;
    if(checkRowAndCol(row,col))
        leds[MapLeds(row,col)]=color;
  }
}
// //******************************************************************************
void drawHexagons(int* bins)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};  
  hexagonVerticeInfo VIBase;

  int a=4;
  for(int bin=0;bin<8;++bin)
  {
    if(bins[bin]>500)
    {
      a=bins[bin]/250;
      if(a<4)a=4;

      setupHexagonVerticeInfoBase(&VIBase,a);
      
      color.hue=bin*30;
      for(int i=0;i<6;++i)
      {
        int newrow,newcol;
        mapHexagon(a, random(ceil(NUM_LEDS_PER_COLUMN/a/3)+2), random( ceil(NUM_COLUMNS/a)+3)-1, &newrow, &newcol);
        for(int i=0;i<6;++i)
        {
          drawLine(VIBase.verticeRows[i%6]+newrow,VIBase.verticeRows[(i+1)%6]+newrow,VIBase.verticeCols[i%6]+newcol,VIBase.verticeCols[(i+1)%6]+newcol,color);
        }
      }
    }      
  }
}
//******************************************************************************
void drawPersistentHexagons(int* bins)
{
  static int centerRows[8][6];
  static int centerCols[8][6];
  static int size[8];
  static int fnCtr=0;
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};  
  hexagonVerticeInfo VIBase;

  if(persistentHexagonsResetFlag==1 || fnCtr%32==0)
  {
    for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
    {

      size[bin]=bins[bin]/250;
      if(size[bin]<4) size[bin]=4;

      for(int entity=0;entity<6;++entity)
      {
        centerRows[bin][entity]=random(ceil(NUM_LEDS_PER_COLUMN/size[bin]/3)+2);
        centerCols[bin][entity]=random( ceil(NUM_COLUMNS/size[bin])+3)-1;        
      }

    } 
    persistentHexagonsResetFlag=0;
    fnCtr=0;   
  }

  for(int bin=0;bin<8;++bin)
  {
    if(bins[bin]>500)
    {
      int tempSize=bins[bin]/250;
      if(tempSize<2)tempSize=2;
      setupHexagonVerticeInfoBase(&VIBase,tempSize);      
      
      color.hue=bins[bin]/200*30+random(8)*30;
      for(int entity=0;entity<6;++entity)
      {
        int newrow,newcol;
        mapHexagon(size[bin], centerRows[bin][entity], centerCols[bin][entity], &newrow, &newcol);
        for(int i=0;i<6;++i)
        {
          drawLine(VIBase.verticeRows[i%6]+newrow,VIBase.verticeRows[(i+1)%6]+newrow,VIBase.verticeCols[i%6]+newcol,VIBase.verticeCols[(i+1)%6]+newcol,color);
        }
      }
    }      
  }
  
  ++fnCtr;
}
//******************************************************************************
void drawPersistentSpinningHexagons(int* bins)
{
  static int centerRows[8];
  static int centerCols[8];
  static int fnCtr=0;
  static int dir=random(2);


  static struct hexagonVerticeInfo VIBase=
  {
    {0,0,0,0,0,0},{0,0,0,0,0,0}, 
  };

  //CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};

  if(persistentSpinningHexagonsResetFlag==1 || fnCtr%48==0)
  {  
    dir=random(2);
 
    for(int i=0;i<8;++i)
    {
      centerRows[i]=random(NUM_LEDS_PER_COLUMN);
      centerCols[i]=random(NUM_COLUMNS);
      // centerRows[i]=35;
      // centerCols[i]=16;      
    } 
    persistentSpinningHexagonsResetFlag=0;
    fnCtr=0;   
  }
 // delay(100);

  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  //for(int bin=0;bin<1;++bin)  
  {

    if(bins[bin]>500)
    {

      //int dir=random(2);
      int tempSize=bins[bin]/250;
      //if(tempSize<2)tempSize=2;
      // tempSize=12;
      setupHexagonVerticeInfoBase(&VIBase,tempSize); 
      drawPersistentSpinningHexagon(bin, bins[bin],dir,centerRows[bin],centerCols[bin],VIBase,fnCtr);
    }
  }
  ++fnCtr;
}
//*******************************************************************************
void drawPersistentSpinningHexagon(int bin, int binval, int dir, int centerRow, int centerCol,struct hexagonVerticeInfo VIBase,int fnCtr)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  static struct hexagonVerticeInfo VINew;

  // To rotate a point (x, y) by an angle θ around the origin (0, 0), you'll use trigonometry to calculate the new coordinates (x', y').
  //     x' = x * cos(θ) - y * sin(θ)
  //     y' = x * sin(θ) + y * cos(θ)
  
  color.hue=bin*30+binval/250*10;

  int trigTableIndex;
  if(dir==0)
    trigTableIndex=(24-1-((fnCtr/2)%24));
  else
    trigTableIndex=((fnCtr/2)%24);
  
  VINew.verticeRows[0]=round((VIBase.verticeCols[0])*sinTable15Deg[trigTableIndex]+(VIBase.verticeRows[0])*cosTable15Deg[trigTableIndex]);
  VINew.verticeRows[1]=round((VIBase.verticeCols[1])*sinTable15Deg[trigTableIndex]+(VIBase.verticeRows[1])*cosTable15Deg[trigTableIndex]);
  VINew.verticeRows[2]=round((VIBase.verticeCols[2])*sinTable15Deg[trigTableIndex]+(VIBase.verticeRows[2])*cosTable15Deg[trigTableIndex]);
  VINew.verticeRows[3]=round((VIBase.verticeCols[3])*sinTable15Deg[trigTableIndex]+(VIBase.verticeRows[3])*cosTable15Deg[trigTableIndex]);  
  VINew.verticeRows[4]=round((VIBase.verticeCols[4])*sinTable15Deg[trigTableIndex]+(VIBase.verticeRows[4])*cosTable15Deg[trigTableIndex]);  
  VINew.verticeRows[5]=round((VIBase.verticeCols[5])*sinTable15Deg[trigTableIndex]+(VIBase.verticeRows[5])*cosTable15Deg[trigTableIndex]);  


  VINew.verticeCols[0]=round((VIBase.verticeCols[0])*cosTable15Deg[trigTableIndex]-(VIBase.verticeRows[0])*sinTable15Deg[trigTableIndex]);
  VINew.verticeCols[1]=round((VIBase.verticeCols[1])*cosTable15Deg[trigTableIndex]-(VIBase.verticeRows[1])*sinTable15Deg[trigTableIndex]);
  VINew.verticeCols[2]=round((VIBase.verticeCols[2])*cosTable15Deg[trigTableIndex]-(VIBase.verticeRows[2])*sinTable15Deg[trigTableIndex]);
  VINew.verticeCols[3]=round((VIBase.verticeCols[3])*cosTable15Deg[trigTableIndex]-(VIBase.verticeRows[3])*sinTable15Deg[trigTableIndex]);  
  VINew.verticeCols[4]=round((VIBase.verticeCols[4])*cosTable15Deg[trigTableIndex]-(VIBase.verticeRows[4])*sinTable15Deg[trigTableIndex]);                                                             
  VINew.verticeCols[5]=round((VIBase.verticeCols[5])*cosTable15Deg[trigTableIndex]-(VIBase.verticeRows[5])*sinTable15Deg[trigTableIndex]); 


  drawLine(centerRow+VINew.verticeRows[0],centerRow+VINew.verticeRows[1],centerCol+VINew.verticeCols[0],centerCol+VINew.verticeCols[1],color);
  drawLine(centerRow+VINew.verticeRows[1],centerRow+VINew.verticeRows[2],centerCol+VINew.verticeCols[1],centerCol+VINew.verticeCols[2],color);  
  drawLine(centerRow+VINew.verticeRows[2],centerRow+VINew.verticeRows[3],centerCol+VINew.verticeCols[2],centerCol+VINew.verticeCols[3],color);
  drawLine(centerRow+VINew.verticeRows[3],centerRow+VINew.verticeRows[4],centerCol+VINew.verticeCols[3],centerCol+VINew.verticeCols[4],color);
  drawLine(centerRow+VINew.verticeRows[4],centerRow+VINew.verticeRows[5],centerCol+VINew.verticeCols[4],centerCol+VINew.verticeCols[5],color);
  drawLine(centerRow+VINew.verticeRows[5],centerRow+VINew.verticeRows[0],centerCol+VINew.verticeCols[5],centerCol+VINew.verticeCols[0],color);    
}
//******************************************************************************
void setupHexagonVerticeInfoBase(hexagonVerticeInfo* VIBase, int a)
{
  float sinConstant=sqrt(3)/2;

  VIBase->verticeRows[0]=round(a);
  VIBase->verticeRows[1]=round(a/2);
  VIBase->verticeRows[2]=round(-a/2);
  VIBase->verticeRows[3]=round(-a);
  VIBase->verticeRows[4]=round(-a/2);
  VIBase->verticeRows[5]=round(a/2);  
  
  VIBase->verticeCols[0]=0;      
  VIBase->verticeCols[1]=round(sinConstant*a);
  VIBase->verticeCols[2]=round(sinConstant*a);
  VIBase->verticeCols[3]=0;
  VIBase->verticeCols[4]=round(-sinConstant*a);
  VIBase->verticeCols[5]=round(-sinConstant*a);
}
//******************************************************************************
void mapHexagon(int a, int hexrow, int hexcol, int* prow, int* pcol)
{
 if(hexcol%2==0)
  {
    *prow=hexrow*a*3+a;
    *pcol=hexcol*a+a-hexcol-1; 
  }
  else
  {
    *prow=hexrow*a*3-a/2;
    *pcol=hexcol*a+a-hexcol-1;
  }
}
//*******************************************************************************
void drawDiagonals(int* bins)
{
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  int startRow;
  int stopRow;
  int startCol;
  int stopCol;

  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {
    if(bins[bin]>semiGlobalThreshold)
    {
      int numColsMinusOne=NUM_COLUMNS-1;
      color.hue=bin*30;
      startRow=-numColsMinusOne+random(NUM_LEDS_PER_COLUMN+NUM_COLUMNS);
      stopRow=startRow+numColsMinusOne;
      startCol=0;
      stopCol=numColsMinusOne;
      drawLine(startRow,stopRow,startCol,stopCol,color);
      int randRow=random(NUM_COLUMNS);
      stopRow=startRow+randRow;      
      startRow=startRow+numColsMinusOne+randRow;
      drawLine(startRow,stopRow,startCol,stopCol,color);
    }
  }
}
//*******************************************************************************
// void drawDiagonalsVariableLength(int* bins)
// {
//   CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
//   int startRow;
//   int stopRow;
//   int startCol;
//   int stopCol;

//   int centerRow=32;
//   int centerCol=16;

//   //slope-intercept form (y = mx + b), point-slope form (y - y1 = m(x - x1)), or standard form (Ax + By + C = 0)
//   //m=1: y-y1=x-x1: -y1=x-x1-y: y1=-x+x1+y: y1=x1+y-x
//   for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
//   {
//     if(bins[bin]>semiGlobalThreshold)
//     {
//       centerRow=random(NUM_LEDS_PER_COLUMN);
//       centerCol=random(NUM_COLUMNS);
//       int numColsMinusOne=NUM_COLUMNS-1;
//       color.hue=bin*30;
//       int binDivBy100=bins[bin]/100;

//       //int startRow=-numColsMinusOne+random(NUM_LEDS_PER_COLUMN+NUM_COLUMNS);
//       int startRow=centerRow-binDivBy100;
//       //int startCol=NUM_COLUMNS-1-random(NUM_COLUMNS*2);
//       int startCol=centerCol-binDivBy100;
//       int stopRow=centerRow+binDivBy100;
//       int stopCol=centerCol+binDivBy100;   
//       drawLine(startRow,stopRow,startCol,stopCol,color);

//       startRow=centerRow+binDivBy100;
//       startCol=startCol;
//       stopRow=centerRow-binDivBy100;   
//       stopCol=stopCol;  
//       drawLine(startRow,stopRow,startCol,stopCol,color);
//     }
//   }
// }
//******************************************************************************
void drawPersistentPulsatingSpinningDiagonals(int* bins)
{
  static int centerRows[8];
  static int centerCols[8];
  static int dir[8];
  static int fnCtr=0;
 
  if(persistentPulsatingSpinningDiagonalsResetFlag==1 || fnCtr%32==0)
  {  
     for(int i=0;i<NUM_AUDIO_BINS;++i)
    {
      centerRows[i]=random(NUM_LEDS_PER_COLUMN);
      centerCols[i]=random(NUM_COLUMNS);    
      dir[i]=random(2);
    } 
    persistentPulsatingSpinningDiagonalsResetFlag=0;
    fnCtr=0;   
  }
  for(int bin=0;bin<NUM_AUDIO_BINS;++bin)
  {
    if(bins[bin]>semiGlobalThreshold)
    {
      drawPersistentPulsatingSpinningDiagonal(bin, bins[bin], dir[bin],centerRows[bin], centerCols[bin],fnCtr);
    }
  }

  ++fnCtr;
}
//******************************************************************************
// void drawPersistentPulsatingDiagonal(int bin, int binval, int dir, int centerRow, int centerCol,int fnCtr)
// {
//   CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
//   int numColsMinusOne=NUM_COLUMNS-1;
//   color.hue=bin*30;
//   int binDivBy100=binval/100;
  
//   int startRow=centerRow-binDivBy100;
//   int startCol=centerCol-binDivBy100;
//   int stopRow=centerRow+binDivBy100;
//   int stopCol=centerCol+binDivBy100;   
//   drawLine(startRow,stopRow,startCol,stopCol,color);

//   startRow=centerRow+binDivBy100;
//   startCol=startCol;
//   stopRow=centerRow-binDivBy100;   
//   stopCol=stopCol;  
//   drawLine(startRow,stopRow,startCol,stopCol,color);
// }
//******************************************************************************
void drawPersistentPulsatingSpinningDiagonal(int bin, int binval, int dir, int centerRow, int centerCol,int fnCtr)
{
  struct VerticeInfo
  {
    int verticeRows[4];
    int verticeCols[4];
  };

  static struct VerticeInfo VIBase=
  {
    {0,0,0,0},{0,0,0,0}, 
  };

  static struct VerticeInfo VINew;
  CHSV color={0,DEFAULT_SAT,DEFAULT_LUM};
  //int numColsMinusOne=NUM_COLUMNS-1;
  color.hue=bin*30+fnCtr*4;
  int binDivBy100=binval/100;

  VIBase.verticeRows[0]=-binDivBy100;
  VIBase.verticeCols[0]=-binDivBy100;
  VIBase.verticeRows[1]=binDivBy100;
  VIBase.verticeCols[1]=binDivBy100; 
  VIBase.verticeRows[2]=binDivBy100;
  VIBase.verticeCols[2]=VIBase.verticeCols[0];
  VIBase.verticeRows[3]=-binDivBy100;   
  VIBase.verticeCols[3]=VIBase.verticeCols[1];
  
  int trigTableIndex;
  if(dir==0)
    trigTableIndex=(16-(fnCtr%16))*2;
  else
    trigTableIndex=(fnCtr%16)*2;
  
  VINew.verticeRows[0]=round((VIBase.verticeCols[0])*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[0])*cosTable11p25Deg[trigTableIndex]);
  VINew.verticeRows[1]=round((VIBase.verticeCols[1])*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[1])*cosTable11p25Deg[trigTableIndex]);
  VINew.verticeRows[2]=round((VIBase.verticeCols[2])*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[2])*cosTable11p25Deg[trigTableIndex]);
  VINew.verticeRows[3]=round((VIBase.verticeCols[3])*sinTable11p25Deg[trigTableIndex]+(VIBase.verticeRows[3])*cosTable11p25Deg[trigTableIndex]);  

  VINew.verticeCols[0]=round((VIBase.verticeCols[0])*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[0])*sinTable11p25Deg[trigTableIndex]);
  VINew.verticeCols[1]=round((VIBase.verticeCols[1])*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[1])*sinTable11p25Deg[trigTableIndex]);
  VINew.verticeCols[2]=round((VIBase.verticeCols[2])*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[2])*sinTable11p25Deg[trigTableIndex]);
  VINew.verticeCols[3]=round((VIBase.verticeCols[3])*cosTable11p25Deg[trigTableIndex]-(VIBase.verticeRows[3])*sinTable11p25Deg[trigTableIndex]); 

  VINew.verticeRows[0]+=centerRow;
  VINew.verticeCols[0]+=centerCol;
  VINew.verticeRows[1]+=centerRow;
  VINew.verticeCols[1]+=centerCol;
  VINew.verticeRows[2]+=centerRow;
  VINew.verticeCols[2]+=centerCol;
  VINew.verticeRows[3]+=centerRow;
  VINew.verticeCols[3]+=centerCol;

  drawLine(VINew.verticeRows[0],  VINew.verticeRows[1],VINew.verticeCols[0],VINew.verticeCols[1],color);
  drawLine(VINew.verticeRows[2],  VINew.verticeRows[3],VINew.verticeCols[2],VINew.verticeCols[3],color);
}
// //******************************************************************************
// int checkTable(int* bins)
// {
//   for(int i=0;i<32;++i)
//   {
//     if (sinTable11p25Deg[i]!=float(sin(i*11.25/(180/PI))))
//     {
//       Serial.println("Error");
//       return(0);
//     }
//   }
//   return(1);
// }

//******************************************************************************
void fillTrigTables(void)
{
    // Fill sin and cos tables
  for(int i=0;i<32;++i)
  {
    sinTable11p25Deg[i]=sin(i*11.25/(180/PI));
    cosTable11p25Deg[i]=cos(i*11.25/(180/PI));
  }
  for(int i=0;i<24;++i)
  {
    sinTable15Deg[i]=sin(i*15.0/(180/PI));
    cosTable15Deg[i]=cos(i*15.0/(180/PI));
  }

}
//******************************************************************************
int checkRowAndCol(int row, int col)
{
  if(!(row>=NUM_LEDS_PER_COLUMN || row < 0 || col >= NUM_COLUMNS || col<0 ))
  {
    return(1);
  }
  return(0);
}
