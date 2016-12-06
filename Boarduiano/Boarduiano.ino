#include <SoftwareSerial.h>
SoftwareSerial mySerial(20, 16); // RX, TX
#include "SPI.h" 
#define NUMBER_OF_SHIFT_CHIPS   3
#define DATA_WIDTH   NUMBER_OF_SHIFT_CHIPS * 8
#define speakerPin        6
#define speakerPin2       5
#define key25Pin          7
#define detectModePin     8
#define octave1           2
#define octave2           3
#define octave3           4
#define octavePotPin     A5
#define ploadPin         18  // if these three pins are changed, don't forget to change the port in read_shift_regs()
#define clockPin         17  // if these three pins are changed, don't forget to change the port in read_shift_regs()
#define dataPin           9  // if these three pins are changed, don't forget to change the port in read_shift_regs()
#define dac              10
#define waveFormPot      A0
#define modulationPot    A1

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// frequencies for speaker
float noteFrequency[6][12]={
   // C      C#      D     D#      E      F     F#      G     G#      A     A#      B
  {130.8, 138.6, 146.8, 155.6, 164.8, 174.6, 185.0, 196.0, 207.7, 220.0, 233.1, 246.9},
  {261.6, 277.2, 293.7, 311.1, 329.6, 349.2, 370.0, 392.0, 415.3, 440.0, 466.2, 493.9}, 
  {523.3, 554.4, 587.3, 622.3, 659.3, 698.5, 740.0, 784.0, 830.6, 880.0, 932.3, 987.8}, 
  { 1047,  1109,  1165,  1245,  1319,  1397,  1480,  1568,  1661,  1760,  1865,  1976}, 
  { 2093,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0}};
  
// note values for midi
byte noteValue[6][12]={
  {48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59}, 
  {60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71}, 
  {72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83}, 
  {84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95}, 
  {96,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}};

// shift register stuff
unsigned long key25;
unsigned long pinValues;
unsigned long oldPinValues;

// determine which mode
byte mode = 0;

// midi stuff
boolean noteOn[25]={}; //for midi
byte currentKeyboardVal[25]={};
unsigned long currentMidiNotes;
unsigned long previousMidiNotes;
byte channel = 0;
byte previousModulationVal=0;

// VS1053 stuff
byte instrument = 0;

// speaker stuff
float currentKeyboardFre[25]={};  //these two change when the octave changes
int previousKey=0;
int currentKey=0;

// octave stuff
byte octave=0;
byte previousOctave=0;
int octavePotReading=0;

// used by interrupt
int delta=409; 
int a=0;
byte high;
byte low;
byte waveForm=0; // 0: square; 1: sawtooth; 2: triangle

int intSaw[25]={1273,1201,1133,1069,1009, 953, 899, 849, 801, 755, 713, 673,   636, 600, 566, 534, 504,476,449,424,400,377,356,336,  317};

int intTri[25]={636, 600, 566, 534, 504,476,449,424,400,377,356,336,   636, 600, 566, 534, 504,476,449,424,400,377,356,336,  317};

boolean triangleUp=true;

ISR(TIMER1_COMPA_vect) { 
  if (waveForm==1) {  // sawtooth
    switch (octave) {
      case 0: delta=43;break;
      case 1: delta=87;break;
      case 2: delta=178;break;
    }
    a+=delta;
    a=(a>4095? 0:a);
    OCR1A = intSaw[currentKey];
  }
  else if (waveForm==2) {
    switch (octave) {
      case 0: delta=(currentKey<12? 42:85);break;
      case 1: delta=(currentKey<12? 85:170);break;
      case 2: delta=(currentKey<12? 170:341);break;
    }
    a=a+(triangleUp? delta:-delta);
    if (a>4095) {
        a=4095-delta;
        triangleUp=false;
    }
    else if (a<0) {
        a=delta;
        triangleUp=true;
    }
    OCR1A =intTri[currentKey];
  }
  // Write two bytes data to the dac
  PORTB &= ~(1<<2); //digitalWrite(dac,LOW);
  high = (0b00110000) | (0x0F & (a >> 8)); 
  low = 0b11111111 & a;
  SPI.transfer(high); SPI.transfer(low);
  PORTB |= (1<<2); //digitalWrite(dac,HIGH);
}

unsigned long read_shift_regs() {
    unsigned long bitVal;
    unsigned long bytesVal = 0;
 
    PORTC &= ~(1<<4); //ploadPin: LOW then HIGH
    PORTC |= (1<<4);  
    
    //check the last note first because it represent the biggest value 2^25
    key25 = (PIND & (1<<7)) == (1<<7);  // key25=digitalRead(key25Pin);
    bytesVal |= key25 << 24;
    
    for(int i = 0; i < DATA_WIDTH; i++) {
      bitVal = (PINB & (1<<1)) == (1<<1);  // bitVal = digitalRead(dataPin);
      bytesVal |= (bitVal << ((DATA_WIDTH-1) - i));
      
      PORTC |= (1<<3); //clockPin: HIGH then LOW
      PORTC &= ~(1<<3);
    }
    return(bytesVal);
}

void setup() {
  // start both serials
  Serial.begin(31250);
  mySerial.begin(31250);
  
  // MIDI VS1053 setup
  talkMIDI(0xB0, 0x07, 127); //0xB0 is channel message, set channel volume to near max (127)
  talkMIDI(0xC0, instrument, 0);
  
  // shift registers
  pinMode(ploadPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin, HIGH);
  
  // speaker (using differential drive to cancel out the noise from mcu)
  pinMode(speakerPin, OUTPUT);
  pinMode(speakerPin2, OUTPUT);
  digitalWrite(speakerPin, LOW);
  digitalWrite(speakerPin2, LOW);
  
  // a switch to change mode (PWM, DAC, MIDI)
  pinMode(detectModePin,INPUT);

  // LEDs indicating octaves
  pinMode(octave1,OUTPUT);
  pinMode(octave2,OUTPUT);
  pinMode(octave3,OUTPUT);
  
  // initilize variables
  pinValues = read_shift_regs();
  oldPinValues = pinValues;
  
  // Set up the ADC
  ADCSRA &= ~PS_128; // remove bits set by Arduino library
  ADCSRA |= PS_32;   //choose a prescaler from above: PS_16, PS_32, PS_64 or PS_128
  
  //Start SPI
  SPI.begin(); 
  SPI.setBitOrder(MSBFIRST); 
  pinMode(dac, OUTPUT);
  
  // Set up timer1 interrupt
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 30000;            // compare match register; the current value is arbitrary
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // 8 prescaler 
  
  // starting sound
  int val = analogRead(waveFormPot);
  if (val<165) {
    tone(speakerPin,523.3,100); delay(100);
    tone(speakerPin,659.3,100); delay(100);
    tone(speakerPin,784.0,100); delay(100);
    tone(speakerPin,1047,100); delay(100);
  }
  else {
    delta=511; octave=2;
    TIMSK1 |= (1 << OCIE1A); 
    currentKey=0; delay(100);
    currentKey=4; delay(100);
    currentKey=7; delay(100);
    currentKey=12; delay(100);
    TIMSK1 &= !(1 << OCIE1A);
    previousKey=currentKey;
  }
}

void determineOctave() {
  octavePotReading=analogRead(octavePotPin);
  if (octavePotReading>642) {
    digitalWrite(octave1,HIGH);
    digitalWrite(octave2,LOW);
    digitalWrite(octave3,LOW);
    octave=0;
  }
  else if (octavePotReading<=642 && octavePotReading>382) {
    digitalWrite(octave1,LOW);
    digitalWrite(octave2,HIGH);
    digitalWrite(octave3,LOW);
    octave=1;
  }
  else {
    digitalWrite(octave1,LOW);
    digitalWrite(octave2,LOW);
    digitalWrite(octave3,HIGH);
    octave=2;
  }
}

void currentKeyboard() {
  for (int i = 0; i < 12; i++) {
    currentKeyboardFre[i]=noteFrequency[octave][i];
    currentKeyboardFre[i+12]=noteFrequency[octave+1][i];
    currentKeyboardVal[i]=noteValue[octave][i];
    currentKeyboardVal[i+12]=noteValue[octave+1][i];
  }
  currentKeyboardFre[24]=noteFrequency[octave+2][0];
  currentKeyboardVal[24]=noteValue[octave+2][0];
}

void startWave() {
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
}

void endWave() {
  TIMSK1 &= !(1 << OCIE1A); // disable timer compare interrupt
}

void speakerMode(unsigned long finalRead) {
  for (int i = 0; i <= DATA_WIDTH; i++) {  //from 0 to 24 corresponding to 25 keys
    if ((finalRead >> i) & 1) {
      currentKey=i;
      for (int j=DATA_WIDTH; j>=0; j--) {
        if ((finalRead >> j) & 1) {
          if (j!=previousKey) {
            currentKey=j;
            break;
          }
        }
      }
      if (waveForm==0) tone(speakerPin, currentKeyboardFre[currentKey]);
      else startWave();
      previousKey=currentKey;
      return;
    }
  }
  noTone(speakerPin);
  endWave();
  previousKey=currentKey;
}

void midiMode(unsigned long finalRead, byte mode, int waveFormPotVal) {
  currentMidiNotes=finalRead;
  for (int i = 0; i <= DATA_WIDTH; i++) {
    if (((currentMidiNotes >> i) & 1) && !((previousMidiNotes >> i & 1))) {  
      if (mode==1) {   
        instrument = (waveFormPotVal-21)/7.85;
        talkMIDI(0xC0,instrument,0);
        noteStart(0, currentKeyboardVal[i], 127);
      }
      else {
        channel=waveFormPotVal/64;
        MIDI(144+channel,currentKeyboardVal[i],127);
      }
      noteOn[i]=true;
    }
    else if (!((currentMidiNotes >> i) & 1) && ((previousMidiNotes >> i & 1))) {
      if (mode==1) noteEnd(0, currentKeyboardVal[i], 127);
      else MIDI(128+channel,currentKeyboardVal[i],127);
      noteOn[i]=false;
    }
  }
  previousMidiNotes=currentMidiNotes;
}

unsigned long currentMidiTime=millis();

void MIDI(byte MESSAGE, byte PITCH, byte VELOCITY) {
  Serial.write(MESSAGE);
  Serial.write(PITCH);
  Serial.write(VELOCITY);
  currentMidiTime=millis();
  while (currentMidiTime+6>millis()) {}
}

void turnOffPrevious() {
  for (int i=0;i<25;i++) {
    if (noteOn[i]==true) {
      if (mode==1) noteEnd(0, currentKeyboardVal[i], 127);
      else MIDI(128,currentKeyboardVal[i],127);
      noteOn[i]=false;
    }
  }
}

void loop() {   
  pinValues = read_shift_regs();
  determineOctave();
  if (octave!=previousOctave) {
    turnOffPrevious();  // MIDI: if the change of octave happens during a keypress
    currentKeyboard();  
    previousOctave=octave;
  }
  mode = (PINB & (1<<0)) == (1<<0);  //mode = digitalRead(detectModePin);  
  int modulationVal = (analogRead(modulationPot)/64)*8; 
  if (mode==0 && modulationVal!=0 && modulationVal!=previousModulationVal) {
    MIDI(176+channel,1,modulationVal);  // modulation message
    previousModulationVal=modulationVal;
  }
  if (pinValues != oldPinValues) {  // do six consecutive readings to debounce
    unsigned long finalRead = confirmKeyChange1();
    int waveFormPotVal = analogRead(waveFormPot);
    if (waveFormPotVal<5) waveForm=0;
    else if (waveFormPotVal<13) waveForm=1;
    else if (waveFormPotVal<21) waveForm=2;
    else waveForm=3;
    if (mode==1 && waveForm!=3)  speakerMode(finalRead);
    else midiMode(finalRead, mode, waveFormPotVal);
    oldPinValues = finalRead;
  }
}

unsigned long confirmKeyChange() {
  unsigned long read1 = read_shift_regs();delay(3);
  unsigned long read2 = read_shift_regs();delay(3);
  unsigned long read3 = read_shift_regs();delay(3);
  unsigned long read4 = read_shift_regs();delay(3);
  unsigned long read5 = read_shift_regs();delay(3);
  unsigned long read6 = read_shift_regs();delay(3);
  unsigned long read7 = read_shift_regs();
  while (!(read1==read2 && read2==read3 && read3==read4 && read4==read5 && read5==read6 && read6==read7)) {
    delay(3);
    read1 = read2; read2 = read3; read3 = read4; read4 = read5; read5 = read6; read6 = read7;
    read7 = read_shift_regs();
  }
  return read7;
}

unsigned long confirmKeyChange1() {
  unsigned long read1 = read_shift_regs();delay(3);
  unsigned long read2 = read_shift_regs();delay(3);
  unsigned long read3 = read_shift_regs();delay(3);
  unsigned long read4 = read_shift_regs();delay(3);
  unsigned long read5 = read_shift_regs();
  while (!(read1==read2 && read2==read3 && read3==read4 && read4==read5)) {
    delay(3);
    read1 = read2; read2 = read3; read3 = read4; read4 = read5;
    read5 = read_shift_regs();
  }
  return read5;
}

void noteStart(byte channel, byte note, byte attack_velocity) {
  talkMIDI( (0x90 | channel), note, attack_velocity);
}

//Send a MIDI note-off message.  Like releasing a piano key
void noteEnd(byte channel, byte note, byte release_velocity) {
  talkMIDI( (0x80 | channel), note, release_velocity);
}

//Plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that data values are less than 127
void talkMIDI(byte cmd, byte data1, byte data2) {
  mySerial.write(cmd);
  mySerial.write(data1);
  //Some commands only have one data byte. All cmds less than 0xBn have 3 data bytes 
  //(sort of: http://253.ccarh.org/handout/midiprotocol/)
  if( (cmd & 0xF0) <= 0xB0)
    mySerial.write(data2);
}
