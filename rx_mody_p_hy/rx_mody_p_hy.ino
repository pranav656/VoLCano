//By Pranav Mani
//Logs:
//Updated PHY layer implementation, removed sync and unused functions
//Removed unnecesary class based implementation
//Updated updateBitQueue() fn, Added debugging options, phy_tx() working from main loop at 10Hz update rate, not working at higher rates,need update in architecture
//Unable to detect Rx pulses. Must be fixed either in code architecture by decreasing Freq
//added faster adc takes about 15us to execute, added timer synchronization(can operate at two freq. 4000Hz and 1000Hz), bytes are decoded with a very high error rate
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#include<QueueArray.h>
#include<math.h>
#define NODE_ADDR 0x01 // adress of current Node
#define LED_TX_PIN A1
#define LED_RX_PIN A0
#define DETECTION_THRESHOLD 0
#define PHY_IDLE_STATE 0
#define RX_LOW_THRESHOLD -1
#define RX_HIGH_THRESHOLD 150
//#define FAST_ADC 1
#define FAST_ADC_2
#define PHY_TX 5
#define PHY_RX 6
//#define TIMER1COUNT 61535 //Freq configuration for 500Hz
#define TIMER1COUNT 65500    //65535-16Mhz/8/Freq currently configured for 2Khz
#define BYTE_LEN 8
//#define TEST_TX
#define CLOCK_FREQ 16000000
#define PRESCALAR_TIMER 8
int timerFreq=2000;
bool syncTimer=true;
int16_t timerControlVariable= (65535) - CLOCK_FREQ/(PRESCALAR_TIMER * timerFreq);
//idle state-ON,OFF,ON
  //bool beginTransmitcheck=false;
bool transmitBusy; //bit used to indicate whether the phy layer is currently transmitting
int8_t transmitCount; //bit that keeps count of sequence of true/false bits that are transmotted
int16_t currentAnalogRx,prevAnalogRx; //variable used to store analogVoltage on the Rx Pins
bool bitTransmitcheck;
bool txStatus,currentBit,nextBit; 
int data_to_be_rec;
bool fallingEdgeDet,risingEdgeDet, currentRx, prevRx, risingEdge, fallingEdge, startSlot;
void phy_tx(uint8_t* message, unsigned char len); //function called to transmit
int phy_rx(uint8_t* rx_buffer); //function called to copy the data onto the rx_buffer, returns -1 if no data is available
bool phySense(); //function used to sense the physical layer
void beginTransmit(); //starts actual transmission of message
void formRxByte(); //function used to form the byte
bool phyDetect(); // function used to detect the data on the analog port
bool phySense();//returns true if node is idle
bool prevData=true;
int byte_pos=0;
void updateBitQueue(); //function used to convert the bytes to bits
uint8_t rx_buffer[263]; //message used to store the received data
uint8_t rx_buff[263]; //test variable used to cpy received data
uint8_t tx_message;
unsigned long TimerFreq;
unsigned char currentLedState,currentPhyState;
void runPhyFSM(uint8_t currentState);  
QueueArray <uint8_t> phy_tx_message_queue; //tx byte queue
QueueArray <uint8_t> phy_rx_message_queue; //rx byte queue
QueueArray <bool> phy_tx_bit_queue; //tx bit queue
QueueArray <bool> phy_rx_bit_queue; // rx bit queue
bool LedState=true;
long prevTime;
bool data_available=false;
bool wait_true= true;
bool wait_false=true;

void ADC_setup(){
  ADCSRA =  bit (ADEN);                      // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX  =  bit (REFS0) | bit (REFS1);    // internal 1.1v reference
}

void ADC_start_conversion(int adc_pin){
  ADMUX &= ~(0x07) ; //clearing enabled channels
  ADMUX  |= (adc_pin & 0x07) ;    // AVcc and select input port
  bitSet (ADCSRA, ADSC) ;
}

int ADC_read_conversion(){
 while(bit_is_set(ADCSRA, ADSC));
 return ADC ;
}

ISR(TIMER1_OVF_vect)
{ 
  //ISR called every time timer register overflows
  phyDetect(); //function used to check whether any data is available

  
  
//  Serial.print("MQC=");
//  Serial.println(phy_tx_message_queue.count());
//  Serial.print("BQC=");
//  Serial.println(phy_tx_bit_queue.count());
  if(phy_tx_bit_queue.count() ){ //checks whether any data is available in the buffer. If yes, then transmits it
    currentPhyState=PHY_TX;
  }
  else {
    currentPhyState=PHY_IDLE_STATE;
  }
#ifdef FAST_ADC
//  ADC_start_conversion(0);
#endif
//  runPhyFSM(currentPhyState);  
  TCNT1 = timerControlVariable;
}

void formRxByte()
{
  //Function that is used to form the Byte from the received bits
  unsigned char ubyte=0x00;  
    for(int j=0;j<8;j++){
      if(phy_rx_bit_queue.dequeue()){
             ubyte = ubyte|(0x01<<j);
      }
      else{
        ubyte=ubyte;
      }
    }
    decodeFrame(ubyte);
//    phy_rx_message_queue.enqueue(byte);
}


bool wait;
char bitCount=0;
bool pState=false;
unsigned char g_ubyte;
void decodeFrame(uint8_t ubyte){
//  pState=!pState;
//  digitalWrite(13,pState);
  g_ubyte=ubyte;
//  Serial.println(ubyte,HEX);
 
  // function used to decode each byte and make a frame
  if(byte_pos==0){
    if(ubyte==0xFF){
//        pState=!pState;
//        digitalWrite(12,pState);
        byte_pos++;
        rx_buffer[0]=ubyte;
//        syncTimer=true;
        data_available=false;      
    }
  }

  else if(byte_pos>=1 && byte_pos<=3){
    byte_pos++;
    rx_buffer[byte_pos]=ubyte;
  }

  else if(byte_pos==4){
    rx_buffer[byte_pos]=ubyte;
    data_to_be_rec=ubyte;
    byte_pos++;
  }

  else if(byte_pos>4 && byte_pos<4+data_to_be_rec){
    rx_buffer[byte_pos]=ubyte;
    byte_pos++;
    data_available=true;
  }

  else if(byte_pos == (4+data_to_be_rec)){
    byte_pos=0;
  }
}

bool phySense()
{
  if(currentPhyState==PHY_IDLE_STATE){
    return true; //returns true if idle
  }
  else{
    return false;
  }
}
int btCount=0;
void beginTransmit()
{
//  Serial.println(btCount);
//   Serial.println("bT");
//    if(btCount==0){
//      btCount++;
//      if(phy_tx_bit_queue.count() >= 2 && (!transmitBusy)){
//      currentBit=phy_tx_bit_queue.dequeue();
//      nextBit=phy_tx_bit_queue.dequeue();}
//    }
//
//    else if(btCount==1){
//      btCount++;
//      
//    }
//
//    else if(btCount==2){
//      btCount++;
//      currentBit=nextBit;
//    }
//
//    else if(btCount==3){
//      btCount=0;
//    }
  //actual bit trasmission takes place
    if(!transmitBusy)
    {
      if(phy_tx_bit_queue.count()){
    currentBit=phy_tx_bit_queue.dequeue();}  
    transmitCount=0;
    }
      
    if(currentBit==true)
    {
      if(transmitCount==0)
      {
       transmitBusy=true;
       transmitCount++;
       digitalWrite(LED_TX_PIN,LOW);  //2-ppm encoding used to encode and send 0s and 1s
       Serial.println("LOW t");      
      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_TX_PIN,HIGH); //2-ppm encoding used to encode and send 0s and 1s
        transmitBusy=false;
        transmitCount=0;
        Serial.println("HIGH");
      }
    }

    else if(currentBit==false)
    {
      if(transmitCount==0)
      {
       transmitCount++;
       transmitBusy=true;
       digitalWrite(LED_TX_PIN,HIGH); //2-ppm encoding used to encode and send 0s and 1s
       Serial.println("HIGH f");
      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_TX_PIN,LOW); //2-ppm encoding used to encode and send 0s and 1s
        transmitBusy=false;
        transmitCount=0;
        Serial.println("LOW");
      }
    }

}

void updateBitQueue()
{
  //Function that updates the bit queue that needs to be sent
  txStatus=true;
  if(phy_tx_message_queue.count()){
  tx_message=phy_tx_message_queue.dequeue();
//  Serial.println(tx_message,HEX);
  for(int i=0;i<BYTE_LEN; i++)
  {
    //finding the information in each bit
    if(((tx_message>>i & 0x01)) == 0x01)
    {
      phy_tx_bit_queue.enqueue(true);
//      printBool(true);
    }
    else if(((tx_message>>i & 0x01)) == 0x00)
    {
      phy_tx_bit_queue.enqueue(false);
//      printBool(false);
    } 
  }}  
}

void phy_tx(uint8_t *message, uint8_t len)
{
//  phy_tx_message_queue.enqueue(0xFF);
  for(int i=0; i<len; i++)
  {
//    Serial.println(message[i],HEX); //uncomment to debug
    phy_tx_message_queue.enqueue(message[i]); //queues the message onto the tx queue
//    Serial.println(phy_tx_message_queue.count());
    updateBitQueue();
//    Serial.println("MQC="); 
//    Serial.println(phy_tx_message_queue.count()); //uncomment to debug
//    Serial.print("CQC=");
//    Serial.println(phy_tx_bit_queue.count());    
    }
    currentPhyState=PHY_TX;
  //does not transmit of in idle state..waits for idle state to finish transmittinfg
//  if(currentPhyState!= PHY_RX)
//  {
//      currentPhyState=PHY_TX;
//    Serial.println("TX"); //uncomment to debug
//     updateBitQueue();
    
//  }
}

int phy_rx(uint8_t* rx_data)
{
  //copies the data in the rx_buffer to whatever variable you specify to
  if(data_available){
    if(rx_buffer[2] == NODE_ADDR){
     for(int i=0;i<(rx_buffer[4]+3);i++){
      rx_data[i]=rx_buffer[i];
      data_available=false;
      return rx_buffer[4];
     }
  }}

  else{
    return -1;
  }
}

//bool idleState=false;
int idleVar=0;
void runPhyFSM(uint8_t currentState) 
{
  //state machine for the PHY layer
//  Serial.println("FSM");
  switch(currentState)
  {
  case PHY_IDLE_STATE:
//  Serial.println("IDLE");  //uncomment to debug
  transmitIdle();  
  break;
  
  case PHY_TX:
//  Serial.println("TD");  //uncomment to debug
  beginTransmit(); //actual bit-by-bit transmission takes place
  break;
  
  }
}

void transmitIdle()
{
  //transmits zero during idle
  digitalWrite(LED_TX_PIN,LOW);
}

void initializeTimerLed()
{
  //function that initializes timer and LED pin
  noInterrupts();   
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = TIMER1COUNT;
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << TOIE1);
  TCNT1 = timerControlVariable;
  interrupts();
  pinMode(LED_TX_PIN,OUTPUT);
  pinMode(LED_RX_PIN,INPUT);
  
}

bool bit_buffer[2];
bool byte_buff[8];
unsigned char byte_rec;
unsigned char bit_buff_count=0;
//unsigned char bitCount=0;
unsigned char tmpByte;
void decodeBitBuff()
{
//  Serial.println("db");
  if(bit_buffer[0] == false && bit_buffer[1] ==true )
  {
    addBitData(true);
//  pState=!pState;
//  digitalWrite(13,pState);
//    digitalWrite(13,HIGH);
//    Serial.println("t");
//    phy_rx_bit_queue.enqueue(true);
//    digitalWrite(12,HIGH);
    
  }

  else if(bit_buffer[0] == true && bit_buffer[1] == false){
//    Serial.println("f");
//    phy_rx_bit_queue.enqueue(false);
//    digitalWrite(12,LOW);
//  digitalWrite(13,LOW);
  addBitData(false);
  }
}
uint8_t g_byte_rec;
void formRxData()
{
  byte_rec=0x00;
  for(int i=0; i<8; i++){
    if(byte_buff[i]){
      byte_rec = byte_rec|(0x01<<i);
    }
    else {
      byte_rec = byte_rec;
    }
  }
//  pState=!pState;
//  digitalWrite(12,pState);
  g_byte_rec=byte_rec;
  decodeFrame(byte_rec);
}

void addBitData(bool var){
  byte_buff[bitCount]=var;
  bitCount++;
  if(bitCount==8){
    bitCount=0;
//    syncTimer=true;
    formRxData();
  }
//  Serial.println(bitCount);
//  if(bitCount==8){
//    syncTimer=true;
//    formRxData();
//    bitCount=0;
//  }
//  else{
//    bitCount++; 
//  }
}
unsigned long startAdcTime,endAdcTime;
bool phyDetect()
{
 
//  Serial.println(bitCount);
//  Serial.println(bit_buff_count);
#ifdef FAST_ADC
    currentAnalogRx=ADC_read_conversion();
    ADC_start_conversion(0);
    Serial.println(currentAnalogRx);
#else
//    startAdcTime=micros();
    currentAnalogRx=analogRead(LED_RX_PIN);
//    endAdcTime=micros();
//    Serial.println(endAdcTime - startAdcTime);
    Serial.println(currentAnalogRx);
#endif
//  Serial.println(currentAnalogRx);
  //checks for threshold and rising edge
  if(currentAnalogRx >= RX_LOW_THRESHOLD && (currentAnalogRx <= RX_LOW_THRESHOLD + 20))
  {
//    Serial.println("LOW"); //uncomment to debug
    digitalWrite(12,LOW);
    currentRx=false;
    if(!syncTimer){
    if(bit_buff_count==0){
//      PORTB&= ~_BV(PB4);
//      digitalWrite(12,LOW);
      bit_buffer[bit_buff_count]=false;
      bit_buff_count++; 
    }

    else if(bit_buff_count==1){
//      PORTB&= ~_BV(PB4);
//      digitalWrite(12,LOW);
      bit_buffer[bit_buff_count]=false;
      decodeBitBuff();
      bit_buff_count=0;  
    }}
  }

  //checks if the current State is high
  else if(currentAnalogRx >= RX_HIGH_THRESHOLD )
  {
//    Serial.println("HIGH"); //uncomment to debug
   digitalWrite(12,HIGH);

    currentRx=true;
    if(!syncTimer){
    if(bit_buff_count==0){
//      PORTB |= _BV(PB4);
//      digitalWrite(12,HIGH);
      bit_buffer[bit_buff_count]=true;
      bit_buff_count++;
    }

    else if(bit_buff_count==1){
//      digitalWrite(12,HIGH);
//      PORTB |= _BV(PB4);
      bit_buffer[bit_buff_count]=true;
      decodeBitBuff();
      bit_buff_count=0;
    }}

  }

  if(currentRx==prevRx)
  {
    if(wait_true==true){
      wait_true==false;
//      currentPhyState=PHY_IDLE_STATE; //high for more than two slots, then go to idle
    }

    else if(wait_false==true){
      wait_false==false;
//      currentPhyState=PHY_IDLE_STATE; //low for more than two slots, then go to idle
    }
    if(currentRx==false)
    {
      wait_true=true;
      wait_false=false;
    }

    if(currentRx==true)
    {
      wait_false=true;
      wait_true=false;
    }
  }
  if(syncTimer==true){
  if(currentRx>prevRx) // detects rising edge(true) and prevents misdetection of false
  {
//    Serial.println("RE"); //uncomment to debug
//    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
    risingEdge=true;
    fallingEdge=false;
    syncTimer=false;
    TimerFreq=1000;
    timerControlVariable=(65535) - CLOCK_FREQ/(PRESCALAR_TIMER * TimerFreq);
    addBitData(true);
//    phy_rx_bit_queue.enqueue(true);
    prevData=true;
//    pState=!pState;
//  digitalWrite(13,pState);
//    bitCount++;
  }
  else {
    TimerFreq=4000;
    timerControlVariable= (65535) - CLOCK_FREQ/(PRESCALAR_TIMER * TimerFreq);
  }
  }

//  if(currentRx<prevRx ) // detects falling edge(false) and prevents misdetection of true
//  {
////    Serial.println("FE"); //uncomment to debug
//    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
//    risingEdge=false;
//    fallingEdge=true;
//    phy_rx_bit_queue.enqueue(false);
//    prevData=false;
//    bitCount++;
//  }

//  if(bitCount==8){ //once 8 bits are received, a byte is formed from it.
//    Serial.println("BF"); //uncomment to debug
//    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
//    bitCount=0;
//    formRxByte();
//    unsigned char tmpByte= phy_rx_message_queue.dequeue();
//    Serial.print(tmpByte,d HEX);
//    Serial.println();
//    decodeFrame(tmpByte);
//  }
  prevAnalogRx=currentAnalogRx;
  prevRx=currentRx;
}

unsigned char tx_data[]={0xFF,0x02,0x01,0x00,0x0A,0x01,0x02,0x03,0x04,0x05};
void enableFastADC(){
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);
}
void setup()
{
  initializeTimerLed();
  Serial.begin(115200);
#ifdef FAST_ADC
  ADC_setup();
  ADC_start_conversion(0);
#else
#ifdef FAST_ADC_2
  enableFastADC();
#endif
#endif
currentPhyState=PHY_IDLE_STATE;
  delay(1000);
//  phy_tx(tx_data,0x0A);
//  while(!sync());  
}

unsigned long pTime;


void loop()
{
//  Serial.println(bitCount);
//  Serial.print("by=");
//  Serial.println(g_ubyte,HEX);
 
 // #ifdef TEST_TX
  if(millis() - pTime >= 50){ //Test Case for transmission every one sec
//    phy_tx(tx_data,0x0A);
//    Serial.println(g_byte_rec,HEX);
    pTime=millis();
    
  }
  //#endif

}

void printBool(bool state){
  //Fn used to debug
    switch(state){
      case true:
      Serial.println("true");
      break;

      case false:
      Serial.println("false");
      break;
      
    }
}

