//By Pranav Mani
//Logs:
//Updated PHY layer implementation, removed sync and unused functions
//Removed unnecesary class based implementation


#include<QueueArray.h>
#include<math.h>
#define NODE_ADDR 0x01 // adress of current Node
#define LED_TX_PIN 9
#define LED_RX_PIN 12
#define DETECTION_THRESHOLD 0
#define PHY_IDLE_STATE 0
#define RX_LOW_THRESHOLD 0
#define RX_HIGH_THRESHOLD 900
#define PHY_TX 5
#define PHY_RX 6
#define TIMER1COUNT 64545    //65535-16Mhz/8/Freq currently configured for 2Khz
#define BYTE_LEN 8


//idle state-ON,OFF,ON
  //bool beginTransmitcheck=false;
bool transmitBusy; //bit used to indicate whether the phy layer is currently transmitting
int8_t transmitCount; //bit that keeps count of sequence of true/false bits that are transmotted
int16_t currentAnalogRx,prevAnalogRx; //variable used to store analogVoltage on the Rx Pins
bool bitTransmitcheck;
bool txStatus,currentBit; 
int data_to_be_rec;
bool fallingEdgeDet,risingEdgeDet, currentRx, prevRx, risingEdge, fallingEdge, startSlot;
void phy_tx(uint8_t* message, unsigned char len); //function called to transmit
int phy_rx(uint8_t* rx_buffer); //function called to copy the data onto the rx_buffer, returns -1 if no data is available
bool phySense(); //function used to sense the physical layer
void beginTransmit(); //starts actual transmission of message
void formRxByte(); //function used to form the byte
bool phyDetect(); // function used to detect the data on the analog port
bool phySense();//returns true if node is idle
bool prevData;
int byte_pos=0;
void updateBitQueue(); //function used to convert the bytes to bits
uint8_t rx_buffer[263]; //message used to store the received data
uint8_t rx_buff[263]; //test variable used to cpy received data
uint8_t tx_message;
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


ISR(TIMER1_OVF_vect)
{ 
  //ISR called every time timer register overflows
  phyDetect(); //function used to check whether any data is available
  if(phy_tx_message_queue.count()){ //checks whether any data is available in the buffer. If yes, then transmits it
    currentPhyState=PHY_TX;
  }
  else {
    currentPhyState=PHY_IDLE_STATE;
  }
  runPhyFSM(currentPhyState);  
  TCNT1 = TIMER1COUNT;
}

void formRxByte()
{
  //Function that is used to form the Byte from the received bits
  char byte=0x00;  
    for(int j=0;j<8;j++){
      if(phy_rx_bit_queue.dequeue()){
             byte==byte|(0x01<<j);
      }
      else{
        byte=byte;
      }
    }
    phy_rx_message_queue.enqueue(byte);
  
}
bool wait;
char bitCount=0;


void decodeFrame(uint8_t byte){
  // function used to decode each byte and make a frame
  if(byte_pos==0){
    if(byte==0xFF){
        byte_pos++;
        rx_buffer[0]=byte;
        data_available=false;      
    }
  }

  else if(byte_pos>=1 && byte_pos<=3){
    byte_pos++;
    rx_buffer[byte_pos]=byte;
  }

  else if(byte_pos==4){
    rx_buffer[byte_pos]=byte;
    data_to_be_rec=byte;
    byte_pos++;
  }

  else if(byte_pos>4 && byte_pos<4+data_to_be_rec){
    rx_buffer[byte_pos]=byte;
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

void beginTransmit()
{  
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
      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_TX_PIN,HIGH); //2-ppm encoding used to encode and send 0s and 1s
        transmitBusy=false;
        transmitCount=0;
      }
    }

    else if(currentBit==false)
    {
      if(transmitCount==0)
      {
       transmitCount++;
       transmitBusy=true;
       digitalWrite(LED_TX_PIN,HIGH); //2-ppm encoding used to encode and send 0s and 1s
      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_TX_PIN,LOW); //2-ppm encoding used to encode and send 0s and 1s
        transmitBusy=false;
        transmitCount=0;
      }
    }
}

void updateBitQueue()
{
  //Function that updates the bit queue that needs to be sent
  txStatus=true;
  if(phy_tx_message_queue.count()){
  tx_message=phy_tx_message_queue.dequeue();
  for(int i=0;i<BYTE_LEN; i++)
  {
    //finding the information in each bit
    if(((tx_message & 0x01)>>i) == 0x01)
    {
      phy_tx_bit_queue.enqueue(true);
    }
    else if(((tx_message & 0x01)>>i) == 0x00)
    {
      phy_tx_bit_queue.enqueue(false);
    } 
  }}  
}

void phy_tx(uint8_t *message, uint8_t len)
{
  phy_tx_message_queue.enqueue(0xFF);
  for(int i=0; i<=len; i++)
  {
    phy_tx_message_queue.enqueue(message[i]); //queues the message onto the tx queue
    
  }
  //does not transmit of in idle state..waits for idle state to finish transmittinfg
  if(currentPhyState!= PHY_RX)
  {
    updateBitQueue();
    currentPhyState=PHY_TX;
  }
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
  switch(currentState)
  {
  case PHY_IDLE_STATE:
  transmitIdle();  
  break;
  
  case PHY_TX:
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
  interrupts();
  pinMode(LED_TX_PIN,OUTPUT);
  pinMode(LED_RX_PIN,INPUT);
  
}

bool phyDetect()
{
  currentAnalogRx=analogRead(LED_RX_PIN);
  //checks for threshold and rising edge
  if(currentAnalogRx >= RX_LOW_THRESHOLD && (currentAnalogRx <= RX_LOW_THRESHOLD + 50))
  {
    currentRx=false;
  }

  //checks if the current State is high
  else if(currentAnalogRx >= RX_HIGH_THRESHOLD )
  {
    currentRx=true;
  }

  if(currentRx==prevRx)
  {
    if(wait_true==true){
      wait_true==false;
      currentPhyState=PHY_IDLE_STATE; //high for more than two slots, then go to idle
    }

    else if(wait_false==true){
      wait_false==false;
      currentPhyState=PHY_IDLE_STATE; //low for more than two slots, then go to idle
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
  if(currentRx>prevRx && prevData!=false) // detects rising edge(true) and prevents misdetection of false
  {
    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
    risingEdge=true;
    fallingEdge=false;
    phy_rx_bit_queue.enqueue(true);
    prevData=true;
    bitCount++;
  }

  if(currentRx<prevRx && prevData!=true) // detects falling edge(false) and prevents misdetection of true
  {
    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
    risingEdge=false;
    fallingEdge=true;
    phy_rx_bit_queue.enqueue(false);
    prevData=false;
    bitCount++;
  }

  if(bitCount==8){ //once 8 bits are received, a byte is formed from it.
    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
    bitCount=0;
    formRxByte();
    decodeFrame(phy_rx_message_queue.dequeue());
  }
  prevAnalogRx=currentAnalogRx;
  prevRx=currentRx;
}


void setup()
{
  initializeTimerLed();
  Serial.begin(9600);
  currentPhyState=PHY_IDLE_STATE;
//  while(!sync());  
}


void loop()
{


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

