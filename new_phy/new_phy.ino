#include<QueueArray.h>
#include<math.h>

#define MAX_BIT_ARRAY_SIZE 500
#define MESSAGE_PREAMBLE 0b00110011
#define LED_TX_PIN 9
#define LED_RX_PIN 12
#define DETECTION_THRESHOLD 0
#define PHY_IDLE_STATE 0
#define RX_LOW_THRESHOLD 0
#define RX_HIGH_THRESHOLD 900
#define PHY_TX 5
#define PHY_RX 6
#define PHY_READY 7
#define PHY_TX_RX 8
#define PHY_IDLE 0
//#define TIMER1COUNT 64535
//#define TIMER1COUNT 0
#define TIMER1COUNT 64545    //65535-16Mhz/8/Freq currently configured for 2Khz
#define BYTE_LEN 8


void TransmitSignal(unsigned char state);
//idle state-ON,OFF,ON
class volcanoPHY{
  public:
  //bool beginTransmitcheck=false;
  bool transmitBusy; //bit used to indicate whether the phy layer is currently transmitting
  int8_t transmitCount; //bit that keeps count of sequence of true/false bits that are transmotted
  int16_t currentAnalogRx,prevAnalogRx; //variable used to store analogVoltage on the Rx Pins
  bool isSynchronized; //used to indicate whether rx is synchronized
  bool bitTransmitcheck;
  bool txStatus,rxStatus,currentBit; 
  bool fallingEdgeDet,risingEdgeDet, currentRx, prevRx, risingEdge, fallingEdge, startSlot;
  void phy_tx(uint8_t* message, unsigned char len); //function called to transmit
  void phy_rx(); //function called to receive
  bool phySense(); //function used to sense the physical layer
  void beginTransmit(); //starts actual transmission of message
  void formRxByte();
  bool phyDetect();
  void updateBitQueue();
  bool sync();
  uint8_t tx_message;
  unsigned char currentLedState,currentPhyState;
  void runPhyFSM(uint8_t currentState);  
  QueueArray <uint8_t> phy_tx_message_queue; //tx byte queue
  QueueArray <uint8_t> phy_rx_message_queue; //rx byte queue
  QueueArray <bool> phy_tx_bit_queue; //tx bit queue
  QueueArray <bool> phy_rx_bit_queue; // rx bit queue
};
volcanoPHY phy;
bool LedState=true;
long prevTime;

ISR(TIMER1_OVF_vect)
{ 
  phy.phyDetect(); //function used to check whether any data is available
  if(phy.phy_tx_message_queue.count()){ //checks whether any data is available in the buffer. If yes, then transmits it
    phy.currentPhyState=PHY_TX;
  }
  LedState=!LedState;
  phy.runPhyFSM(phy.currentPhyState);  
  TCNT1 = TIMER1COUNT;
}

void volcanoPHY::formRxByte()
{
  char byte=0x00;  
  for(int i=0;i<(phy_rx_bit_queue.count()/8);i++)
  {
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
  
}
bool wait;

bool volcanoPHY::phyDetect()
{
  currentAnalogRx=analogRead(LED_RX_PIN);
  //checks for threshold and rising edge
  if(currentAnalogRx >= RX_LOW_THRESHOLD && (currentAnalogRx <= RX_LOW_THRESHOLD + 50))
  {
    currentRx=false;
    return true;
  }
  
  else if(currentAnalogRx >= RX_HIGH_THRESHOLD )
  {
    currentRx=true;
    return true;
  }

  if(currentRx==prevRx)
  {
    if(currentRx==false)
    {
      wait=true;
    }

    if(currentRx==true)
    {
      wait=true;
      
    }
  }
  if(currentRx>prevRx)
  {
    risingEdge=true;
    fallingEdge=false;
    phy_rx_bit_queue.enqueue(false);
  }

  if(prevRx<currentRx)
  {
    risingEdge=false;
    fallingEdge=true;
    phy_rx_bit_queue.enqueue(true);
  }
  prevAnalogRx=currentAnalogRx;
  prevRx=currentRx;
}

void volcanoPHY::beginTransmit()
{  
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
       digitalWrite(LED_TX_PIN,HIGH); 
      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_TX_PIN,LOW);
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
       digitalWrite(LED_TX_PIN,LOW); 
      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_TX_PIN,HIGH);
        transmitBusy=false;
        transmitCount=0;
      }
    }
}

void volcanoPHY::updateBitQueue()
{
  //Function that updates the bit queue that needs to be sent
  txStatus=true;
  if(phy_tx_message_queue.count()){
  tx_message=phy_tx_message_queue.dequeue();
  for(int i=0;i<BYTE_LEN; i++)
  {
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

void volcanoPHY::phy_tx(uint8_t *message, uint8_t len)
{
  phy_tx_message_queue.enqueue(MESSAGE_PREAMBLE);
  for(int i=0; i<=len; i++)
  {
    phy_tx_message_queue.enqueue(message[i]); //queues the message onto the tx queue
    
  }
  //does not transmit of in idle state..waits for idle state to finish transmittinfg
  if(currentPhyState!= PHY_RX)
  {
    currentPhyState=PHY_TX;
    updateBitQueue();
  }
  
}

void volcanoPHY::phy_rx()
{
  
}

//bool idleState=false;
int idleVar=0;
void volcanoPHY::runPhyFSM(uint8_t currentState) 
{
  //state machine for the PHY layer
  switch(currentState)
  {
  case PHY_IDLE_STATE:
  transmitIdle();  
  break;
  
  case PHY_TX:
  phy.beginTransmit();
  break;

  case PHY_RX:
  phy_rx();
  break;
  }
}

void transmitIdle()
{
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
  
}

void setup()
{
  initializeTimerLed();
  Serial.begin(9600);
  phy.currentPhyState=PHY_IDLE_STATE;
//  while(!phy.sync());  
}


void loop()
{


}

void printBool(bool state){
    switch(state){
      case true:
      Serial.println("true");
      break;

      case false:
      Serial.println("false");
      break;
      
    }
}

