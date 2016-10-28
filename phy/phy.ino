#include <QueueArray.h>
#define LED_PIN 11
#define DETECTION_THRESHOLD 0
#define PHY_IDLE_STATE 0
#define PHY_IDLE_STATE_0 1  //4 states in PHY_IDLE
#define PHY_IDLE_STATE_1 2
#define PHY_IDLE_STATE_2 3
#define PHY_IDLE_STATE_3 4
#define PHY_TX 5
#define PHY_RX 6
#define PHY_READY 7
#define PHY_TX_RX 8
#define TIMER1COUNT 64545    //65535-16Mhz/8/Freq currently configured for 2Khz
#define BYTE_LEN 8

//idle state-ON,OFF,ON
class volcanoPHY{
  
  public:
  //bool beginTransmitcheck=false;
  bool transmitBusy;
  int8_t transmitCount;
  bool isSynchronized;
  bool bitTransmitcheck;
  bool txStatus,rxStatus,currentBit;
  void phy_tx();
  void phy_rx();
  void beginTransmit();
  uint8_t tx_message;
  unsigned char currentLedState,currentPhyState;
  void runPhyFSM(uint8_t currentState);
  QueueArray <uint8_t> phy_tx_message_queue;
  QueueArray <uint8_t> phy_rx_message_queue;
  QueueArray <bool> phy_tx_bit_queue;
  QueueArray <bool> phy_rx_bit_queue;
  
};
volcanoPHY phy;


ISR(TIMER1_OVF_vect)
{
   //ISR function- caled every 500us  
  TCNT1 = TIMER1COUNT;
  phy.runPhyFSM(phy.currentPhyState);
  phy.beginTransmit();
}

void phy::beginTransmit()
{
  if(phy_tx_bit_queue.count)
  {
    //condition to check whether any other bit is being transmitted
    if(!transmitBusy)
    {
    currentBit=dequeue(phy_tx_bit_queue);
    transmitCount=0;
    transmitBusy=true;
    }
    if(currentBit==true)
    {
      if(transmitCount==0)
      {
       transmitCount++;
       digitalWrite(LED_PIN,HIGH); 

      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_PIN,LOW);
        transmitBusy=false;
        transmitCount=0;
      }
    }

    else if(currentBit==false)
    {
      if(transmitCount==0)
      {
       transmitCount++;
       digitalWrite(LED_PIN,LOW); 

      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_PIN,HIGH);
        transmitBusy=false;
        transmitCount=0;
      }
    }

  }
}

void volcanoPHY::phy_tx(uint8_t *message, uint8_t len)
{
  for(int i=0; i<=len; i++)
  {
    phy_tx_message_queue.enqueue(message[i]); //queues the message onto the tx queue
    
  }
  //does not transmit of in idle state..waits for idle state to finish transmittinfg
  if(currentPhyState!= PHY_IDLE_STATE_0 || currentPhyState!= PHY_IDLE_STATE_1 || currentPhyState!= PHY_IDLE_STATE_2 || currentPhyState!= PHY_IDLE_STATE_3)
  {
    currentPhyState=PHY_TX;
    updateBitQueue();
  }
  
}

void volcanoPHY::updateBitQueue()
{
  //Function that updates the bit queue that needs to be sent
  txStatus=true;
  tx_message=phy_tx_message_queue.dequeue();
  for(int i=0;i<BYTE_LEN; i++)
  {
    if(((tx_message & 0x01)>>i) == 0x01)
    {
      phy_tx_bit_queue.enqueue(true);
    }
    else if(((tx_message & 0x01)>>i) == 0x00)
    {
      phy_rx_bit_queue(false);
    }
    
  }
  
}

void volcanoPHY::transmit()
{
  
}

void phy_rx()
{
  
}


void volcanoPHY::runPhyFSM(uint8_t currentState) 
{
  //state machine for the PHY layer
  switch(currentState)
  {
//  case PHY_IDLE_STATE_0:
//
//  break;
//
//  case PHY_IDLE_STATE_1:
//
//  break;
//
//  case PHY_IDLE_STATE_2:
//
//  break;
//
//  case PHY_IDLE_STATE_3:
//
//  break;

  case PHY_IDLE_STATE:
  phy_tx_bit_queue.enqueue(false);
  phy_tx_bit_queue.enqueue(true);
  break;

//  case PHY_READY:
//
//  break;

  case PHY_TX:
  phy_tx();
  break;

  case PHY_RX:
  phy_rx();
  break;
  
  }
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
  pinMode(LED_PIN,OUTPUT);
  
}

void setup()
{
  initializeTimerLed();
  phy.currentPhyState=PHY_IDLE_STATE_0;
    
}


void loop()
{


}


