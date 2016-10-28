#define MAC_IDLE 0
#define MAC_INIT_WAIT 1
#define MAC_RANDOM_CW 2
#define MAC_WAIT_CW 3
#define MAC_ACCESS 4
#define MAC_WAIT_ACK 6
#define CW_SLOT 16
#define CW_DELAY 500
#define MAC_ACK_LEN 0
#define MAC_PDU_MAX_LEN 261 // src(1) + dst(1) + res(1) + nav(1) + data(255) + fcs(2)

class volcanoMAC {
    uint8_t mac_state;
    uint8_t random_cw;
    uint8_t i;
    bool b_medium_idle;
    QueueArray <char> mac_queue;

    void _access_mac();
    void _rx_mac();
    void _calculate_fcs();
    void _mac_idle();
    void _mac_init_wait();
    void _mac_random_cw();
    void _mac_wait_cw();
    void _mac_access();
    void _mac_wait_ack(); // read phy
    void _mac_fsm_control();        
  
  public:
    char mac_rx();
    void mac_tx();
}

char volcanoMAC::mac_rx() {
  mac_queue.dequeue();
}

void volcanoMAC::mac_tx() {
  mac_queue.enqueue();
}

void setup() { // 
  // put your setup code here, to run once:
  mac_state = MAC_IDLE;
  random_cw = 0;
  // setup task library (TaskScheduler) to call mac_fsm_control() periodically
}

void loop() {
  // put your main code here, to run repeatedly:
  _mac_fsm_control();
}

void mac_fsm_control() {
   // read phy
  phy_rx();
  mac_queue.push();
  switch(mac_state){
    case MAC_IDLE:
      _mac_idle();
      break;
    case MAC_INIT_WAIT:
      _mac_init_wait();
      break;
    case MAC_RANDOM_CW:
      _mac_cw();
      break;
    case MAC_WAIT_CW:
      _mac_wait();
      break;
    case MAC_ACCESS:
      _mac_access();
      break;
    case MAC_WAIT_ACK:
      _mac_wait_ack();
      break;
  }
}

void _wait_ack_slot() {
  uint8_t slot_counter=0;
  while (cw_slot_counter < 4) {
    delayMicroseconds(CW_DELAY);
    slot_counter++;
  }
}

void _wait_cw_slot(uint8_t num_cw) {
  uint8_t cw_slot_counter=0;
  uint8_t cw_counter=0;
   
  while (cw_counter < num_cw) {
    while (cw_slot_counter < CW_SLOT) {
      delayMicroseconds(CW_DELAY);
      cw_slot_counter++;
    }
    cw_counter++;
  }
}

void _mac_idle() {
  phy_rx();
  // check if medium is idle
  b_medium_idle = PhySense(*_req, *_cnf);
  if (b_medium_idle) {
    mac_state = MAC_INIT_WAIT;
  } else {
    mac_state = MAC_IDLE;
  }
}

void _mac_init_wait(){
  _wait_one_cw_slot(1);
  mac_state = MAC_RANDOM_CW;
}

void _mac_random_cw() {
  random_cw = random(0, 16); 
  mac_state = MAC_WAIT_CW;
}

void _mac_wait_cw() {
  i = 0;
  b_medium_idle = true;
  if ((i<=random_cw) && b_medium_idle ){
    b_medium_idle = phy_check_medium();
    if (b_medium_idle) {
      _wait_cw_slot(1);
      i++;
    } else {
      mac_state = MAC_RANDOM_CW;
    }
  }
  if (i == random_cw) {
    mac_state = MAC_ACCESS;
  }
}

void _mac_access() {
  _mac_create_pdu();
  phy_tx();
  mac_state = MAC_WAIT_ACK;
}

void _mac_wait_ack() {
  mac_state = MAC_ACCESS;
  if ((mac_state == MAC_ACCESS) && (i<134)) { // wait 134 slots
    _wait_cw_slot(1);
    phy_rx(); // trying to read ACK
    if { // if ACK received
      mac_state = MAC_IDLE;
      i = 135; // get out of wait_ack loop
    }
  }
}

void _mac_create_pdu(uint8_t mac_pdu[], uint8_t frame_body_size, uint8_t frame_control, uint8_t dest, uint8_t src, uint8_t seq, uint8_t *data, uint8_t body_len) {
  uint16_t mac_pdu_len = 5+body_len+2;
  uint16_t fcs;
  uint8_t i;

  mac_pdu[0] = frame_body_size;
  mac_pdu[1] = frame_control;
  mac_pdu[2] = dest;
  mac_pdu[3] = src;
  mac_pdu[4] = seq;
  for (i=0;i<body_len;i++) {
    mac_pdu[i+5] = data[i];
  }
  fcs = calculate_fcs();  
  mac_pdu[4+i] = (fcs >> 8) & 0xFF;
  mac_pdu[4+i+1] = fcs & 0xFF;
}

void _calculate_fcs() {

}
