#define MAC_IDLE 0
#define MAC_INIT_WAIT 1
#define MAC_RANDOM_CW 2
#define MAC_WAIT_CW 3
#define MAC_TX 4
#define MAC_RX 5
#define MAC_WAIT_ACK 6
#define CW_SLOT 16
#define CW_DELAY 500
#define MAC_ACK_LEN 0
#define MAC_PDU_MAX_LEN 262 // 5+255+2

uint8_t mac_state;
uint8_t random_cw;

public void MacDataDeliver(*_req, *_cnf, *_ind, *_res);

void mac_idle();
void mac_init_wait();
void mac_random_cw();
void mac_wait_cw();
void mac_tx();
void mac_wait_ack();
void mac_rx();
void mac_fsm_control();

void setup() {
  // put your setup code here, to run once:
  mac_state = MAC_IDLE;
  random_cw = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  mac_fsm_control();
}

void mac_fsm_control() {
  switch(mac_state){
    case MAC_IDLE:
      mac_idle();
      break;
    case MAC_INIT_WAIT:
      mac_init_wait();
      break;
    case MAC_RANDOM_CW:
      mac_cw();
      break;
    case MAC_WAIT_CW:
      mac_wait();
      break;
    case MAC_TX:
      mac_tx();
      break;
    case MAC_WAIT_ACK:
      mac_wait_ack();
      break;
    case MAC_RX:
      mac_rx();
      break;
  }
}

void wait_ack_slot() {
  uint8_t slot_counter=0;
  while (cw_slot_counter < 4) {
    delayMicroseconds(CW_DELAY);
    slot_counter++;
  }
}

void wait_cw_slot(uint8_t num_cw) {
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

void mac_idle() {
  // check if medium is idle
  bool b_medium_idle = PhySense(*_req, *_cnf);
  if (b_medium_idle) {
    mac_state = MAC_INIT_WAIT;
  } else {
    mac_state = MAC_IDLE;
  }
}

void mac_init_wait(){
  wait_one_cw_slot(1);
  mac_state = MAC_RANDOM_CW;
}

void mac_random_cw() {
  random_cw = random(0, 16); 
  mac_state = MAC_WAIT_CW;
}

void mac_wait_cw() {
  uint8_t i = 0;
  bool b_medium_idle = true;
  while ( (i<=random_cw) && b_medium_idle ){
    b_medium_idle = phy_check_medium();
    if (b_medium_idle) {
      wait_cw_slot(1);
      i++;
    }
  }
  if (i == random_cw) {
    mac_state = MAC_TX;
  } else {
    mac_state = MAC_RX;
  }
}

void mac_tx() {
  mac_create_pdu();
  PhyPduDeliver(*_req, *_cnf, *_cnf);
  mac_state = MAC_WAIT_ACK;
}

void mac_wait_ack() {
  uint i;
  mac_state = MAC_TX;
  while ((mac_state == MAC_TX) && (i<134)) {
    wait_cw_slot(1);
    PhyPduDeliver(*_req, *_cnf, *_cnf); // trying to read ACK
    if {
      mac_state = MAC_IDLE;
    }
  }
}

void mac_rx() {
  // read from PHY
  PhyPduDeliver(*_req, *_cnf, *_cnf);
  wait_ack_slot();
  mac_create_pdu(); // send ACK

  wait_one_cw_slot(1); // Wait before check if medium is idle
  if (phy_check_medium()) {
    mac_state = MAC_WAIT_CW;
  }
}

void mac_create_pdu(uint8_t mac_pdu[], uint8_t frame_body_size, uint8_t frame_control, uint8_t dest, uint8_t src, uint8_t seq, uint8_t *data, uint8_t body_len) {
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
