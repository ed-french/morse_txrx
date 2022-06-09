
#define USE_ESPNOW // Instead of wifi


#include <Arduino.h>
#include <WiFi.h>

#ifdef USE_ESPNOW
  #include <esp_now.h>
  #include <esp_wifi.h>
  #define WIFI_CHANNEL 0
#elif
  
  #include <WiFiUdp.h>
  #include "credentials.h"
#endif


#include "driver/ledc.h"
#include "driver/periph_ctrl.h"

#ifdef JOSEPHINE
  #define THIS_DEVICE_NAME "josephine"
  #define PARTNER_DEVICE_NAME "edward"
  #define PARTNER_DEVICE_ACK_STR " ack edward"
  #define NETWORK_NAME "josephine morse"
#else
  #define THIS_DEVICE_NAME "edward"
  #define PARTNER_DEVICE_NAME "josephine"
  #define PARTNER_DEVICE_ACK_STR " ack josephine"
  #define NETWORK_NAME "edward morse"
#endif




#define LOOP_WAIT_TIME_MS 1
#define WAIT_BEFORE_RESEND 100
#define OWN_FREQ 523
#define OTHER_FREQ 440
#define BOTH_FREQ 740
#define ACKS_AWAITED_SIZE_WORDS 64
#define PIN_KEY 12
#define PIN_SPEAKER 13

#ifdef USE_ESPNOW
  esp_now_peer_info_t peerInfo;
  uint8_t broadcast_addr[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
#else
  WiFiUDP udp;
#endif

const char  g_partner_device_ack_str[]=PARTNER_DEVICE_ACK_STR;
const char partner_device_name[]=PARTNER_DEVICE_NAME;
const char this_device_name[]=THIS_DEVICE_NAME;



struct g_state_t
{
  //char waiting_for_ack_id[4];// This is the last sent message we are currently awaiting a reply on
  bool last_sent; // true is key down
  bool received_state; // true means the other device has key down
  bool ack_rxd; // true once the last message ack is rx'd
  uint32_t last_sent_time; // millis() when the last send was tried
  uint16_t current_speaker_frequency;
  bool key_down;
  char acks_awaited[4*ACKS_AWAITED_SIZE_WORDS]; //4 per awaited, up-to 16 awaited
  uint8_t next_ack_index;
  uint32_t last_keydown_time;
  char rx_buffer[64];
  char tx_buffer[64];
};

g_state_t g_state; // Holds the global state






// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
// here is broadcast address

/*
      Multicasts new state, returns true if it's acknowledged
      within ACK_WAIT_TIME_MS

      messages look like:
        
                picokit 712 1
                ^       ^   ^ 
                |       |   |
                |       |   New bool state 0 or 1
      device name  message_id

      acknowledgements look like:

                712 ack lolin
                ^    ^    ^
                |    |    |
      message ack'd  |    device ack'ing
                     |
                always ack

  */



//const char * udpAddress = "224.3.29.71";
const char * udpAddress = "255.255.255.255";
const uint8_t udp_addr_bytes[4]={71,29,3,224};
const int udp_broadcast_port = 10000;
char * sender_name=THIS_DEVICE_NAME;


#ifdef USE_ESPNOW
void buff_print_mac(char * buffer,uint8_t * mac_addr)
{
  // Writes a nicely formatted mac address
  sprintf(buffer,"%x:%x:%x:%x:%x:%x",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);
}

void OnRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  // Just places received message into rx_buffer
  strncpy(g_state.rx_buffer,(char *)incomingData,min(len,(int)sizeof(g_state.rx_buffer)-1));
  Serial.print("Rx from mac:");
  char buff[40];
  buff_print_mac(buff,(uint8_t*)mac);
  Serial.printf(": %s\n",g_state.rx_buffer);

}

#endif




void play_tone(uint16_t freq)
{


    ledcSetup(0, freq, 1);
    
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(PIN_SPEAKER, 0);

    ledcWrite(0, (freq>0)?1:0);

}


bool get_key_down()
{
  g_state.key_down=!digitalRead(PIN_KEY);
  //Serial.printf("%s",g_state.key_down?"X":".");
  return g_state.key_down;
}

bool serial_get_key_down()
{
  char ser_rx_buff[6];
  uint8_t size=Serial.read(ser_rx_buff,1);
  bool kd=(size==1);
  if (kd)
  {
    Serial.print("*");
  }
  return kd;  
}

void set_speaker()
{
  g_state.key_down=get_key_down();
  uint8_t play_state=2*g_state.key_down+(uint8_t)g_state.received_state;
  uint16_t freq;
  switch (play_state)
  {
    case 0:
      freq=0;// Speaker off
      break;
    case 1: // rx'd only
      freq=OTHER_FREQ;
      break;
    case 2: // tx only
      freq=OWN_FREQ;
      break;
    case 3:
      freq=BOTH_FREQ;
      break;
    default:
      Serial.println("!!!!!!!! ERROR IMPOSSIBLE STATE FOR SPEAKER!!!!!!");
      return;
  }

  if (freq==g_state.current_speaker_frequency) return;
  Serial.printf("\t\tSPEAKER SET TO : %d\n",freq);
  play_tone(freq);
  g_state.current_speaker_frequency=freq;
  
}

void play_beep(uint16_t freq, uint16_t duration_ms)
{
  play_tone(freq);
  delay(duration_ms);
  play_tone(0);
}





void transmit(bool new_state)
{
  uint32_t message_id=random(999); // pick a random message id to track correct ack
  char msg_id_str[4];
  sprintf(msg_id_str,"%03d",message_id);// Shound have leading zeros if required

  // Add to acks awaited array
  memcpy(g_state.acks_awaited+4*g_state.next_ack_index,msg_id_str,3);
  g_state.next_ack_index=(g_state.next_ack_index+1)&(ACKS_AWAITED_SIZE_WORDS-1); // wrap round

  
  sprintf(g_state.tx_buffer,"%s %s %s",sender_name,msg_id_str,new_state?"1":"0");
  Serial.printf(">>>>: %s\n",g_state.tx_buffer);


  #ifdef USE_ESPNOW
    // Send via espnow

    peerInfo.channel=WIFI_CHANNEL;
    peerInfo.encrypt=false;
    memcpy(peerInfo.peer_addr,broadcast_addr,6);
    esp_err_t peer_add_error=esp_now_add_peer(&peerInfo)-ESP_ERR_ESPNOW_BASE;
    if (peer_add_error != ESP_OK)
    {
      Serial.printf("Failed to add peer: %d\n",peer_add_error);
      

      //return;
    }


    // Was sent to pair_address, now it's broadcast
    esp_err_t result=esp_now_send(broadcast_addr,
                              (uint8_t *) g_state.tx_buffer,
                              strlen(g_state.tx_buffer)+1);
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.printf("Error sending the data: %s\n",esp_err_to_name(result));
    }

  #else
    
      // Send via UDP broadcast  
      udp.beginPacket(udpAddress,udp_broadcast_port);
      udp.print(g_state.tx_buffer);
      udp.endPacket();
  #endif

  //async verison
  //audp.broadcast(g_state.tx_buffer);

  g_state.ack_rxd=false;
  g_state.last_sent_time=millis();
  g_state.last_sent=new_state;
  //strncpy(g_state.waiting_for_ack_id,msg_id_str,3);
  //Serial.printf("Now waiting for : %s\n",g_state.waiting_for_ack_id);

}


// Sync receive
void receive_sync(uint32_t max_wait_time_ms)
{
  uint32_t endtime=millis()+max_wait_time_ms;
  
  while (millis()<endtime)
  {
    #ifdef USE_ESPNOW
    int8_t readcount=strnlen(g_state.rx_buffer,sizeof(g_state.rx_buffer));
    #else
    udp.parsePacket();
    uint8_t readcount=udp.read(g_state.rx_buffer, sizeof(g_state.rx_buffer));
    #endif
    if (readcount > 0)
    {
      Serial.printf("\t\t\t\tRx: %s\n",g_state.rx_buffer);
      if (strncmp(g_state.rx_buffer,partner_device_name,strlen(partner_device_name))!=0)
      {
        // No match, so we ignore it
        Serial.println("\t\t\t\tIgnoring as not from partner");
      } else {
        g_state.received_state=(g_state.rx_buffer[strlen(g_state.rx_buffer)-1]=='1');
        Serial.printf(">>>>>>>    -------   %s\n",g_state.received_state?"1":"0");
        
        
        // PLAY THE INBOUND SOUND GOES HERE!
        
        set_speaker();

      }
      //Zero the buffer again
      memset(g_state.rx_buffer,0,sizeof(g_state.rx_buffer));


    }

  }
}



void setup()
{
  Serial.begin(115200);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_protocol( WIFI_IF_AP, WIFI_PROTOCOL_LR );
  pinMode(PIN_KEY,INPUT_PULLUP);
  
  Serial.println();

  #ifdef USE_ESPNOW
  
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("\n\nError initializing ESP-NOW");
      delay(4000);
      ESP.restart();
    }
    esp_now_register_recv_cb(OnRecv);
    

  #else
    Serial.print("Connecting to ");
    Serial.println(ssid);
    //Connect to the WiFi network
    WiFi.setHostname(NETWORK_NAME);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    Serial.println("");

    // Wait for connection
    uint16_t try_count=0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(300);
      play_beep(392,100);
      try_count++;
      Serial.print(".");
      if (try_count>50)
      {
        ESP.restart();
      }
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    udp.begin(udp_broadcast_port);
  #endif


  play_beep(440,450);
  play_beep(523,300);
  play_beep(740,150);

  // Reset global state
  g_state.ack_rxd=false;
  g_state.last_sent=false;
  g_state.last_sent_time=0;
  g_state.received_state=false;
  //memset(g_state.waiting_for_ack_id,0,4);
  memset(g_state.rx_buffer,0,sizeof(g_state.rx_buffer));
  memset(g_state.tx_buffer,0,sizeof(g_state.tx_buffer));
  g_state.current_speaker_frequency=0;
  memset(g_state.acks_awaited,0,sizeof(g_state.acks_awaited));
  g_state.next_ack_index=0;


  set_speaker();


  

}



void loop()
{
  // while(true)
  // {
  //   g_state.key_down=get_key_down();
  //   delay(200);
  // }

  // Wait for key down
  while (!g_state.key_down)
  {
    g_state.key_down=get_key_down();
    receive_sync(10);
  }
  transmit(g_state.key_down);
  set_speaker();
  receive_sync(10);
  transmit(g_state.key_down);
  receive_sync(10);
  transmit(g_state.key_down);
  receive_sync(50);

  while(g_state.key_down)
  {
    g_state.key_down=get_key_down();
    receive_sync(10);
  }
  transmit(g_state.key_down);
  set_speaker();
  receive_sync(10);
  transmit(g_state.key_down);
  receive_sync(10);
  transmit(g_state.key_down);
  receive_sync(50);

  
  receive_sync(LOOP_WAIT_TIME_MS);




}
