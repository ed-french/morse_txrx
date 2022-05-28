#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "credentials.h"
#include <AsyncUDP.h>
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"

#ifdef PICOKIT
  #define THIS_DEVICE_NAME "picokit"
  #define PARTNER_DEVICE_NAME "lolin"
  #define PARTNER_DEVICE_ACK_STR " ack lolin"
#else
  #define THIS_DEVICE_NAME "lolin"
  #define PARTNER_DEVICE_NAME "picokit"
  #define PARTNER_DEVICE_ACK_STR " ack picokit"
#endif


#define LOOP_WAIT_TIME_MS 1
#define WAIT_BEFORE_RESEND 100
#define OWN_FREQ 1000
#define OTHER_FREQ 500
#define BOTH_FREQ 700
#define ACKS_AWAITED_SIZE_WORDS 64
#define PIN_KEY 12

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
AsyncUDP audp;






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



const char * udpAddress = "224.3.29.71";
const uint8_t udp_addr_bytes[4]={71,29,3,224};
const int udpPort = 10000;
char * sender_name=THIS_DEVICE_NAME;

//create tx UDP instance
//WiFiUDP udp;


void play_tone(uint16_t freq)
{
    /*periph_module_enable(PERIPH_LEDC_MODULE);

    // Set up timer
    ledc_timer_config_t ledc_timer;
    ledc_timer.duty_resolution=LEDC_TIMER_1_BIT;
    ledc_timer.freq_hz=freq;
    ledc_timer.speed_mode=LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num=LEDC_TIMER_0;


    





    /*
     = {
       .duty_resolution = LEDC_TIMER_1_BIT,     // We need clock, not PWM so 1 bit is enough.
       .freq_hz = freq,                      // Clock frequency, 1 MHz
       .speed_mode = LEDC_HIGH_SPEED_MODE,
       .timer_num = LEDC_TIMER_0,
        // .clk_cfg = LEDC_USE_APB_CLK          // I think this is needed for neweer espressif software, if problem, try uncommenting this line
    };
    */
   /*
    ledc_timer_config(&ledc_timer);

    // Set up GPIO PIN
    ledc_channel_config_t channel_config;
    channel_config.channel    = LEDC_CHANNEL_0;
    channel_config.duty       = 1;
    channel_config.gpio_num   = 2;                   // GPIO pin
    channel_config.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_config.timer_sel  = LEDC_TIMER_0;
    
    
    
    /* = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 1,
        .gpio_num   = 2,                        // GPIO pin
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0
    };*/

    /*
    ledc_channel_config(&channel_config);
    */

    ledcSetup(0, freq, 1);
    
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(2, 0);

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


void receive()
{
  //memset(g_state.rx_buffer,0,sizeof(g_state.rx_buffer)); // Clear the buffer
  //udp.beginMulticast(IPAddress(udp_addr_bytes[0],udp_addr_bytes[1],udp_addr_bytes[2],//udp_addr_bytes[3]),udpPort);
  //udp.begin(udpPort);
  //udp.parsePacket();
  // if(udp.read(g_state.rx_buffer, 50) == 0)
  // {
  //   return; // Nothing rxd
  // }
  
  Serial.printf("\t\t\trx: %s\n",g_state.rx_buffer);
  // Three possibilities:
  //     we have recieved an ack from another device- e.g. server
  //     we have received an ack from the partner device
  //     we have received a new remote state
  //     we have received a corrupt/random packet
  
  // test for ack from partner
  if (strcmp(g_state.rx_buffer+3,g_partner_device_ack_str)==0)
  {
    Serial.println("Ignoring ack");
    return;
    // is the message_id the one we are waiting for
    Serial.println("ack received from partner...");
    bool found=false;
    for (uint8_t i=0;i<ACKS_AWAITED_SIZE_WORDS;i++)
    {
      if (strncmp(g_state.rx_buffer,g_state.acks_awaited+4*i,3)==0)
      {
        found=true;
        break;
      }
    }
    if (!found)
    {
      Serial.printf("\t\t\t\tNot an ack we were awaiting");
      return;
    }
    // It is the ack we were waiting for!
    Serial.println("\t\t\t\tsuccessful ack rx");
    g_state.ack_rxd=true;
    return;
  }

  // test if we have a new state from the partner keydown
  // e.g. picokit 712 1
  if (strncmp(g_state.rx_buffer,partner_device_name,strlen(partner_device_name))!=0)
  {
    // No match, so we ignore it
    Serial.println("\t\t\t\tIgnoring as not from partner");
    return;
  }
  // Get the message_id
  char message_id_str[4];
  uint8_t rx_len=strnlen(g_state.rx_buffer,sizeof(g_state.rx_buffer)-1);
  memcpy(message_id_str,g_state.rx_buffer+rx_len-5,3);
  message_id_str[3]=(char)0;
  //Serial.printf("\t\t\t\t\tmessage_id_str: %s\n",message_id_str);
  //Serial.printf("\t\t\t\t\tthis_device_name: %s\n",this_device_name);
  //delay(500); // Pause to let the serial buffer send before it resets!
  // make the ack
  sprintf(g_state.tx_buffer,"%s ack %s",message_id_str,this_device_name);



  // We do have a new state, so we record it
  g_state.received_state=(g_state.rx_buffer[strlen(g_state.rx_buffer)-1]=='1');
  Serial.printf(">>>>>>>    -------   %s\n",g_state.received_state?"1":"0");
  set_speaker();
  // Now send the ack back...
  // IPAddress ip_address=udp.remoteIP(); // We need this for ack'ing
  // uint16_t remote_port=udp.remotePort();

  
  // udp.beginPacket(udpAddress, udpPort);
  // udp.print(g_state.tx_buffer);
  
  // udp.endPacket();
  // audp.broadcast(g_state.tx_buffer);

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
  

  // udp.beginPacket(udpAddress, udpPort);
  // udp.print(g_state.tx_buffer);

  // udp.endPacket();
  audp.broadcast(g_state.tx_buffer);

  g_state.ack_rxd=false;
  g_state.last_sent_time=millis();
  g_state.last_sent=new_state;
  //strncpy(g_state.waiting_for_ack_id,msg_id_str,3);
  //Serial.printf("Now waiting for : %s\n",g_state.waiting_for_ack_id);

}
/*
bool try_send_state(bool new_state)
{
  
  char buffer[50] = "\0";
  char rx_buffer[50]="\0";
  uint32_t message_id=random(999); // pick a random message id to track correct ack
  char msg_id_str[4];
  sprintf(msg_id_str,"%03d",message_id);// Shound have leading zeros if required

  
  sprintf(buffer,"%s %d %s",sender_name,message_id,new_state?"1":"0");
  Serial.printf("About to send: %s\n",buffer);
  
  //This initializes udp and transfer buffer
  udp.beginPacket(udpAddress, udpPort);
  udp.print(buffer);
  //udp.write(buffer, 11);
  udp.endPacket();

  

  uint8_t try_count=6;

  while (try_count>0)
  {
    //receive response from server
    memset(buffer, 0, sizeof(buffer)); // Zero out the buffer for next time
    delay(LOOP_WAIT_TIME_MS);
    udp.parsePacket();
    if(udp.read(rx_buffer, 50) > 0)
    {
      Serial.printf("\t\t\t\tReply # %d : %s\n",try_count,rx_buffer);
      
    }
    
    // check rx buffer is the ack we wanted...
    // for now assume so if the message id is right

    if (strncmp(rx_buffer,msg_id_str,strlen(msg_id_str))==0)
    {
      return true;
    }
    try_count--;
  }
  Serial.println("Ran out of tries for an ack!!!!!!");
  
  return false;

}

void old_loop()
{
  // Do 5 sends, then pause for a bit
  bool state=false;
  for (uint8_t i=0;i<5;i++)
  {
    bool ack_success=try_send_state(state);
    Serial.printf("\t\t\t\t\t\t%s\n",ack_success?"success":"no-ack!!!!!!!!!!!!!!!!!");
    delay(100);
  }
  delay(10000);
}
*/



void setup()
{
  Serial.begin(115200);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //Connect to the WiFi network
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.println("");
  delay(1000);
  pinMode(PIN_KEY,INPUT_PULLUP);

  // Wait for connection
  uint16_t try_count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
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

  // Set up async handler for rx
  if(audp.listen(udpPort)) {
        audp.onPacket([](AsyncUDPPacket packet) {
            //Serial.printf("Received %d bytes of data: ",packet.length());
            // Serial.write(packet.data(), packet.length());
            memcpy(g_state.rx_buffer,(char *)packet.data(),packet.length());
            g_state.rx_buffer[packet.length()]=(char)0;
            //Serial.printf("\t\t\t\t\t\trxstr: %s\n",g_state.rx_buffer);
            if (packet.length()>0)
            { 
              receive();// Process the buffer!
            }
        });
    } else {
      Serial.println("\n\nFailed to set up async udp!!!\n\n");
      delay(2000);
      ESP.restart();
    }
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
  }
  transmit(g_state.key_down);
  delay(10);
  transmit(g_state.key_down);
  delay(10);
  transmit(g_state.key_down);
  delay(50);

  while(g_state.key_down)
  {
    g_state.key_down=get_key_down();
  }
  transmit(g_state.key_down);
  delay(10);
  transmit(g_state.key_down);
  delay(10);
  transmit(g_state.key_down);
  delay(50);

  
  // if (g_state.key_down) g_state.last_keydown_time=millis();
  // set_speaker();
  // if ((millis()-g_state.last_keydown_time)<5000)
  // {
  //   transmit(g_state.key_down);
  // } else {
  //   Serial.print("~");
  // }
  //receive(); // Tries to rx- could be an ack, a state change, or nothing- non-blocking
            // results go into g_state
  
  // uint32_t time_since_last_send=millis()-g_state.last_sent_time;
  // if (g_state.key_down!=g_state.last_sent || ((!g_state.ack_rxd) && time_since_last_send>WAIT_BEFORE_RESEND))
  // {
  //   if (g_state.key_down!=g_state.last_sent)
  //   {
  //     // First time we need to re-zero the awaited acks array
  //     Serial.println("key change!");
  //     memset(g_state.acks_awaited,0,sizeof(g_state.acks_awaited));
  //     g_state.next_ack_index=0;
  //   }
  //   transmit(g_state.key_down);
  // }
  delay(LOOP_WAIT_TIME_MS);




}

/*
void old_loop()
{

  uint32_t starttime=millis();
  for (uint16_t i=0;i<100;i++)
  {
  //data will be sent to server
  char buffer[50] = "\0";
  sprintf(buffer,"esp->server: %d\n",message_count);
  message_count++;


  //This initializes udp and transfer buffer
  udp.beginPacket(udpAddress, udpPort);
  udp.print(buffer);
  //udp.write(buffer, 11);
  udp.endPacket();
  memset(buffer, 0, sizeof(buffer));
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  //receive response from server,
  if(udp.read(buffer, 50) > 0)
  {
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
  }
  
  delay(1);
  }
  float time_taken=(millis()-starttime)/1000.0;
  printf("\nTook: %.3f seconds\n",time_taken);
  delay(12000);
}
*/





/*
 * Example: Generate 1 MHz clock signal with ESP32
   note 24.3.2020/pekka
   ESP32: LEDC peripheral can be used to generate clock signals between 40 MHz (half of APB clock) and approximately 0.001 Hz.
   Please check the LEDC chapter in Technical Reference Manual. Example code below.
*/

/*



*/