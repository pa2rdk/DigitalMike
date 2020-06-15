# 1 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
# 2 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 2
# 3 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 2
# 4 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 2
# 5 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 2
# 6 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 2
# 7 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 2

#define RXPIN_WS 25
#define RXPIN_DATA 22
#define RXPIN_SCK 12

#define TXPIN_WS 17
#define TXPIN_DATA 13
#define TXPIN_SCK 2
#define CONN_LED 5
#define STAT_LED 23
#define PTT_OUT 18
#define PTT 39
#define MUTE 14

#define OLED_SCL 15 /* GPIO15*/
#define OLED_SDA 4 /* GPIO4*/
#define OLED_RST 16
#define OLED_ADR 0x3C /* Default 0x3C for 0.9", for 1.3" it is 0x78*/
#define hasLCD 

#define offsetEEPROM 0x10
#define EEPROM_SIZE 200

#define READ_LEN 512
#define STD_DELAY 5 /*depends on READ_LEN, READ_LEN 1024 = STD_DELAY 20*/
#define bufferQty 22
#define commandQty 5

uint8_t COMMANDO[10] = {0};
volatile bool isConnected = false;
volatile bool activePTT = false;
bool i2sDebug = true;
bool UDPDebug = true;
bool remotePTT = false;
bool CommunicationStarted = false;
volatile bool commandInQueue = false;


char receivedString[128];
char chkGS[3] = "GS";

typedef struct {
  uint8_t BUFFER[512] = {0};
  bool isEmpty = true;
} queueItem;

queueItem queueItems[22];

typedef struct {
  uint8_t BUFFER[6] = {0};
  bool isEmpty = true;
} commandItem;

commandItem commandRXItems[5];

struct StoreStruct {
 byte chkDigit;
    bool isAccesPoint;
 char SSID[25];
 char password[25];
 int ipPort;
 int i2sSpeed;
    bool isTX;
    bool useInternalADC;
    char otherMac[18];
};

// const char* ssid = "MARODEKWiFi";
// const char* password = "MAROWiFi19052004!";

StoreStruct storage = {
  '#',
        false,
  "RDKMobile",
  "0919932003",
        3005,
        11025,
        false,
        false,
        "80:7D:3A:DA:B4:0C"
};

IPAddress myIP(0,0,0,0);
IPAddress remoteIP(0,0,0,0);

volatile byte bufferRX = 0;
volatile byte bufferTX = 0;
volatile byte bufferTX_CMD = 0;
volatile byte bufferRX_CMD = 0;

WiFiUDP udp;


SSD1306 lcd(0x3C /* Default 0x3C for 0.9", for 1.3" it is 0x78*/, 4 /* GPIO4*/, 15 /* GPIO15*/);// i2c ADDR & SDA, SCL on wemos


void setup()
{

 pinMode(16,0x02);
 digitalWrite(16, 0x0); // low to reset OLED
 delay(50);
 digitalWrite(16, 0x1); // must be high to turn on OLED
 delay(50);
 Wire.begin(4, 15);



 lcd.init();
 lcd.flipScreenVertically();
 lcd.setFont(ArialMT_Plain_10);
 lcd.setTextAlignment(TEXT_ALIGN_LEFT);
 lcd.drawString(0, 4, "WiFi PORTO");
 lcd.drawString(0, 12, "STARTING");
 lcd.display();
 lcd.setFont(ArialMT_Plain_10);
    delay(1000);


    COMMANDO[0] = 0;
    pinMode(23,0x02);
    digitalWrite(23,0x1);
    pinMode(18,0x02);
    digitalWrite(18,0x1);
    pinMode(39,0x05);
    pinMode(5,0x02);
    digitalWrite(5,0x1);
    pinMode(14,0x02);
 digitalWrite(14, 0x1); // High to activate amplifier

    for (int i =0;i<5;i++){
        digitalWrite(23,0);
        delay(100);
        digitalWrite(23,1);
        delay(100);
    }
    Serial.begin(115200);

    if (!EEPROM.begin(200))
 {
  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("failed to initialise EEPROM")))));
  while(1);
 }
 if (EEPROM.read(0x10) != storage.chkDigit){
  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Writing defaults....")))));
  saveConfig();
 }
 loadConfig();
 printConfig();

    Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Type GS to enter setup:")))));
 delay(5000);
 if (Serial.available()) {
  Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Check for setup")))));
  if (Serial.find(chkGS)) {
   Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Setup entered...")))));
   setSettings(1);
   delay(2000);
  }
 }

    if (storage.isAccesPoint){
        WiFi.softAP(storage.SSID, storage.password);
        myIP = WiFi.softAPIP();
        Serial.print("Access point"); Serial.println(myIP);
        isConnected = true;
    } else {
        check_connection();
    }

    Serial.println(WiFi.macAddress());
    i2sInit_rx();
    if (storage.useInternalADC){
        i2sInit_txa();
    } else {
        i2sInit_txm();
    }

    xTaskCreate(ReceiveI2SData, "ReceiveI2SData", 2048, 
# 185 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                       __null
# 185 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                           , 1, 
# 185 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                                __null
# 185 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                                    );
    xTaskCreate(sendI2SData, "sendI2SData", 2048, 
# 186 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                 __null
# 186 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                     , 1, 
# 186 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                          __null
# 186 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                              );
    xTaskCreate(processUDPData, "processUDPData", 2048, 
# 187 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                       __null
# 187 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                           , 1, 
# 187 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                                __null
# 187 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                                    );
    xTaskCreate(processIncomingCommand, "processIncomingCommand", 2048, 
# 188 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                                       __null
# 188 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                                           , 1, 
# 188 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                                                __null
# 188 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                                                    );
    xTaskCreate(processOutgoingCommand, "processOutgoingCommand", 2048, 
# 189 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                                       __null
# 189 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                                           , 2, 
# 189 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                                                __null
# 189 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                                                    );
}

void loop(){
  delay(500);
  if (!storage.isAccesPoint) check_connection();

    if (isConnected){
        digitalWrite(5, 0x0);
    } else {
        digitalWrite(5, 0x1);
    }



    lcd.clear();
 lcd.drawString(0, 4, "WiFi PORTO");
 lcd.drawString(0, 12, "        ");
    if (isConnected){
        lcd.drawString(0, 20, "Connected    ");
    } else {
        lcd.drawString(0, 20, "Not connected");
    }

    if (activePTT){
        lcd.drawString(0, 28, "Transmitting");
    } else {
        lcd.drawString(0, 28, "Receiving   ");
    }

    if (remotePTT){
        lcd.drawString(0, 36, "Remote PTT");
    } else {
        lcd.drawString(0, 36, "          ");
    }

    lcd.drawString(0, 44, myIP.toString());
    lcd.drawString(0, 52, remoteIP.toString());

 lcd.display();
 lcd.setFont(ArialMT_Plain_10);
    delay(1000);


}

boolean check_connection() {
 if (WiFi.status() !=WL_CONNECTED) {
  InitConnection();
 }
 return (WiFi.status() == WL_CONNECTED);
}

void InitConnection() {
 Serial.print("++++++++++++++"); Serial.println("Initialize connection");

 if (WiFi.status() != WL_CONNECTED){
        isConnected = false;
  Serial.println("Connecting to WiFi");
  WlanReset();
  WiFi.begin(storage.SSID,storage.password);
  int agains=1;
  while ((WiFi.status() != WL_CONNECTED) && (agains < 20)){
   Serial.print(".");
   delay(1000);
   agains++;
  }
 }
    if (WiFi.status() == WL_CONNECTED){
        printWifiStatus();
        Serial.println("Connected to wifi");
        isConnected = true;
    }
}

void WlanReset() {

    if (CommunicationStarted){
        udp.stop();
        CommunicationStarted = false;
    }

 WiFi.persistent(false);
 WiFi.disconnect();
 WiFi.mode(WIFI_MODE_NULL);
 WiFi.mode(WIFI_MODE_STA);
 delay(1000);
}

void processOutgoingCommand (void* arg)
{
    Serial.println("Outgoing commands task started");
    int i=0;
    int j=0;
    int lastPTT = false;
    while(1){
        if (i==0){
            Serial.println("Broadcast IP");
            j=0;
            while (commandInQueue){
                vTaskDelay(100 / ( ( TickType_t ) 1000 / ( 1000 ) ));
            }
            COMMANDO[j++] = 0x18;
            COMMANDO[j++] = 0x81;
            COMMANDO[j++] = myIP[0];
            COMMANDO[j++] = myIP[1];
            COMMANDO[j++] = myIP[2];
            COMMANDO[j++] = myIP[3];
            commandInQueue = true;
        }

        lastPTT = !digitalRead(39);
        if (storage.isTX){
            lastPTT = true;
            if (remotePTT) lastPTT = false;
        }

        if ((activePTT != lastPTT) || i==0){
            Serial.println("Broadcast PTT");
            j = 0;
            while (commandInQueue){
                vTaskDelay(100 / ( ( TickType_t ) 1000 / ( 1000 ) ));
            }
            COMMANDO[j++] = 0x16;
            COMMANDO[j++] = 0x61;
            COMMANDO[j++] = 0;
            COMMANDO[j++] = 0;
            COMMANDO[j++] = 0;
            COMMANDO[j++] = lastPTT;
            commandInQueue = true;
            activePTT = lastPTT;
        }
        i++;
        if (i==50) i=0;
        vTaskDelay(100 / ( ( TickType_t ) 1000 / ( 1000 ) ));
    }
}

void ReceiveI2SData (void* arg)
{
    Serial.println("Receive I2S Data task started");
    uint32_t bytesWritten=0;
    bool playAudio = true;
    bool playEnabled = true;
    int bufferLoad = 0;
    bool hasSkipped = false;
    while(1){
        if (activePTT){
            if (playEnabled){
                i2s_stop(I2S_NUM_1);
                playEnabled = false;
                clearBuffers();
            }
            vTaskDelay(10 / ( ( TickType_t ) 1000 / ( 1000 ) )); //was 10!!!
        } else {
            if (!playEnabled){
                i2s_start(I2S_NUM_1);
                playEnabled = true;
                playAudio = true;
                clearBuffers();
            }
            bufferLoad = bufferRX-bufferTX;
            if (bufferLoad<0) bufferLoad = bufferLoad+22;
            if (bufferLoad==0 && playAudio){
                if (i2sDebug) Serial.println("Audio stopped");
                i2s_stop(I2S_NUM_1);
                playAudio = false;
                clearBuffers();
                vTaskDelay(100 / ( ( TickType_t ) 1000 / ( 1000 ) ));
            }
            if (bufferLoad > (22 -5) && !playAudio){
                if (i2sDebug) Serial.println("Audio restarted");
                i2s_start(I2S_NUM_1);
                playAudio = true;
            }

            if (playAudio){
                if (bufferLoad<22 -5 && !hasSkipped){
                    if (i2sDebug) Serial.println("Skip play to fill buffer");
                    hasSkipped = true;
                }
                else
                {
                    hasSkipped = false;
                    if (!queueItems[bufferTX].isEmpty){
                        queueItems[bufferTX].isEmpty = true;
                        bytesWritten=0;
                        i2s_write(I2S_NUM_1,queueItems[bufferTX].BUFFER, 512, &bytesWritten, ( TickType_t ) 0xffffffffUL);
                        if (i2sDebug) Serial.print("-> TX:");
                        if (i2sDebug) Serial.println(bufferTX);
                        bufferTX++;
                        if (bufferTX==22) bufferTX = 0;
                    }
                }
            }
        }
        vTaskDelay(5 /*depends on READ_LEN, READ_LEN 1024 = STD_DELAY 20*/ / ( ( TickType_t ) 1000 / ( 1000 ) ));
    }
}

void processUDPData (void* arg)
{
    Serial.println("Process UDP data task started");
    int noBytes = 0;
    while(1){
        digitalWrite(23,!activePTT);
        digitalWrite(18,activePTT);
        digitalWrite(14,!activePTT);
        if (isConnected){
            if (!CommunicationStarted){
                Serial.print("UDP server started at port ");
                Serial.println(storage.ipPort);
                udp.begin(storage.ipPort);
                CommunicationStarted = true;
            }
            noBytes = udp.parsePacket();
            if (noBytes == 512) {
                if (!activePTT){
                    digitalWrite(23,0);
                    udp.read(queueItems[bufferRX].BUFFER,noBytes);
                    queueItems[bufferRX].isEmpty = false;
                    if (UDPDebug) Serial.print(">- RX:");
                    if (UDPDebug) Serial.println(bufferRX);
                    bufferRX++;
                    if (bufferRX==22) bufferRX = 0;
                    digitalWrite(23,1);
                }
            }
            else if (noBytes>0) {
                if (commandRXItems[bufferRX_CMD].isEmpty){
                    udp.read(commandRXItems[bufferRX_CMD].BUFFER,noBytes);
                    commandRXItems[bufferRX_CMD].isEmpty = false;
                    bufferRX_CMD++;
                    if (bufferRX_CMD==5) bufferRX_CMD=0;
                }
            } else {
                vTaskDelay(5 / ( ( TickType_t ) 1000 / ( 1000 ) ));
            }
            udp.flush();

            if (commandInQueue){
                IPAddress broadcastIp(myIP[0],myIP[1],myIP[2],255);
                udp.beginPacket(broadcastIp,storage.ipPort);
                for (int i=0;i<6;i++){
                    if (COMMANDO[i] == 0x81){
                        if (UDPDebug) Serial.println("Send command IP");
                    }
                    if (COMMANDO[i] == 0x61){
                        if (UDPDebug) Serial.println("Send command PTT");
                    }
                    udp.write(COMMANDO[i]);
                }
                udp.endPacket();
                commandInQueue = false;
            }
        }

        if (activePTT){
            if (remoteIP[0]!=0){
                if (!queueItems[0].isEmpty){
                    udp.beginPacket(remoteIP,storage.ipPort);
                    udp.write(queueItems[bufferTX].BUFFER,512);
                    udp.endPacket();
                    queueItems[0].isEmpty = true;
                }
            }
        }
        vTaskDelay(5 /*depends on READ_LEN, READ_LEN 1024 = STD_DELAY 20*/ / ( ( TickType_t ) 1000 / ( 1000 ) ));
    }
}

void clearBuffers(){
    for (int i=0;i<22;i++) {queueItems[i].isEmpty = true;}
    bufferRX = 0;
    bufferTX = 0;
}

void sendI2SData (void* arg)
{
    Serial.println("Send I2S Data task started");
    size_t bytesRead;
    int i = 0;
    bool playEnabled = true;
    while(1){
        if (!activePTT){
            if (playEnabled){
                i2s_stop(I2S_NUM_0);
                playEnabled = false;
                clearBuffers();
            }
            vTaskDelay(10 / ( ( TickType_t ) 1000 / ( 1000 ) )); //was 10!!!
        }
        else
        {
            if (!playEnabled){
                i2s_start(I2S_NUM_0);
                playEnabled = true;
                clearBuffers();
            }
            bytesRead = 0;
            while (bytesRead == 0){
                i2s_read(I2S_NUM_0,(char*) queueItems[bufferTX].BUFFER, 512, &bytesRead, ( TickType_t ) 0xffffffffUL); //(5 / portTICK_RATE_MS)
            }
            if (bytesRead==512){
                queueItems[0].isEmpty = false;
                bufferTX++;
                if (bufferTX==22) bufferTX = 0;
            }

            i++;
            Serial.print(" "); Serial.print(bytesRead);
            if (i%32==0) Serial.println();
            vTaskDelay(5 /*depends on READ_LEN, READ_LEN 1024 = STD_DELAY 20*/ / ( ( TickType_t ) 1000 / ( 1000 ) ));
        }
    }
}

void processIncomingCommand(void* arg)
{
    Serial.println("Incoming commands task started");
    while(1){
        for (int i=0; i<5; i++){
            if (!commandRXItems[i].isEmpty){
                Serial.println("Received command");
                if (commandRXItems[i].BUFFER[0]==0x18 && commandRXItems[i].BUFFER[1]==0x81){
                    Serial.print("Received host IP Address:");
                    remoteIP = {commandRXItems[i].BUFFER[2],commandRXItems[i].BUFFER[3],commandRXItems[i].BUFFER[4],commandRXItems[i].BUFFER[5]};
                    Serial.println(remoteIP);
                }
                if (commandRXItems[i].BUFFER[0]==0x16 && commandRXItems[i].BUFFER[1]==0x61){
                    Serial.print("Received PTT Command:");
                    if (commandRXItems[i].BUFFER[5]) Serial.println(" On"); else Serial.println(" Off");
                    remotePTT = commandRXItems[i].BUFFER[5];
                }
                commandRXItems[i].isEmpty = true;
                break;
            }
        }
        vTaskDelay(100 / ( ( TickType_t ) 1000 / ( 1000 ) ));
    }
}

void i2sInit_rx()
{
    Serial.println("Init i2s RX");
    i2s_config_t i2s_config_rx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = storage.i2sSpeed,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = (1<<1) /*|< Accept a Level 1 interrupt vector (lowest priority)*/,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
    };

    i2s_pin_config_t pin_config_rx;
    pin_config_rx.bck_io_num = 12;
    pin_config_rx.ws_io_num = 25;
    pin_config_rx.data_out_num = 22;
    pin_config_rx.data_in_num = (-1) /*!< Use in i2s_pin_config_t for pins which should not be changed */;

    i2s_driver_install(I2S_NUM_1, &i2s_config_rx, 0, 
# 551 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                    __null
# 551 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                        );
    i2s_set_pin(I2S_NUM_1, &pin_config_rx);
    i2s_set_clk(I2S_NUM_1, storage.i2sSpeed, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void i2sInit_txm()
{
    Serial.println("Init i2s TX Mike");
    i2s_config_t i2s_config_txm = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = storage.i2sSpeed,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = (1<<1) /*|< Accept a Level 1 interrupt vector (lowest priority)*/,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = true
    };

    i2s_pin_config_t pin_config_txm;
    pin_config_txm.bck_io_num = 2;
    pin_config_txm.ws_io_num = 17;
    pin_config_txm.data_out_num = (-1) /*!< Use in i2s_pin_config_t for pins which should not be changed */;
    pin_config_txm.data_in_num = 13;

    i2s_driver_install(I2S_NUM_0, &i2s_config_txm, 0, 
# 577 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                     __null
# 577 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                         );
    i2s_set_pin(I2S_NUM_0, &pin_config_txm);
    i2s_set_clk(I2S_NUM_0, storage.i2sSpeed, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    i2s_stop(I2S_NUM_0);
}

void i2sInit_txa(){
    Serial.println("Init i2s TX ADC");
    i2s_config_t i2s_config_txa = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = storage.i2sSpeed,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = (1<<1) /*|< Accept a Level 1 interrupt vector (lowest priority)*/,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
    };

    //    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);
    //    adc1_config_width(ADC_WIDTH_12Bit);

    i2s_pin_config_t pin_config_txa;
    pin_config_txa.bck_io_num = 2;
    pin_config_txa.ws_io_num = 17;
    pin_config_txa.data_out_num = (-1) /*!< Use in i2s_pin_config_t for pins which should not be changed */;
    pin_config_txa.data_in_num = 13;

    i2s_driver_install(I2S_NUM_0, &i2s_config_txa, 0, 
# 605 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino" 3 4
                                                     __null
# 605 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
                                                         );
    //i2s_set_pin(I2S_NUM_0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config_txa);
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
    i2s_set_clk(I2S_NUM_0, storage.i2sSpeed, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    i2s_stop(I2S_NUM_0);
}

void printWifiStatus() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    myIP = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(myIP);
}

void saveConfig() {
 for (unsigned int t = 0; t < sizeof(storage); t++)
  EEPROM.write(0x10 + t, *((char*)&storage + t));
 EEPROM.commit();
}

void loadConfig() {
 if (EEPROM.read(0x10 + 0) == storage.chkDigit)
  for (unsigned int t = 0; t < sizeof(storage); t++)
   *((char*)&storage + t) = EEPROM.read(0x10 + t);
}

void printConfig() {
 if (EEPROM.read(0x10 + 0) == storage.chkDigit){
  for (unsigned int t = 0; t < sizeof(storage); t++)
   Serial.write(EEPROM.read(0x10 + t));
  Serial.println();
  setSettings(0);
 }
}

void setSettings(bool doAsk) {
 int i = 0;

 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Is access point (0=no/1=yes) (")))));
 if (storage.isAccesPoint == 0) {
  Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("no")))));
 } else {
  Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Yes")))));
 }
 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("): ")))));
 if (doAsk == 1) {
  i = getNumericValue();
  if (receivedString[0] != 0) storage.isAccesPoint = i;
 }
 Serial.println();

 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("SSID (")))));
 Serial.print(storage.SSID);
 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("):")))));
 if (doAsk == 1) {
  getStringValue(26);
  if (receivedString[0] != 0) {
   storage.SSID[0] = 0;
   strcat(storage.SSID, receivedString);
  }
 }
 Serial.println();

 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Password (")))));
 Serial.print(storage.password);
 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("):")))));
 if (doAsk == 1) {
  getStringValue(26);
  if (receivedString[0] != 0) {
   storage.password[0] = 0;
   strcat(storage.password, receivedString);
  }
 }
 Serial.println();

    Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("IP Port (")))));
 Serial.print(storage.ipPort);
 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("):")))));
 if (doAsk == 1) {
  i = getNumericValue();
  if (receivedString[0] != 0) storage.ipPort = i;
 }
 Serial.println();

    Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("I2S Speed (")))));
 Serial.print(storage.i2sSpeed);
 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("):")))));
 if (doAsk == 1) {
  i = getNumericValue();
  if (receivedString[0] != 0) storage.i2sSpeed = i;
 }
 Serial.println();

 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Auto TX (0=no/1=yes) (")))));
 if (storage.isTX == 0) {
  Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("no")))));
 } else {
  Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Yes")))));
 }
 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("): ")))));
 if (doAsk == 1) {
  i = getNumericValue();
  if (receivedString[0] != 0) storage.isTX = i;
 }
 Serial.println();

    Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Use internal ADC (0=no/1=yes) (")))));
 if (storage.useInternalADC == 0) {
  Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("no")))));
 } else {
  Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Yes")))));
 }
 Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("): ")))));
 if (doAsk == 1) {
  i = getNumericValue();
  if (receivedString[0] != 0) storage.useInternalADC = i;
 }
 Serial.println();

 if (doAsk == 1) {
  saveConfig();
  loadConfig();
 }
}

void getStringValue(int length) {
 serialFlush();
 receivedString[0] = 0;
 int i = 0;
 while (receivedString[i] != 13 && i < length) {
  if (Serial.available() > 0) {
   receivedString[i] = Serial.read();
   if (receivedString[i] == 13 || receivedString[i] == 10) {
    i--;
   }
   else {
    Serial.write(receivedString[i]);
   }
   i++;
  }
 }
 receivedString[i] = 0;
 serialFlush();
}

byte getCharValue() {
 serialFlush();
 receivedString[0] = 0;
 int i = 0;
 while (receivedString[i] != 13 && i < 2) {
  if (Serial.available() > 0) {
   receivedString[i] = Serial.read();
   if (receivedString[i] == 13 || receivedString[i] == 10) {
    i--;
   }
   else {
    Serial.write(receivedString[i]);
   }
   i++;
  }
 }
 receivedString[i] = 0;
 serialFlush();
 return receivedString[i - 1];
}

int getNumericValue() {
 serialFlush();
 int myByte = 0;
 byte inChar = 0;
 bool isNegative = false;
 receivedString[0] = 0;

 int i = 0;
 while (inChar != 13) {
  if (Serial.available() > 0) {
   inChar = Serial.read();
   if (inChar > 47 && inChar < 58) {
    receivedString[i] = inChar;
    i++;
    Serial.write(inChar);
    myByte = (myByte * 10) + (inChar - 48);
   }
   if (inChar == 45) {
    Serial.write(inChar);
    isNegative = true;
   }
  }
 }
 receivedString[i] = 0;
 if (isNegative == true) myByte = myByte * -1;
 serialFlush();
 return myByte;
}

void serialFlush() {
 for (int i = 0; i < 10; i++)
 {
  while (Serial.available() > 0) {
   Serial.read();
  }
 }
}
