#include <Arduino.h>
#line 1 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
#include <driver/i2s.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include "EEPROM.h"
#include "Wire.h"
#include "SSD1306.h"

#define RXPIN_WS  25
#define RXPIN_DATA 22
#define RXPIN_SCK 12

#define TXPIN_WS  17
#define TXPIN_DATA 13
#define TXPIN_SCK 2
#define CONN_LED 5
#define STAT_LED 23
#define PTT_OUT 18
#define PTT 39
#define MUTE 14

#define OLED_SCL	15			// GPIO15
#define OLED_SDA	4			// GPIO4
#define OLED_RST	16
#define OLED_ADR	0x3C		// Default 0x3C for 0.9", for 1.3" it is 0x78
#define hasLCD

#define offsetEEPROM       0x10
#define EEPROM_SIZE        200

#define READ_LEN 512
#define STD_DELAY 5     //depends on READ_LEN, READ_LEN 1024 = STD_DELAY 20
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
  uint8_t BUFFER[READ_LEN] = {0};
  bool isEmpty = true;
} queueItem;

queueItem queueItems[bufferQty];

typedef struct {
  uint8_t BUFFER[6] = {0};
  bool isEmpty = true;  
} commandItem;

commandItem commandRXItems[commandQty];

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

#ifdef hasLCD
SSD1306  lcd(OLED_ADR, OLED_SDA, OLED_SCL);// i2c ADDR & SDA, SCL on wemos
#endif

#line 103 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void setup();
#line 192 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void loop();
#line 235 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
boolean check_connection();
#line 242 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void InitConnection();
#line 264 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void WlanReset();
#line 278 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void processOutgoingCommand(void* arg);
#line 327 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void ReceiveI2SData(void* arg);
#line 389 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void processUDPData(void* arg);
#line 460 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void clearBuffers();
#line 466 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void sendI2SData(void* arg);
#line 506 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void processIncomingCommand(void* arg);
#line 531 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void i2sInit_rx();
#line 556 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void i2sInit_txm();
#line 583 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void i2sInit_txa();
#line 613 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void printWifiStatus();
#line 622 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void saveConfig();
#line 628 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void loadConfig();
#line 634 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void printConfig();
#line 643 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void setSettings(bool doAsk);
#line 733 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void getStringValue(int length);
#line 753 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
byte getCharValue();
#line 774 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
int getNumericValue();
#line 803 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void serialFlush();
#line 103 "/Users/robertdekok/Dropbox/Arduino-workspace/DigitalMike/DigitalMike.ino"
void setup()
{
    #ifdef hasLCD
	pinMode(OLED_RST,OUTPUT);
	digitalWrite(OLED_RST, LOW); // low to reset OLED
	delay(50);
	digitalWrite(OLED_RST, HIGH); // must be high to turn on OLED
	delay(50);
	Wire.begin(4, 15);
    #endif

    #ifdef hasLCD
	lcd.init();
	lcd.flipScreenVertically();
	lcd.setFont(ArialMT_Plain_10);
	lcd.setTextAlignment(TEXT_ALIGN_LEFT);
	lcd.drawString(0, 4, "WiFi PORTO");
	lcd.drawString(0, 12, "STARTING");
	lcd.display();
	lcd.setFont(ArialMT_Plain_10);
    delay(1000);
	#endif

    COMMANDO[0] = 0;
    pinMode(STAT_LED,OUTPUT);
    digitalWrite(STAT_LED,HIGH);
    pinMode(PTT_OUT,OUTPUT);
    digitalWrite(PTT_OUT,HIGH);
    pinMode(PTT,INPUT_PULLUP);
    pinMode(CONN_LED,OUTPUT);
    digitalWrite(CONN_LED,HIGH);
    pinMode(MUTE,OUTPUT);
	digitalWrite(MUTE, HIGH); // High to activate amplifier

    for (int i =0;i<5;i++){
        digitalWrite(STAT_LED,0);
        delay(100);
        digitalWrite(STAT_LED,1);
        delay(100);
    }
    Serial.begin(115200);

    if (!EEPROM.begin(EEPROM_SIZE))
	{
		Serial.println(F("failed to initialise EEPROM"));
		while(1); 
	}
	if (EEPROM.read(offsetEEPROM) != storage.chkDigit){
		Serial.println(F("Writing defaults...."));
		saveConfig();
	}
	loadConfig();
	printConfig();

    Serial.println(F("Type GS to enter setup:"));
	delay(5000);
	if (Serial.available()) {
		Serial.println(F("Check for setup"));
		if (Serial.find(chkGS)) {
			Serial.println(F("Setup entered..."));
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

    xTaskCreate(ReceiveI2SData, "ReceiveI2SData", 2048, NULL, 1, NULL);
    xTaskCreate(sendI2SData, "sendI2SData", 2048, NULL, 1, NULL);
    xTaskCreate(processUDPData, "processUDPData", 2048, NULL, 1, NULL);
    xTaskCreate(processIncomingCommand, "processIncomingCommand", 2048, NULL, 1, NULL);    
    xTaskCreate(processOutgoingCommand, "processOutgoingCommand", 2048, NULL, 2, NULL);
}

void loop(){
  delay(500);
  if (!storage.isAccesPoint) check_connection();

    if (isConnected){
        digitalWrite(CONN_LED, LOW);
    } else {
        digitalWrite(CONN_LED, HIGH);
    }


    #ifdef hasLCD
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
	#endif

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
	WiFi.mode(WIFI_OFF);
	WiFi.mode(WIFI_STA);
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
                vTaskDelay(100 / portTICK_RATE_MS);
            }
            COMMANDO[j++] = 0x18;
            COMMANDO[j++] = 0x81;
            COMMANDO[j++] = myIP[0];   
            COMMANDO[j++] = myIP[1]; 
            COMMANDO[j++] = myIP[2];   
            COMMANDO[j++] = myIP[3]; 
            commandInQueue = true;
        }

        lastPTT = !digitalRead(PTT);
        if (storage.isTX){
            lastPTT = true;
            if (remotePTT) lastPTT = false;
        }

        if ((activePTT != lastPTT) || i==0){
            Serial.println("Broadcast PTT");
            j = 0;
            while (commandInQueue){
                vTaskDelay(100 / portTICK_RATE_MS);
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
        vTaskDelay(100 / portTICK_RATE_MS);
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
            vTaskDelay(10 / portTICK_RATE_MS); //was 10!!!
        } else {
            if (!playEnabled){
                i2s_start(I2S_NUM_1);
                playEnabled = true;
                playAudio = true;
                clearBuffers();
            }
            bufferLoad = bufferRX-bufferTX;
            if (bufferLoad<0) bufferLoad = bufferLoad+bufferQty;
            if (bufferLoad==0 && playAudio){
                if (i2sDebug) Serial.println("Audio stopped");
                i2s_stop(I2S_NUM_1);
                playAudio = false;
                clearBuffers();
                vTaskDelay(100 / portTICK_RATE_MS);
            }
            if (bufferLoad > (bufferQty-5) && !playAudio){
                if (i2sDebug) Serial.println("Audio restarted");
                i2s_start(I2S_NUM_1);
                playAudio = true;
            }

            if (playAudio){
                if (bufferLoad<bufferQty-5 && !hasSkipped){
                    if (i2sDebug) Serial.println("Skip play to fill buffer");
                    hasSkipped = true;
                }
                else
                {
                    hasSkipped = false;
                    if (!queueItems[bufferTX].isEmpty){
                        queueItems[bufferTX].isEmpty = true;
                        bytesWritten=0;
                        i2s_write(I2S_NUM_1,queueItems[bufferTX].BUFFER, READ_LEN, &bytesWritten, portMAX_DELAY); 
                        if (i2sDebug) Serial.print("-> TX:"); 
                        if (i2sDebug) Serial.println(bufferTX);
                        bufferTX++;
                        if (bufferTX==bufferQty) bufferTX = 0;
                    } 
                }
            }
        }
        vTaskDelay(STD_DELAY / portTICK_RATE_MS);
    }
}

void processUDPData (void* arg)
{
    Serial.println("Process UDP data task started");
    int noBytes = 0;
    while(1){    
        digitalWrite(STAT_LED,!activePTT);
        digitalWrite(PTT_OUT,activePTT);
        digitalWrite(MUTE,!activePTT);
        if (isConnected){    
            if (!CommunicationStarted){
                Serial.print("UDP server started at port ");
                Serial.println(storage.ipPort);
                udp.begin(storage.ipPort);
                CommunicationStarted = true;
            }
            noBytes = udp.parsePacket();
            if (noBytes == READ_LEN) {
                if (!activePTT){
                    digitalWrite(STAT_LED,0);
                    udp.read(queueItems[bufferRX].BUFFER,noBytes);
                    queueItems[bufferRX].isEmpty = false;
                    if (UDPDebug) Serial.print(">- RX:"); 
                    if (UDPDebug) Serial.println(bufferRX);
                    bufferRX++;
                    if (bufferRX==bufferQty) bufferRX = 0;
                    digitalWrite(STAT_LED,1);
                }
            }
            else if (noBytes>0) {
                if (commandRXItems[bufferRX_CMD].isEmpty){
                    udp.read(commandRXItems[bufferRX_CMD].BUFFER,noBytes);
                    commandRXItems[bufferRX_CMD].isEmpty = false;
                    bufferRX_CMD++;
                    if (bufferRX_CMD==commandQty) bufferRX_CMD=0;
                }
            } else {
                vTaskDelay(5 / portTICK_RATE_MS);
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
                    udp.write(queueItems[bufferTX].BUFFER,READ_LEN);
                    udp.endPacket();
                    queueItems[0].isEmpty = true;
                }
            } 
        }
        vTaskDelay(STD_DELAY / portTICK_RATE_MS);
    }
}

void clearBuffers(){
    for (int i=0;i<bufferQty;i++) {queueItems[i].isEmpty = true;}
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
            vTaskDelay(10 / portTICK_RATE_MS); //was 10!!!
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
                i2s_read(I2S_NUM_0,(char*) queueItems[bufferTX].BUFFER, READ_LEN, &bytesRead, portMAX_DELAY); //(5 / portTICK_RATE_MS)
            }
            if (bytesRead==READ_LEN){
                queueItems[0].isEmpty = false;
                bufferTX++;
                if (bufferTX==bufferQty) bufferTX = 0;
            }

            i++;
            Serial.print(" "); Serial.print(bytesRead);
            if (i%32==0) Serial.println();       
            vTaskDelay(STD_DELAY / portTICK_RATE_MS);   
        } 
    }
}

void processIncomingCommand(void* arg)
{
    Serial.println("Incoming commands task started");
    while(1){
        for (int i=0; i<commandQty; i++){
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
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void i2sInit_rx()
{
    Serial.println("Init i2s RX");
    i2s_config_t i2s_config_rx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate =  storage.i2sSpeed,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, 
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
    };

    i2s_pin_config_t pin_config_rx;
    pin_config_rx.bck_io_num   = RXPIN_SCK;
    pin_config_rx.ws_io_num    = RXPIN_WS;
    pin_config_rx.data_out_num = RXPIN_DATA;
    pin_config_rx.data_in_num  = I2S_PIN_NO_CHANGE;

    i2s_driver_install(I2S_NUM_1, &i2s_config_rx, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pin_config_rx);
    i2s_set_clk(I2S_NUM_1, storage.i2sSpeed, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void i2sInit_txm()
{
    Serial.println("Init i2s TX Mike");
    i2s_config_t i2s_config_txm = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate =  storage.i2sSpeed,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = true
    };

    i2s_pin_config_t pin_config_txm;
    pin_config_txm.bck_io_num   = TXPIN_SCK;
    pin_config_txm.ws_io_num    = TXPIN_WS;
    pin_config_txm.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config_txm.data_in_num  = TXPIN_DATA;

    i2s_driver_install(I2S_NUM_0, &i2s_config_txm, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config_txm);
    i2s_set_clk(I2S_NUM_0, storage.i2sSpeed, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    i2s_stop(I2S_NUM_0);
}

void i2sInit_txa(){
    Serial.println("Init i2s TX ADC");
    i2s_config_t i2s_config_txa = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate =  storage.i2sSpeed,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
    };

    //    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);
    //    adc1_config_width(ADC_WIDTH_12Bit);

    i2s_pin_config_t pin_config_txa;
    pin_config_txa.bck_io_num   = TXPIN_SCK;
    pin_config_txa.ws_io_num    = TXPIN_WS;
    pin_config_txa.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config_txa.data_in_num  = TXPIN_DATA;

    i2s_driver_install(I2S_NUM_0, &i2s_config_txa, 0, NULL);
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
		EEPROM.write(offsetEEPROM + t, *((char*)&storage + t));
	EEPROM.commit();
}

void loadConfig() {
	if (EEPROM.read(offsetEEPROM + 0) == storage.chkDigit)
		for (unsigned int t = 0; t < sizeof(storage); t++)
			*((char*)&storage + t) = EEPROM.read(offsetEEPROM + t);
}

void printConfig() {
	if (EEPROM.read(offsetEEPROM + 0) == storage.chkDigit){
		for (unsigned int t = 0; t < sizeof(storage); t++)
			Serial.write(EEPROM.read(offsetEEPROM + t));
		Serial.println();
		setSettings(0);
	}
}

void setSettings(bool doAsk) {
	int i = 0;

	Serial.print(F("Is access point (0=no/1=yes) ("));
	if (storage.isAccesPoint == 0) {
		Serial.print(F("no"));
	} else {
		Serial.print(F("Yes"));
	}
	Serial.print(F("): "));
	if (doAsk == 1) {
		i = getNumericValue();
		if (receivedString[0] != 0) storage.isAccesPoint = i;
	}
	Serial.println();

	Serial.print(F("SSID ("));
	Serial.print(storage.SSID);
	Serial.print(F("):"));
	if (doAsk == 1) {
		getStringValue(26);
		if (receivedString[0] != 0) {
			storage.SSID[0] = 0;
			strcat(storage.SSID, receivedString);
		}
	}
	Serial.println();

	Serial.print(F("Password ("));
	Serial.print(storage.password);
	Serial.print(F("):"));
	if (doAsk == 1) {
		getStringValue(26);
		if (receivedString[0] != 0) {
			storage.password[0] = 0;
			strcat(storage.password, receivedString);
		}
	}
	Serial.println();

    Serial.print(F("IP Port ("));
	Serial.print(storage.ipPort);
	Serial.print(F("):"));
	if (doAsk == 1) {
		i = getNumericValue();
		if (receivedString[0] != 0) storage.ipPort = i;
	}
	Serial.println();

    Serial.print(F("I2S Speed ("));
	Serial.print(storage.i2sSpeed);
	Serial.print(F("):"));
	if (doAsk == 1) {
		i = getNumericValue();
		if (receivedString[0] != 0) storage.i2sSpeed = i;
	}
	Serial.println();

	Serial.print(F("Auto TX (0=no/1=yes) ("));
	if (storage.isTX == 0) {
		Serial.print(F("no"));
	} else {
		Serial.print(F("Yes"));
	}
	Serial.print(F("): "));
	if (doAsk == 1) {
		i = getNumericValue();
		if (receivedString[0] != 0) storage.isTX = i;
	}
	Serial.println();

    Serial.print(F("Use internal ADC (0=no/1=yes) ("));
	if (storage.useInternalADC == 0) {
		Serial.print(F("no"));
	} else {
		Serial.print(F("Yes"));
	}
	Serial.print(F("): "));
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
