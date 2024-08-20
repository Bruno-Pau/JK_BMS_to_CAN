//-----------------------------------------------------------------------------
//  JkBms_Sofar5.ino         B.Vannier   26 fev 2023
//-----------------------------------------------------------------------------
//  Version ESP32
//  Interrogation d'un BMS JK en RS et Emulation d'un BMS PYLONTECH en CAN
//
// Cette version interroge directement le BMS en RS232 TTL
// Declanche un defaut d'isolation dans l'onduleur Sofar si la RS n'est pas isolee
//
// Les infos BMS sont renvoyees en UDP (UDP_PORT) et TCP (TCP_PORT) sur le reseau
//
//-----------------------------------------------------------------------------
// Modifs
//  03/02/23 : la gestion du Wifi et de TCP passent en thread 
//  26/02/23 : V4 version debug bufin 
//  27/02/23 : correction bug tBms.sPower = fPower; et non tBms.sPower = (short)fPower;
//  28/02/23 : correction Bug JK tBms.sCurrent
//  29/06/23 : V5.0 Envoi vers CAN seulement si analyse() OK
/*------------------------------------
Cablage 
MPC2515         ESP32

MPC2515 - VCC   +5V
MPC2515 - GND   GND
MPC2515 - CS    GPIO5 (VSPI_SS)
MPC2515 - SO    GPIO19 (VSPI_MISO)
MPC2515 - SI    GPIO23(VSPI_MOSI) 
MPC2515 - SCK   GPIO18 (VSPI_SCK)
MPC2515 - INT   GPIO26

            Bat 3.3v
   UART-TTL    │                  isolateur
┌──────────┐   │                 ┌──────────┐         ┌───────────────────┐
│          │   └─────── rouge ──>│ V2    V1 │<--------│ 3.3V              │
│       Tx │------ TX noir ----->│ AI    AO │-------->│ GPIO17 (`rx_pin`) │
│       Rx │<----- RX jaune -----│ BO    BI │<--------│ GPIO16 (`tx_pin`) │
│      GND │<----- GND vert ---->│ G2    G1 │<------->│ GND               │<-- 3.3V
│  JK-BMS  │                     │ ADuM1201 │         │    ESP32          │<-- GND
└──────────┘                     └──────────┘         └───────────────────┘

  --------------------------------------*/

#include <mcp_can.h>
#include <SPI.h>
#include <HardwareSerial.h>

#define uchar unsigned char
#define ushort unsigned short
#define uint unsigned int
#define ulong unsigned long

#define WIFI_OPTION     // enableled for sending UDP & TCP report

#define CAN_OPTION     // enabeled for CAN 

//#define SERIAL_DEBUG


// ------------------ Wifi -----------------------------------------------------
#ifdef WIFI_OPTION
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <pthread.h>

  #define UDP_PORT  5025
  IPAddress remoteIP (192,168,1,255); // broadcast

  #define TCP_PORT  5025    // TCP Server
  #define MAX_CLIENTS 6

  const char* ssid = "**************";
  const char* pass = "*********";
  bool connectedOnWifi = false;
  
  pthread_t threadTcp;
  // Creation d'un semaphore binaire
  SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();
  bool fin=false;
  
  WiFiUDP udp;
  WiFiServer server(TCP_PORT);
#endif

//---------------------------------------------------------------------------
// definition de macros pour mise a 1 et reset de bits
#ifndef SETBIT
#define SETBIT(x,n)  ((x)|=1<<(n))
#define RAZBIT(x,n)  ((x)&= ~(1<<(n)))
// definition de macros pour tester 1 bit dans une variable
#define TESTBIT(x,n)   ((x)&(1<<(n)))>>(n) // x: valeur a tester, n : no du bit
#endif
//---------------------------------------------------------------------------
// TBMS : tableau de sortie (vers TCP)
#pragma pack(2)

struct TBMS {
  uint id;
  short sTabCel[16];    // 0.001V
  short sTemp1, sTemp2, sTemp3; // temp 1.0°C 
  short sTotalVolt;     // 0.01V
  short sCurrent;       // 0.01A
  short sPower;
  ushort uSoc;          // %
  ushort usNbCycle;
  ushort sAlarm;
  ushort status;
  ushort sCapa;
  int rssi;     // niveau Wifi en db
  int crRsBms;  // cr liaison RS BMS
} tBms;

#pragma pack(4)

// TBMS_CAN : tableau de sortie (vers CAN PYLON)
struct TBMS_CAN {
  short sSoC = 50;
  short ssVoltBat = 52.4;    // 0.01V
  short sCurrent = 0;     // 0.1A
  short sTemp = 150;        // 0.1 °C
  uchar ucAla0 =0, ucAla1 = 0, ucAla2 = 0; 
  uchar ucStatus = 0xC;
} tBmsCan;
//---------------------------------------------------------------------------

ulong rxId;

unsigned char len = 0;  // longueur trame CAN recue
unsigned char rxBuf[9];
unsigned char txBuf[9];
unsigned char szId[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } ;  // Id du BMS ?

#ifdef CAN_OPTION
#define CAN0_INT 26//2                              // Set INT to pin 26
MCP_CAN CAN0(5); 
#endif                               // Set CS to pin GPIO5

uchar etatCAN = 0; 
char szMsg[80];

HardwareSerial SerialPort(2); // use UART2

uchar bufin[512];  // buffer reception RS

//------------------------------------------------------------------------------
// TcpThread() : thread Wifi & TCP
//------------------------------------------------------------------------------
#ifdef WIFI_OPTION

void *TcpThread(void *threadid) {

  while (!fin) {
  
    static WiFiClient *clients[MAX_CLIENTS] = { NULL, NULL, NULL, NULL, NULL, NULL };
      
    do {    // .. while (WiFi.status() == WL_CONNECTED) 

      Serial.printf( (char*)F("\nConnecting to SSID \"%s\" "), ssid);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, pass);

      int nb;
      for (nb=0; (nb<40) && (WiFi.status() != WL_CONNECTED) ; nb++)  {
        delay(500);
        Serial.print(".");
      }
      if (nb >= 40) {
        Serial.print( (char*) F("\nImpossible to connect to WiFi! \t aborting\n"));
        break;
      }
      else {
        connectedOnWifi = true;
        Serial.print("\nConnected\tLocal IP : " );
        Serial.print( WiFi.localIP() );

        long rssi = WiFi.RSSI();
        //Serial.printf("\nWifi Signal level: %ddb ", rssi);

        // ---------- TCP ------------------------------
        server.begin();
        server.setNoDelay(true);
        delay(500);
        Serial.printf("\nYou can now Connect on TCP port: %d\n", TCP_PORT);
      }



      while (!fin) {

        if (WiFi.status() == WL_CONNECTED) connectedOnWifi = true;
        else {
          connectedOnWifi = false;
          break;
        }

        xSemaphoreTake( semaphore, portMAX_DELAY ); // attente d'un message a envoyer
        
        long rssi = WiFi.RSSI();    // Wifi Level in db
        Serial.printf("\nWifi Signal level: %ddb ", rssi);
        tBms.rssi = rssi;  
        
        //
        // ----------- send UDP frame ------------
        //          
        udp.beginPacket(remoteIP, UDP_PORT);
        udp.write( (uchar*)&tBms, sizeof(tBms)); 
        udp.endPacket();          

        //
        // manage the TCP server 
        //
        // Check if a new client has connected
        WiFiClient newClient = server.available();

        if (newClient) {
          Serial.print("\nnouveau client TCP... ");
          // Find the first unused space
          for (int i=0 ; i<MAX_CLIENTS ; ++i) {
            if ( clients[i] == NULL ) {
              clients[i] = new WiFiClient(newClient);
              break;
            }
          }
        }

        //
        // ----------- Gestion des messages TCP vers les stations connectees ------------
        //
        char szMsg[80];           

        for (int i = 0; i < MAX_CLIENTS; i++)
        {
          if ( clients[i] != NULL ) 
          {
            int cr2 = clients[i]->connected();
            if (  cr2 > 0 ) {
              //int cr3 = clients[i]->printf("Wifi Signal level: %ddb ", rssi);
              //int cr3 = clients[i]->write((uchar*)&tBms, sizeof(tBms));
              int cr3 = clients[i]->write(bufin, 291);
              clients[i]->flush();  
              Serial.printf( "\send client TCP[%d] cr3: %d", i, cr3 );        
            }
            else {
              sprintf(szMsg, "\nclients[%d] disconnected => suppress! ", i);
              Serial.printf( szMsg );
              clients[i]->stop();
              delete ( clients[i] );
              clients[i] = NULL;
            }
          }
        }

      }
  
    } while (WiFi.status() == WL_CONNECTED); 
    
    server.end();
    Serial.printf("\nserver.end()...");
    WiFi.disconnect();
    Serial.printf("\tWifi disConnected! ");
  
  }   //   while (!fin) {

}
#endif

//---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.print("\n\nJkBms_Sofar5\t");
  Serial.print(__DATE__);
  Serial.print("  ");
  Serial.println(__TIME__);
  
#ifdef WIFI_OPTION

  int returnValue = pthread_create(&threadTcp, NULL, TcpThread, (void *) 1 );

  if (returnValue) {
      Serial.printf("\nErreur creation thread " );
  }

#endif  

#ifdef CAN_OPTION
  delay(500);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {  //Attention, MCP_16MHZ pas avec NANO !
    Serial.println("MCP2515 Initialized Successfully!");
    etatCAN = 1;
  }
  else {
    Serial.println("Error Initializing MCP2515!...");
    etatCAN = 128;
  }
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
#endif


  SerialPort.begin(115200, SERIAL_8N1, 17, 16); // use UART2 17:Rx  16:Tx A VERIFIER!
  SerialPort.setTimeout(200);
  
  
  delay(150);

}

#ifdef CAN_OPTION   // --------------------------------------------------

//---------------------------------------------------------------------
// 0x351
//---------------------------------------------------------------------
int sendCan351( short sVolt, short sCurLim1, short sCurLim2)
{
  //
  // requete indiquant la tension 0x14 0x02 0xC8 0x05 0xC8 0x05 0xCC 0x01
  //        
  ulong ulId = 0x351;  
  
  short * ps = (short*) &txBuf[0];   // pour considerer la trame comme un tableau de short
                  
  ps[0] = sVolt;      // en 0.1V
  ps[1] = sCurLim1;   // en 0.1A
  ps[2] = sCurLim2;   // en 0.1A
  ps[3] = 490;   // 460;    

  CAN0.sendMsgBuf(ulId, CAN_STDID, 8, txBuf);  // CAN_STDID (0): standard frame         
  sprintf( szMsg, "\n-> %08lX\t", ulId );
  Serial.print(szMsg); 

  for (int i=0; i < 8; i++ ) {
    sprintf( szMsg, "%02X ", txBuf[i]);
    Serial.print(szMsg); 
  } 
  
  return (1);
}

//---------------------------------------------------------------------
// 0x355 SOC, SoH
//---------------------------------------------------------------------
int sendCan355(short sSoc, short sSoh)
{
  ulong ulId = 0x355;  
  
  short * ps = (short*) &txBuf[0];   
                  
  ps[0] = sSoc;  // en %
  ps[1] = sSoh;   // en %

  CAN0.sendMsgBuf(ulId, CAN_STDID, 4, txBuf);           
  sprintf( szMsg, "\n-> %08lX\t", ulId );
  Serial.print(szMsg); 

  for (int i=0; i < 4; i++ ) {
    sprintf( szMsg, "%02X ", txBuf[i]);
    Serial.print(szMsg); 
  } 
  
  return (1);
}

//---------------------------------------------------------------------
// 0x356 
//---------------------------------------------------------------------
int sendCan356(short sVolt, short sCur, short sTemp)
{
  // Average Voltage (0x4e, 0x13 = 4942)/ Current(0x02, 0x03 = 770) / Temp  (0x04, 0x05 = 102.9°C)  
         
  ulong ulId = 0x356;  
  
  short * ps = (short*) &txBuf[0];   
                  
  ps[0] = sVolt;  // en 0.01V
  ps[1] = sCur;   // en 0.1A
  ps[2] = sTemp;   // en 0.1 deg C

  CAN0.sendMsgBuf(ulId, CAN_STDID, 6, txBuf);           
  sprintf( szMsg, "\n-> %08lX\t", ulId );
  Serial.print(szMsg); 

  for (int i=0; i < 6; i++ ) {
    sprintf( szMsg, "%02X ", txBuf[i]);
    Serial.print(szMsg); 
  } 
  
  return (1);
}

//---------------------------------------------------------------------
// 0x359  Alarms 
//---------------------------------------------------------------------
int sendCan359(uchar protec1, uchar protec2, uchar Alarm1, uchar Alarm2)
{
  //  0x00 0x00 0x00 0x00 0x04 0x50 0x4E
         
  ulong ulId = 0x359;  
  
                       //        bit 7         / 6 / 5 /       4       /    3           / 2          / 1
  txBuf[0] = protec1;  // Discharge over current / / / Cell under Temp / Cell over Temp / Under Volt / Over Volt
  txBuf[1] = protec2;  //    bit 7      /  6 / 5  / 4  / 3: system error  / 2  / 1  / 0 : Charge Over current
  txBuf[2] = Alarm1;   
  txBuf[3] = Alarm2;
  txBuf[4] = 1;       // Pack number
  txBuf[5] = 'P'; 
  txBuf[6] = 'N'; 
                  
  CAN0.sendMsgBuf(ulId, CAN_STDID, 7, txBuf);           
  sprintf( szMsg, "\n-> %08lX\t", ulId );
  Serial.print(szMsg); 

  for (int i=0; i < 7; i++ ) {
    sprintf( szMsg, "%02X ", txBuf[i]);
    Serial.print(szMsg); 
  } 
  
  return (1);
}

//---------------------------------------------------------------------
// 0x35C
//---------------------------------------------------------------------
int sendCan35C(uchar flags)
{
  // Bit 7 : Charge enable 
  // Bit 6 : Discharge enable 
  // Bit 5 : Request force charge I
  // Bit 4 : Request force charge II
  // Bit 3 : Request full charge 
         
  ulong ulId = 0x35C;  
  
  txBuf[0] = flags;   
                  
  CAN0.sendMsgBuf(ulId, CAN_STDID, 1, txBuf);           
  sprintf( szMsg, "\n-> %08lX\t", ulId );
  Serial.print(szMsg); 

  for (int i=0; i < 1; i++ ) {
    sprintf( szMsg, "%02X ", txBuf[i]);
    Serial.print(szMsg); 
  } 
  
  return (1);
}

//---------------------------------------------------------------------
// 0x35E
//---------------------------------------------------------------------
int sendCan35E()
{
  strncpy((char*)txBuf, "PYLON   ", 8);         
  ulong ulId = 0x35E;  
                  
  CAN0.sendMsgBuf(ulId, CAN_STDID, 8, txBuf);           
  sprintf( szMsg, "\n-> %08lX\t", ulId );
  Serial.print(szMsg); 

  for (int i=0; i < 8; i++ ) {
    sprintf( szMsg, "%02X ", txBuf[i]);
    Serial.print(szMsg); 
  } 
  Serial.print((char*)txBuf);
  
  return (1);
}

#endif  // ----------------------------------------------------------

//------------------------------------------------------------------------------
// int analyse()  // Analyse de la trame RS venant du BMS JK
//------------------------------------------------------------------------------

int analyse (uchar *buf, int lg)
{

  if (lg < 80) {
    Serial.printf( "\nErreur nb recu: %d ", lg);
    return(-1);
  }
  
  ushort chk1 = chksum(buf, lg-4);
  ushort chk2 = ( buf[lg-2] << 8 ) | ( buf[lg-1] & 0xFF ); //*(ushort*) &buf[lg-2];
  if (chk1 != chk2) {
    Serial.printf( "\nErreur chksum : %04X / %04X ", chk1, chk2);
    return(-2);
  }
  
  // Valeurs cellules:
  
  int nbCel = buf[12] / 3;
  if ( ( nbCel < 4) || ( nbCel > 24 ) || (buf[11] != 0x79) ) {
    Serial.printf( "\nErreur nb Cell: %d ", nbCel);
    return(-3);
  } 
#ifdef SERIAL_DEBUG
  Serial.printf( "\n%d Cell: ", nbCel);
#endif

  int offset1 = 13;   // depart zone des cellules
  
  for ( int i=0, k=offset1 + 1; (i<nbCel) && (i<16); i++, k+=3 ) {
    tBms.sTabCel[i] = (buf[k] << 8) | buf[k+1] ;
    float fCell = (int) tBms.sTabCel[i] * 0.001;
#ifdef SERIAL_DEBUG
    Serial.printf( "%5.3fV ", fCell);
#endif
    if ((fCell > 5.0) || (fCell < 0.1)) {
      Serial.printf( "\nErreur coherence--> abandon trame! ");
      return(-4);
    }
    if (i == 7) Serial.printf( "\n\t ");
  }
  //
  // Temperatures 0x80 0x00 0x1D  0x81 ...
  //
  int offset2 = offset1 + buf[12]  ; // point de depart zone temperature (0x80 ...)
  tBms.sTemp1 = buf[offset2+2];     // 1 °C
  tBms.sTemp2 = buf[offset2+5];
  tBms.sTemp3 = buf[offset2+8];

  tBmsCan.sTemp = (tBms.sTemp1 + tBms.sTemp2) * 5;        // 0.1 °C
  
#ifdef SERIAL_DEBUG
  printf("\nTemp: %d°C\t%d°C\t%d°C", tBms.sTemp1, tBms.sTemp2, tBms.sTemp3 );
#endif

  //
  // Total battery voltage: 0x83 0x14 0xEF  Total battery current: 0x84 0x01 0x15 
  int offset3 = offset2 + 9;
  tBms.sTotalVolt = (buf[offset3+1] << 8) | buf[offset3+2] ;
 
  short sCur = (buf[offset3+4] << 8) | buf[offset3+5] ; // 28/02/23 Bug Daly
  if ( sCur & 0x8000) {
    tBms.sCurrent = sCur & 0x7FFF;
  }
  else {
    tBms.sCurrent = -(sCur & 0x7FFF);
  }

  tBmsCan.ssVoltBat = tBms.sTotalVolt;    // 0.01V
  tBmsCan.sCurrent = tBms.sCurrent / 10;  // 0.1A
 
  float fVoltage = tBms.sTotalVolt * 0.01;
  float fCurrent = tBms.sCurrent * 0.01;
  
  float fPower = fVoltage * fCurrent;
  tBms.sPower = fPower;  // 27/02/23  1.0W
  
  printf("\nBat: %5.2fV %5.2fA %4.0fW  ", fVoltage, fCurrent, fPower );
  
  tBms.uSoc = buf[offset3+7];
  tBmsCan.sSoC = tBms.uSoc;
  
  printf("\tSoC: %2d%% ", tBms.uSoc );
  
  // 0x87 0x00 0x04: Number of battery cycles 
  tBms.usNbCycle = (buf[offset3+11] << 8) | buf[offset3+12];
#ifdef SERIAL_DEBUG
  printf("\tnb cycles: %d ", tBms.usNbCycle );
#endif

  // 0x8B 0x00 0x00: Battery warning message                     0000 0000 0000 0000
  //
  // Bit 0    Low capacity                                1 (alarm), 0 (normal)    warning
  // Bit 1    Power tube overtemperature                  1 (alarm), 0 (normal)    alarm
  // Bit 2    Charging overvoltage                        1 (alarm), 0 (normal)    alarm
  // Bit 3    Discharging undervoltage                    1 (alarm), 0 (normal)    alarm
  // Bit 4    Battery over temperature                    1 (alarm), 0 (normal)    alarm
  // Bit 5    Charging overcurrent                        1 (alarm), 0 (normal)    alarm
  // Bit 6    Discharging overcurrent                     1 (alarm), 0 (normal)    alarm
  // Bit 7    Cell pressure difference                    1 (alarm), 0 (normal)    alarm
  // Bit 8    Overtemperature alarm in the battery box    1 (alarm), 0 (normal)    alarm
  // Bit 9    Battery low temperature                     1 (alarm), 0 (normal)    alarm
  // Bit 10   Cell overvoltage                            1 (alarm), 0 (normal)    alarm
  // Bit 11   Cell undervoltage                           1 (alarm), 0 (normal)    alarm
  // Bit 12   309_A protection                            1 (alarm), 0 (normal)    alarm
  // Bit 13   309_A protection                            1 (alarm), 0 (normal)    alarm
  // Bit 14   Reserved
  // Bit 15   Reserved
  
  int offset4 = offset3 + 21;   // zone Alarmes & statut
  
  tBms.sAlarm = (buf[offset4+1] << 8) | buf[offset4+2];
#ifdef SERIAL_DEBUG
  printf("\nAlarmes: %04X ", tBms.sAlarm  );
#endif

  // 0 / over Temp / under Volt / over Temp2 / under Temp / 0 / 0 / over Current Discharge
  tBmsCan.ucAla0 = (TESTBIT(tBms.sAlarm,4) << 1) | (TESTBIT(tBms.sAlarm,11) << 2) | (TESTBIT(tBms.sAlarm,8) << 3) 
                    | (TESTBIT(tBms.sAlarm,9) << 4) | (TESTBIT(tBms.sAlarm,6) << 7);

  // over current Charge / 0 / 0 / BMS internal / 0 / 0 / 0 / 0                
  tBmsCan.ucAla1 = TESTBIT(tBms.sAlarm,5) | ( TESTBIT(tBms.sAlarm,12) || TESTBIT(tBms.sAlarm,13) ) << 3 ;

  // 0 / Hight Volt / Low Volt / Low Temp 0 / 0 / Hight current disCharge
  tBmsCan.ucAla2 = (TESTBIT(tBms.sAlarm,10) << 1) | (TESTBIT(tBms.sAlarm,11) << 2) | (TESTBIT(tBms.sAlarm,9) << 4) | (TESTBIT(tBms.sAlarm,6)  << 7);                   


  // 0x8C 0x00 0x07: Battery status information                  0000 0000 0000 0111
  // Bit 0: Charging enabled        1 (on), 0 (off)
  // Bit 1: Discharging enabled     1 (on), 0 (off)
  // Bit 2: Balancer enabled        1 (on), 0 (off)
  // Bit 3: Battery dropped(?)      1 (normal), 0 (offline)
  // Bit 4...15: Reserved
  
  tBms.status = (buf[offset4+4] << 8) | buf[offset4+5];
  printf("\tStatus info: %04X ", tBms.status);

  // Charging & DisCharging enabled 
  tBmsCan.ucStatus = (TESTBIT(tBms.status,0) << 7 ) | (TESTBIT(tBms.status,1) << 6 );


  int offset5 = offset2 + 118;

  tBms.sCapa = (buf[offset5+3] << 8) | buf[offset5+4] ;
#ifdef SERIAL_DEBUG
  printf("\tInitial capacity %d: ", tBms.sCapa);
#endif  
    /*-------------
    uchar ucCanAla0 = (TESTBIT(tBms.sAlarm,4) << 1) | (TESTBIT(tBms.sAlarm,11) << 2) | (TESTBIT(tBms.sAlarm,8) << 3) 
                    | (TESTBIT(tBms.sAlarm,9) << 4) | (TESTBIT(tBms.sAlarm,6) << 7);
                    
    uchar ucCanAla1 = TESTBIT(tBms.sAlarm,5) | ( TESTBIT(tBms.sAlarm,12) || TESTBIT(tBms.sAlarm,13) ) << 3 ;
    
    uchar ucCanAla2 = (TESTBIT(tBms.sAlarm,10) << 1) | (TESTBIT(tBms.sAlarm,11) << 2) | (TESTBIT(tBms.sAlarm,9) << 4) | (TESTBIT(tBms.sAlarm,6)  << 7); 
                      -----------------*/
 
  
  return(0);
}
//------------------------------------------------------------------------------
//  chksum()
//------------------------------------------------------------------------------
ushort chksum(const uchar data[], const short len) {
  ushort checksum = 0;
  for (int i = 0; i < len; i++) {
    checksum = checksum + data[i];
  }
  return checksum;
}

//-------------------------------------------------------------------
void affich (uchar *buf, int nb, bool bEmis)
{
  if (bEmis) Serial.printf("\n> %d bytes 0x", nb);
  else Serial.printf("\n< %d bytes 0x", nb);

  for (int i=0; (i<nb) && (i<50); i++) {
    Serial.printf("%02X ", buf[i]&0xFF );
  }
}
//---------------------------------------------------------------------
// loop()
//---------------------------------------------------------------------

void loop() {

  //-------------- Liaison RS 485 - BMS JK --------------------------------

  int cr, cr1, cr2, cr3;

  // Requete de demande de lecture de toutes les valeurs
  static uchar bufout[64] = {0x4E, 0x57, 0, 0x13, 0, 0, 0, 0, 0x06, 0x03, 0, 0, 0, 0, 0, 0, 0x68, 0, 0, 0x01, 0x29};
  //static uchar bufin[512];

  
  ushort chk1 = chksum(bufout, 17);
  ushort chk2 = ( bufout[19] << 8 ) | ( bufout[20] & 0xFF ); 

  // envoi requete RS vers BMS JK
  cr1 = SerialPort.write(bufout, 21);
#ifdef SERIAL_DEBUG
  affich(bufout, 21, true);
#endif
  
  // attente reponse
  cr2 = SerialPort.readBytes(bufin, 512);
  if (cr2 > 0) {
#ifdef SERIAL_DEBUG
    affich(bufin, cr2, false);
#endif
    cr = analyse (bufin, cr2);
    tBms.crRsBms = cr; // -1; Er nb<80 / -2: Er chksum / -3 Er nb cellules / -4 Er coherence valeur Cell
  }
  else tBms.crRsBms = -6;    // rien recu ou erreur SerialPort

 //---------------------------------- Wifi ------------------------------------
 
   
#ifdef WIFI_OPTION

  xSemaphoreGive( semaphore );    // declenchement de l'envoi du message en UDP et TCP

#endif  

   
#ifdef CAN_OPTION  // ------------------- CAN ----------------------------------
  static int cptCAN = 0;
  
  if ( etatCAN != 1 ) {
    Serial.print("\nErreur, CAN non initialise! ");
    Serial.println( etatCAN );
    delay(1000);
    return;
  }

  if ( cr < 0 ) {   // V5.0 analyse() OK?
    delay(100);     // NON
    return;
  }
  
  for( int loop =0; loop<6; loop++){

    switch (loop) {
      case 0: cr3 = sendCan351( 545, 650, 650); break;  // Max Volt (54.5*10), Current Lim1 (65*10), current Lim2 (65*10)
      case 1: cr3 = sendCan355(tBmsCan.sSoC, 100); break; 
      case 2: cr3 = sendCan356(tBms.sTotalVolt, tBms.sCurrent/10, tBms.sTemp1*10); break; 
      //case 3: cr = sendCan359(ucCanAla0, ucCanAla1, ucCanAla2, 0); break; 
      case 3: cr3 = sendCan359(0, 0, 0, 0); break; 
      //case 4: cr = sendCan35C(usCanStatus); break;   // 0xC0 : charge & discharge enabled
      case 4: cr3 = sendCan35C(0xC0); break; 
      case 5: cr3 = sendCan35E(); break; 

    }
    

    
    delay(150);
  }

  
  
#endif    // -------------------------- fin CAN --------------------------------


  
  delay(100);

}
