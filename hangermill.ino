//Possible bug log
//added address of speed and incline characteristics
//11-14-2019-->Working on Windows
//#include <NeoPixelBrightnessBus.h>
//#include <NeoPixelBus.h>

/*
//ESP32 Arduino Settings
Board ESP32 Dev Module
Upload Speed: 921600
CPU Freq 80 mHz *BT/Wifi"

*/
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <String.h>


///////////SOUND FUNCTIONS///////////


TaskHandle_t Task1;

//NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1800KbpsMethod> strip(LED_COUNT, LED_PIN);
//byte flags = 0b00111110;

#define WALKING_METS 1
#define RUNNING_METS 2
float fGrade = 5;    //percent 
float fSpeed = 8;    //mph
int nHeartRate = 60;   //bpm


int targetMetsMode = RUNNING_METS;
 
////////////////INTEGRATION VALUES//////////////////////
float fTotalCalories = 0;
float fWeight = 70.0;
float fElevationGain = 0;
float fIntegratedDistance = 0;

float fElevation = 0.0;
float fCadence = 180;
float fCadenceFast = 185;
bool _BLEClientConnected = false;

#define HEART_RATE_UUID BLEUUID((uint16_t)0x180d)
#define FITNESS_MACHINE_UUID BLEUUID((uint16_t)0x1826)
#define DEVICE_INFORMATION_UUID BLEUUID((uint16_t)0x180a)

BLECharacteristic *pFitnessMachineControlPoint;

class ControlPointCallback: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
      byte *pData = pCharacteristic->getData();
      {
        if (pData[0] == 0x00) //initiate procedure to request control of treadmill
        {
          byte controlPointResponse[8];
          controlPointResponse[0] = 0x80;
          controlPointResponse[1] = 0x00;
          pFitnessMachineControlPoint->setValue((byte *)&controlPointResponse,2);
          Serial.println("Request Control");
        }
        if (pData[0] == 0x01) //initiate procedure to reset treadmill
        {
          byte controlPointResponse[8];
          controlPointResponse[0] = 0x80;
          controlPointResponse[1] = 0x00;
          pFitnessMachineControlPoint->setValue((byte *)&controlPointResponse,2);
          Serial.println("Reset Treadmill");
        }
        if (pData[0] == 0x02) //initiate procedure to set Speed
        {
           Serial.println("Set Speed");
          uint16_t nSpeed = (pData[1] & 0xff) + ((pData[2] & 0xff) << 8);
          float fDesiredSpeed = nSpeed / 160.934;
          
          byte controlPointResponse[8];
          controlPointResponse[0] = 0x80;
          controlPointResponse[1] = 0x02;
          pFitnessMachineControlPoint->setValue((byte *)&controlPointResponse,2);
          char szCommand[16];
          char szSpeed[8];
          dtostrf(fDesiredSpeed,0,2,szSpeed);
          sprintf(szCommand,"speed=%s",szSpeed);
          Serial2.println(szCommand);
          Serial.println(szCommand);
        }
        if (pData[0] == 0x03) //initiate procedure to set Incline
        {
          
          Serial.println("Set Incline");
          uint16_t nIncline = (pData[1] & 0xff) + ((pData[2] & 0xff) << 8);
          float fIncline = nIncline / 10.0;
          Serial.print("Desired Incline: "); Serial.println(fIncline);
          byte controlPointResponse[8];
          controlPointResponse[0] = 0x80;
          controlPointResponse[1] = 0x03;
          pFitnessMachineControlPoint->setValue((byte *)&controlPointResponse,2);
          
          char szCommand[16];
          char szIncline[8];
          dtostrf(fIncline,0,2,szIncline);
          sprintf(szCommand,"grade=%s",szIncline);
          Serial2.println(szCommand);
          Serial.println(szCommand);
        }
      }
      /*std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }*/
    }
};


//BLECharacteristic heartRateMeasurementCharacteristics(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
//BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A38), BLECharacteristic::PROPERTY_READ);
//BLEDescriptor heartRateDescriptor(BLEUUID((uint16_t)0x2901));
//BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));



///MYSTUFF HANGERMILL



BLECharacteristic treadmillDataCharacteristic(BLEUUID((uint16_t)0x2acd),BLECharacteristic::PROPERTY_NOTIFY);//this needs to stay global for data





byte fitnessMachineFeaturesArray[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};//FITNESS MACHINE FEATURES FIELD
                                                  //0 - Average Speed Supported
                                                   //1 - Cadence Supported
                                                   //2 - Total Distance Supported
                                                   //3 - Inclination Supported
                                                   //4 - Elevation Gain Supported
                                                   //5 - Pace Supported
                                                   //6 - Step Count Supported
                                                   //7 - Resistance Level Supported
                                        //8 - Stride Count Supported
                                                   //9 - Expended Energy Supported
                                                   //10 - Heart Rate Measurement Supported
                                                   //11 - Metabolic Equivalent Supported
                                                   //12 - Elapsed Time Supported
                                                   //13 - Remaining Time Supported
                                                   //14 - Power Measurement Supported
                                                   //15 - Force on Belt and Power Output Supported
                                                   //16 - User Data Retention Supported
                                                  
                                                  //includes both the supported features and the target features.  Here, supporting target speed and target heart rate
/*
 
 
 
 

 *17-31 Reserved
 *TARGET SETTING FEATURES FIELD
 *0 - Speed Target Setting Supported
 *1 - Inclination Target Setting Supported
 *2 - Resistance Target Setting Supported
 *3 - Power Target Setting Supported
 *4 - Heart Rate Target Setting Supported
 *5 - Targeted Expended Energy Configuration Supported
 *6 - Targeted Step Number Configuration Supported
 *7 - Targeted Stride Number Configuration Supported
 *8 - Targeted Distance Configuration Supported
 *9 - Targeted Training Time Configuration Supported
 *10 - Targeted Time in Two Heart Rate Zones Configuration Supported
 *11 - Targeted Time in Three Heart Rate Zones Configuration Supported
 *12 - Targeted Time in Five Heart Rate Zones Configuration Supported
 *13 - Indoor Bike Simulation Parameters Supported
 *14 - Wheel Circumference Configuration Supported
 *15 - Spin Down Control Supported
 *16 - Targeted Cadence Configuration Supported
 *17-31 - Reserved
 *
 *
 ** FLAGS
 * 

*/

uint16_t supportedSpeedRangeArray[3] = {80,2896,16};//0.5 mph - 18 mph - 0.1kph all decimal minus 2
int16_t supportedInclinationRangeArray[3] = {-20,150,2};
byte trainingStatusArray[8] = {0,0,0,0,0,0,0,0};

class MyServerCallbacks : public BLEServerCallbacks 
{
    
    void onConnect(BLEServer* pServer) 
    {
      _BLEClientConnected = true;
      Serial.println("BLE connected");
    };

    void onDisconnect(BLEServer* pServer) 
    {
      _BLEClientConnected = false;
      Serial.println("BLE disconnected");
    }
};

void InitBLE() 
{
  Serial.println("Initializing BLE!!");
  BLEDevice::init("HANGMILL3");
  // Create the BLE Server
  Serial.println("creating Server");
  BLEServer *pServer = BLEDevice::createServer();
  
  pServer->setCallbacks(new MyServerCallbacks());

  
  Serial.println("Creaiging Service");
  ////////////TREADMILL STUFF///////////////
  // Create the BLE Service
  BLEService *pTreadmillService = pServer->createService(FITNESS_MACHINE_UUID);
  Serial.print("Treadmill UUID: ");
  Serial.println(pTreadmillService->getUUID().toString().c_str());
  
  
  
  Serial.println("Setting fitness machine");
  ////////FITNESS MACHINE FEATURE CHACTERISTIC////////////////////
  BLECharacteristic *pFitnessMachineFeatureCharacteristic = pTreadmillService->createCharacteristic (BLEUUID((uint16_t)0x2acc),BLECharacteristic::PROPERTY_READ);
  pFitnessMachineFeatureCharacteristic->setValue((byte *)&fitnessMachineFeaturesArray,sizeof(fitnessMachineFeaturesArray));

  Serial.println("Setting Speed Range");
  ////////SPEED RANGE CHARACTERISTIC////////////////////
  BLECharacteristic *pSupportedSpeedRangeCharacteristic = pTreadmillService->createCharacteristic(BLEUUID((uint16_t)0x2ad4),BLECharacteristic::PROPERTY_READ);
  pSupportedSpeedRangeCharacteristic->setValue((byte *)&supportedSpeedRangeArray,sizeof(supportedSpeedRangeArray));

  Serial.println("Setting Incline Range");
  //////////INCLINATION RANGE CHARACTERISTIC//////////////////////////////
  
  BLECharacteristic *pSupportedInclinationRange = pTreadmillService->createCharacteristic(BLEUUID((uint16_t)0x2ad5),BLECharacteristic::PROPERTY_READ);
  pSupportedInclinationRange->setValue ((byte *)&supportedInclinationRangeArray,sizeof(supportedInclinationRangeArray));
  
  Serial.println("Setting Treadmill Data");
  ////////////////TREADMILL DATA CHARACTERISTIC/////////////////////////
  pTreadmillService->addCharacteristic(&treadmillDataCharacteristic);
  treadmillDataCharacteristic.addDescriptor(new BLE2902());

  Serial.println("Treadmill Control Point");
  ////////////////FITNESS MACHINE CONTROL POINT/////////////////////////
  pFitnessMachineControlPoint = pTreadmillService->createCharacteristic(BLEUUID((uint16_t)0x2ad9),BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pFitnessMachineControlPoint->setCallbacks(new ControlPointCallback());

  Serial.println("Device Info");
  /////////////////////////////////////////////////////////////////////////////////
  ////DEVICE INFORMATION STUFF/////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  
  BLEService *pDeviceInformationService = pServer->createService(DEVICE_INFORMATION_UUID);
  BLECharacteristic *pManufacturerNameCharacteristic = pDeviceInformationService->createCharacteristic(BLEUUID((uint16_t)0x2a29),
                                                                                                        BLECharacteristic::PROPERTY_READ);
  
  
  pManufacturerNameCharacteristic->setValue("HANGERMILL");
  
  pTreadmillService->start();
  pDeviceInformationService->start();
  ///ADVERTISING STUFF
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  
  
  pAdvertising->addServiceUUID(FITNESS_MACHINE_UUID);
  pAdvertising->addServiceUUID(DEVICE_INFORMATION_UUID);
  
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
  Serial.print("Treadmill Service UUID: ");
  Serial.println(pTreadmillService->getUUID().toString().c_str());
  
  Serial.print("Fitness Machine Characteristic UUID: ");
  Serial.println(pFitnessMachineFeatureCharacteristic->getUUID().toString().c_str());

  
  Serial.print("Supported Speed Range Characteristic UUID: ");
  Serial.println(pSupportedSpeedRangeCharacteristic->getUUID().toString().c_str());
  
  Serial.print("Supported Incline Range UUID: ");
  Serial.println(pSupportedInclinationRange->getUUID().toString().c_str());
  
  Serial.print("Treadmill Data UUID: ");
  Serial.println(treadmillDataCharacteristic.getUUID().toString().c_str());
  
  
  
  
  
  
}




void tone (int freq,int delay)
{
  unsigned long t = millis();
  long d = 500000 / freq;
  while (millis() - t < delay)
  {
    dacWrite(DAC1,64);
    delayMicroseconds(d);
    dacWrite(DAC1,0);
    delayMicroseconds(d);
      
  }
}

void noise(int delay,int hiFreq)
{
  unsigned long t = millis();
  
  while (millis() - t < delay)
  {
    int r = random(hiFreq/4,hiFreq);
    for (int i = 0;i<1;i++)
    {
      dacWrite(DAC1,64);
      delayMicroseconds(r);
      dacWrite(DAC1,0);
      delayMicroseconds(r);
    }
    
      
  }
}

void play(int hz, int duration)
{
  tone(hz,duration);
}
void upChirp ()
{
  play(610,75);
  play(1220,75);
}

void doubleChirp ()
{
  play(610,75);
  play(1220,75);
  play(610,75);
  play(1220,75);
  
  
  
  
}

void doubleTone ()
{
  play(610,30);
  delay(30);
  play(610,30);
  
  
  
}
void quickTone ()
{
  play(1220,25);
  
}
void quickToneHigh ()
{
  play(2440,25);
  
}



void pollSerial()
{
  static char szStr[16] = "";
  static uint16_t nIndex = 0;
 
  
  //Serial.println(szStr);
  while (Serial2.available() > 0)
  {
    
    char ch = Serial2.read();
    //Serial.println(ch);
   
    
    if ((ch != 10) && (ch != 13)) szStr[nIndex] = ch;
    nIndex++;
    if (nIndex >= 16) nIndex = 0;
    if (ch == '\n')
    {
      szStr[nIndex-1] = 0;
      if (szStr[0] == 'g') fGrade = atof(&szStr[2]);
      if (szStr[0] == 's') fSpeed = atof(&szStr[2]);
      if (szStr[0] == 'p') fCadence = atof(&szStr[2]);
      if (szStr[0] == 'h') nHeartRate = atoi(&szStr[2]);
      
      if (szStr[0] == 'c') 
      {
        int nChirp = atoi(&szStr[2]);
        if (nChirp == 1) upChirp();
        if (nChirp == 2) doubleChirp();
        if (nChirp == 3) doubleTone();
        if (nChirp == 4) quickTone();
        if (nChirp == 5) quickToneHigh();
      }
      if (szStr[0] == 'r')
      {
        fTotalCalories = 0;
        fWeight = 70.0;
        fElevationGain = 0;
        fIntegratedDistance = 0;
      }
      
      
      nIndex = 0;
      memset(szStr,0,sizeof(szStr));
    }
    
  }

  //Now for regular serial entry
  static char szCommandStr[16] = "";
  static uint16_t nCommandIndex = 0;
  while (Serial.available() > 0)
  {
    
    char ch = Serial.read();
   
    
    if ((ch != 10) && (ch != 13)) szCommandStr[nCommandIndex] = ch;
    nCommandIndex++;
    if (nCommandIndex >= 16) nCommandIndex = 0;
    if (ch == '\n')
    {
      Serial.println("Command read");
      szCommandStr[nCommandIndex-1] = 0;
      if (szCommandStr[0] == 'g') fGrade = atof(&szCommandStr[2]);
      if (szCommandStr[0] == 's') fSpeed = atof(&szCommandStr[2]);
      if (szCommandStr[0] == 'p') fCadence = atof(&szCommandStr[2]);
      if (szCommandStr[0] == 'h') nHeartRate = atoi(&szCommandStr[2]);
      if (szCommandStr[0] == 'c') 
      {
        int nChirp = atoi(&szCommandStr[2]);
        if (nChirp == 1) upChirp();
        if (nChirp == 2) doubleChirp();
        if (nChirp == 3) doubleTone();
        if (nChirp == 4) quickTone();
        if (nChirp == 5) quickToneHigh();
      }
      if (szStr[0] == 'r')
      {
        fTotalCalories = 0;
        fWeight = 70.0;
        fElevationGain = 0;
        fIntegratedDistance = 0;
      }
      
      nCommandIndex = 0;
      memset(szCommandStr,0,sizeof(szStr));
    }
    
  
  }
  
}







#pragma pack (push,1)
struct
{
  byte flags1;                    //0 - More Data
                                  //1 - Average Speed present
                                  //2 - Total Distance Present
                                  //3 - Inclination and Ramp Angle Setting present
                                  //4 - Elevation Gain present
                                  //5 - Instantaneous Pace present
                                  //6 - Average Pace present
                                  //7 - Expended Energy present
  byte flags2;                    //8 - Heart Rate present
                                  //9 - Metabolic Equivalent present
                                  //10 - Elapsed Time present
                                  //11 - Remaining Time present
                                  //12 - Force on Belt and Power Output present
                                  //13-15 Reserved

  uint16_t instantaneousSpeed;    //Kilometers per hour resolution of 0.01
  uint16_t averageSpeed;          //Kilometers per hour resolution of 0.01
  byte totalDistance[3];          //meters with a resolution of 1 [OFFSET: 6]
  int16_t inclination;            //Percent with a resolution of 0.1 [OFFSET: 9]
  int16_t rampAngleSetting;       //Degree with a resolution of 0.1 [OFFSET:11]
  uint16_t positiveElevationGain; //meters with a resolution of 0.1 [OFFSET 13]
  uint16_t negativeElevationGain; //meters with a resolution of 0.1 [OFFSET:15]
  uint8_t instantaneousPace;      //Kilometers per minute with a resolution of 0.1 [OFFSET: 17]
  uint8_t averagePace;            //Kilometers per minute with a resolution of 0.1 [OFFSET: 18]
  uint16_t totalEnergy;           //KCAL with a resolution of 1 [OFFSET: 19]
  uint16_t energyPerHour;         //KCAL with a resolution of 1 [OFFSET: 21]
  uint8_t energyPerMinute;        //KCAL with a resolution of 1 [OFFSET: 23]
  uint8_t heartRate;              //Heart Rate beats per minute resolution of 1 [OFFSET: 24]
  uint8_t metabolicEquivalent;    //METS with resolution of 0.1 [OFFSET: 25]
  uint16_t elapsedTime;           //Elapsed time in seconds with a resolution of 1
  uint16_t remainingTime;         //Time Remaining in seconds with a resolution of 1
  int16_t forceOnBelt;            //Newton with a resolution of 1
  int16_t powerOutput;            //Watts with a resolution of 1
}treadmillData;
#pragma pack(pop)

#pragma pack (push,1)
struct
{
  byte flags;
  uint16_t speed; //meters per second.  resolution is 1/256
  byte cadence;//1/Miniute... RPM.  resolution is 1
  uint16_t strideLength;// in meters resolution 1/100 m (cm)
  uint32_t totalDistance;// in meters resolution is 1/10 m (decimeter)
  
}RSCmeasurementData;
#pragma pack(pop)



float fWalkingMets()
{
    float mpm = fSpeed * 26.8224;
    return (0.1 * mpm + 1.8 * mpm * fGrade/100 + 3.5)/3.5;
}
float fRunningMets()
{
    float mpm = fSpeed * 26.8224;
    return (0.2 * mpm + 0.9 * mpm * fGrade/100 + 3.5)/3.5;
}

float fMets()
{
  if (targetMetsMode == WALKING_METS) return fWalkingMets();
  if (targetMetsMode == RUNNING_METS) return fRunningMets();
  return (-1);
}
void integrateStuff()
{
  

  static unsigned long lastCheck = millis();
  //if (millis() - lastCheck < 1000) return;
  float fDeltaSecs = (millis() - lastCheck) / 1000.0;//seconds
  lastCheck = millis();
  float deltaCalories = fDeltaSecs * fMets() * fWeight / 3600.0;
  //Serial.println(deltaCalories);
  if (fSpeed >= 0.5) fTotalCalories += deltaCalories;
  fElevationGain = fElevationGain + (fGrade / 100 * 5280.0 * fDeltaSecs * fSpeed / 3600.0);
  fIntegratedDistance += (fDeltaSecs * fSpeed / 3600.0);
  //Serial.println(fDeltaSecs);
  //Serial.print(fIntegratedDistance);
  //Serial.print(" ");
  //Serial.println(fDist());
  

}
void setup() 
{
  
  delay(3000);//sanity check
  
  //DAC1 is 25
  //DAC2 is 26
  
  
  Serial.begin(115200);
  Serial.println("Start");
  Serial2.begin(57600);
  
  InitBLE();
  Serial.print ("DAC1: ");
  Serial.println(DAC1);

  Serial.print ("DAC2: ");
  Serial.println(DAC2);
  //for (int i = 0;i<100;i++)
  //{
  //  noise (30);
  //  delay(337-30);
  //
  //}
  

  
  
  Serial.println(sizeof(treadmillData));
 
}

void printTreadmillData()
{
  Serial.print("instantaneousSpeed: ");Serial.print(treadmillData.instantaneousSpeed);Serial.print("  ");
  Serial.print("averageSpeed: ");Serial.print(treadmillData.averageSpeed);Serial.print("  ");
  uint32_t t = treadmillData.totalDistance[0] + (treadmillData.totalDistance[1] << 8) + (treadmillData.totalDistance[2] << 16);
  Serial.print("totalDistance: ");Serial.print(t);Serial.println("  ");
  Serial.print("inclination: ");Serial.print(treadmillData.inclination);Serial.print(" ");
  Serial.print("Ramp Angle: ");Serial.print(treadmillData.rampAngleSetting);Serial.println();
  
}
void updateBT ()
{
  /**********THIS IS HUGE!!*****************
   * FLAG INDEX 0 is 1
   * FLAG INDEX 1 is 2
   * FLAG INDEX 2 is 4
   * etc....
   * to set FLAGS indexed 0,1,2, set the byte to 1 + 2 + 4 = 7!
  */
  memset(&treadmillData,0,sizeof(treadmillData));
  static unsigned int lastUpdate = millis();
  if (millis() - lastUpdate < 1000) return;
  lastUpdate = millis();
  //////////////FLAGS////////////////////
  
  treadmillData.flags1 = 0b11111110;
  treadmillData.flags2 = 0b00011111;
  
 /////////////////////////////fSpeed//////////////////////
 

  float fSpeedConst = constrain (fSpeed,0,16);
  treadmillData.instantaneousSpeed = (uint16_t)(fSpeedConst * 160.934);
  
  
  ////////////////////////fGrade//////////////////////
  
 
  treadmillData.inclination = (int16_t)(fGrade * 10.0);
  float angle = atan(fGrade / 100.0)*180.0/3.14159;
  treadmillData.rampAngleSetting = (int16_t)(angle * 10.0);
  
  ////////////////////averageSpeed///////////////////////
  float fAverageSpeed = 6.0;
  uint16_t nAverageSpeed = (uint16_t)(fAverageSpeed * 160.934);
  
  ///////////TOTAL DISTANCE///////////////////////
  uint32_t td = fIntegratedDistance * 1609.34;  //in meters;
  
  treadmillData.totalDistance[0] = (byte)td & 0xff;
  treadmillData.totalDistance[1] = (byte)(td >> 8) & 0xff;
  treadmillData.totalDistance[2] = (byte)(td >> 16) & 0xff;

  //treadmillData.totalDistance[0] = 0;
  //treadmillData.totalDistance[1] = 0;
  //treadmillData.totalDistance[2] = 0;
  
  /////////////ELEVATION GAIN////////////
  treadmillData.positiveElevationGain = fElevationGain * 0.3048 * 10.0;   //feet to meters * 10
  treadmillData.negativeElevationGain = 0; 
  
  /////////HEART RATE///////////////////////
  treadmillData.heartRate = nHeartRate;

  
  //////////////////////METS////////////////////
  treadmillData.metabolicEquivalent = (byte)(fRunningMets() * 10);


  //////////////////////KCAL////////////////////
  treadmillData.totalEnergy = (uint16_t)(fTotalCalories * 10);

  
  
  
  
  
   //printTreadmillData();
   treadmillDataCharacteristic.setValue((byte *)&treadmillData,sizeof(treadmillData));
   treadmillDataCharacteristic.notify();
  
  

  

  
  

}

void loop() 
{
  //Serial.println("Looping");
  static unsigned long lastPaceCheck = millis();
  
  static bool onOff = false;
  //static RgbColor black(0,0,0);
  static unsigned long lastBTUpdate = millis();
  
  static float lastSpeed = 0;
  static float lastGrade = 0;
  pollSerial();
  
  if (lastSpeed != fSpeed)
  {
    lastSpeed = fSpeed;
    //Serial.print("fSpeed: ");
    //Serial.println(fSpeed);
  
  }
  
  if (lastGrade != fGrade)
  {
    lastGrade = fGrade;
    
    //Serial.print("fGrade: ");
    //Serial.println(fGrade);
  
  }
  integrateStuff();
  //int nSecs = millis() / 1000;
  //fSpeed = (nSecs % 100) / 10.0;
  //fGrade = (nSecs % 150) / 10.0;
  updateBT();
  
  
  if (fCadence > 0) 
  {
    long fmsecPace = 60000 / fCadence;
    if (fSpeed > 8.4) fmsecPace = 60000 / fCadenceFast;
    if (fSpeed > 1)
    {
      if (millis() - lastPaceCheck > fmsecPace)
    
      {
        //Serial.println(millis() - lastPaceCheck);
        lastPaceCheck = millis();
        onOff = !onOff;
        if (onOff) noise(30,2000);
        else noise(30,1000);  
        
      }  
      
      
        
    }
  }
  
  
  
}
