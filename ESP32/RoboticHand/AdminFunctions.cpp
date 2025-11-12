#include "Header.h"
#include "PinDefinitions.h"
#include "IMUAddresses.h"
#include "MuxAddresses.h"
#include "AHRSAddresses.h"

// VARIABLES -------------------------------------------
volatile bool Output = false;
volatile bool pollSens = false;
volatile bool CALIB1 = false;
volatile bool CALIB2 = false;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLECharacteristic *pWriteCharacteristic = nullptr; // add global pointer
bool deviceConnected = false;

String receivedCommand = "";
bool commandReady = false;

float dataBuffer[39] = {0};

// FUNCTIONS -------------------------------------------
// Server callbacks
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("BLE client connected!");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("BLE client disconnected!");
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar) override
  {
    String cmd = pChar->getValue();
    cmd.trim(); // remove stray spaces / newlines
    if (cmd.length() > 0)
    {
      Serial.print("Received BLE command: ");
      Serial.println(cmd);

      // Store command in global variable
      receivedCommand = cmd;
      commandReady = true;
    }
  }
};

// Startup message of commands
void startScreen()
{
  Serial.print("===================================================================== \n");
  Serial.print("---------------------LIST OF AVAILABLE COMMANDS:--------------------- \n");
  Serial.print("===================================================================== \n");
  Serial.print("FREE:  Allow data to be output to the terminal.\n");
  Serial.print("LOCK:  Prevent data from being output to the terminal. \n");
  Serial.print("ADDR:  Get the addresses of all sensors connected to board. \n");
  Serial.print("POLL:  Start/Stop the sensor from polling continuously. \n");
  Serial.print("HELP:  Print start screen again. \n");
  Serial.print("===================================================================== \n");
}

Commands parseCommand(const String &cmd)
{
  if (cmd == "ADDR")
    return CMD_ADDR;
  if (cmd == "FREE")
    return CMD_FREE;
  if (cmd == "LOCK")
    return CMD_LOCK;
  if (cmd == "POLL")
    return CMD_POLL;
  if (cmd == "HELP")
    return CMD_HELP;
  if (cmd == "CALIB1")
    return CMD_CALIB1;
  if (cmd == "CALIB2")
    return CMD_CALIB2;
  return CMD_UNKNOWN;
}

void CommandChecker(const String &cmd)
{
  switch (parseCommand(cmd))
  {
  case CMD_ADDR:
    I2C_AddressFinder();
    break;
  // case CMD_FREE:
  //     Output = true;
  //     Serial.print("Terminal opened. \n");
  //     delay(1000);
  //     break;
  // case CMD_LOCK:
  //     Output = false;
  //     Serial.print("Terminal locked. \n");
  //     break;
  case CMD_POLL:
    pollSens = !pollSens;
    // Serial.print(pollSens ? "Polling opened.\n" : "Polling locked.\n");
    break;
  case CMD_HELP:
    startScreen();
    break;

  case CMD_CALIB1:
    if (CALIB1 == true)
    {
      CALIB1 = false;
    }
    else
    {
      muxStartup();
      CALIB1 = true;
    }
    break;

  case CMD_CALIB2:
    if (CALIB2 == true)
    {
      CALIB2 = false;
    }
    else
    {
      CALIB2 = true;
    }
    break;

  default:
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    break;
  }
}

// Transmission to sensor / mux
void SensorTransmit(uint8_t sensor_addr, uint8_t register_addr, uint8_t message, uint16_t wait_ms)
{
  Wire.beginTransmission(sensor_addr);
  Wire.write(register_addr);
  Wire.write(message);
  Wire.endTransmission();
  delay(wait_ms);
}

uint8_t SensorReceive(uint8_t sensor_addr, uint8_t reg_addr)
{
  Wire.beginTransmission(sensor_addr); // Start communication
  Wire.write(reg_addr);                // Tell sensor which register we want
  Wire.endTransmission(false);         // Send, but keep connection for read

  Wire.requestFrom((int)sensor_addr, 1); // Ask for 1 byte back
  if (Wire.available())
  {
    return Wire.read(); // Return received byte
  }
  else
  {
    return 0; // Return 0 if nothing received (error case)
  }
}

// Read sensor data from registers (single I2C bus version)
int16_t read16(uint8_t sensor_addr, uint8_t reg)
{
  Wire.beginTransmission(sensor_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(sensor_addr, (uint8_t)2);

  if (Wire.available() < 2)
    return 0; // safety check

  uint8_t l = Wire.read();
  uint8_t h = Wire.read();
  return (int16_t)(h << 8 | l);
}

// Initialise I2C protocol
void I2C_init(uint8_t I2C_data, uint8_t I2C_clock)
{
  Serial.begin(115200);
  Wire.begin(I2C_data, I2C_clock);
  Wire.setClock(100000);
}

void I2C_AddressFinder()
{
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++)
  {                                  // Loop through all addresses
    Wire.beginTransmission(address); // Sends START condition
    error = Wire.endTransmission();  // Tries to complete transmission
    if (error == 0)
    { // Device is present (device ACKs)
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
}

void BluetoothSerialStart()
{
  // Initialize BLE
  BLEDevice::init("ESP32S3_BLE"); // BLE device name

  // Set larger MTU for more efficient transmission
  // BLEDevice::setMTU(247); // Maximum MTU size (244 bytes payload)

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // --- Notify characteristic ---
  pCharacteristic = pService->createCharacteristic(
      NOTIFY_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902()); // allow notify subscriptions

  // --- Write characteristic ---
  pWriteCharacteristic = pService->createCharacteristic(
      WRITE_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pWriteCharacteristic->setCallbacks(new CommandCallbacks()); // handle commands

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for BLE client to connect...");
}

void BluetoothTransmit()
{
  int chunkSizes[] = {21, 18}; // two chunks: palm+F1+F2, F3+F4+F5
  int index = 0;

  for (int chunk = 0; chunk < 2; chunk++)
  {
    int size = chunkSizes[chunk];
    char msg[300]; // make sure this is large enough for the bigger chunks
    int offset = 0;

    for (int i = 0; i < size; i++)
    {
      offset += snprintf(msg + offset, sizeof(msg) - offset, "%.3f", dataBuffer[index]);
      index++;

      if (i < size - 1)
      {
        offset += snprintf(msg + offset, sizeof(msg) - offset, ",");
      }
    }

    msg[offset] = '\0';
    pCharacteristic->setValue(msg);
    pCharacteristic->notify(); // send this chunk
    delay(2);                  // smaller delay to prevent BLE overflow
  }
}
