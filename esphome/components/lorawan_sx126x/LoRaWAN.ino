#include <Arduino.h>

#include <LoRaWan-Arduino.h>
#include <SPI.h>

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */

#define LORAWAN_APP_DATA_BUFF_SIZE 64  /**< Size of the data to be transmitted. */
#define LORAWAN_APP_TX_DUTYCYCLE 10000 /**< Defines the application data transmission duty cycle. 10s, value in [ms]. */
#define APP_TX_DUTYCYCLE_RND 1000	   /**< Defines a random delay for application data transmission duty cycle. 1s, value in [ms]. */
#define JOINREQ_NBTRIALS 3			   /**< Number of trials for the join request. */

hw_config hwConfig;

int PIN_LORA_RESET = 12;	 // LORA RESET
int PIN_LORA_NSS = 8;	 // LORA SPI CS
int PIN_LORA_SCLK = 9;	 // LORA SPI CLK
int PIN_LORA_MISO = 11;	 // LORA SPI MISO
int PIN_LORA_DIO_1 = 14; // LORA DIO_1
int PIN_LORA_BUSY = 13;	 // LORA SPI BUSY
int PIN_LORA_MOSI = 10;	 // LORA SPI MOSI
int RADIO_TXEN = -1;	 // LORA ANTENNA TX ENABLE
int RADIO_RXEN = -1;	 // LORA ANTENNA RX ENABLE

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void send_lora_frame(void);
static uint32_t timers_init(void);

// APP_TIMER_DEF(lora_tx_timer_id);                                              ///< LoRa tranfer timer instance.
TimerEvent_t appTimer;														  ///< LoRa tranfer timer instance.
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {
    BoardGetBatteryLevel,
    BoardGetUniqueId,
    BoardGetRandomSeed,
    lorawan_rx_handler,
    lorawan_has_joined_handler,
    lorawan_confirm_class_handler,
    lorawan_join_failed_handler
};

//uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x34, 0x61};
//uint8_t nodeAppEUI[8]    = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
//uint8_t nodeAppKey[16] = {0xAE, 0xC5, 0xB2, 0x64, 0x39, 0x00, 0x69, 0xF8, 0x7A, 0x09, 0xDA, 0xC5, 0x73, 0x0F, 0x6B, 0xED};
// uint8_t nodeAppKey[16]   = {0x3B, 0xB6, 0xF6, 0xC2, 0x3A, 0xDF, 0x99, 0xB8, 0x0E, 0xDA, 0x9F, 0xCE, 0xE2, 0x28, 0xFF, 0x1B};
// uint8_t nodeAppKey[16]   = {0x1B, 0xFF, 0x28, 0xE2, 0xCE, 0x9F, 0xDA, 0x0E, 0xB8, 0x99, 0xDF, 0x3A, 0xC2, 0xF6, 0xB6, 0x3B};

//uint8_t nodeNwsKey[16] = {0xAE, 0xC5, 0xB2, 0x64, 0x39, 0x00, 0x69, 0xF8, 0x7A, 0x09, 0xDA, 0xC5, 0x73, 0x0F, 0x6B, 0xED};
//uint32_t nodeDevAddr = 0x260116F8;
//uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

//Set2
uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x35, 0x4C};
uint8_t nodeAppEUI[8]    = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
uint8_t nodeAppKey[16]   = {0x4A, 0xB7, 0xA2, 0xE4, 0xDC, 0xCC, 0xE2, 0x13, 0xDE, 0x71, 0xE5, 0xB2, 0x19, 0x83, 0x71, 0x69};
//uint8_t nodeNwsKey[16]   = {0x38, 0x08, 0xF5, 0xB9, 0x23, 0x3F, 0xDE, 0x96, 0x5D, 0x79, 0x98, 0xD6, 0x63, 0x7F, 0x45, 0x66};

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Define the HW configuration between MCU and SX126x
  hwConfig.CHIP_TYPE = SX1262_CHIP;		  // Example uses an eByte E22 module with an SX1262
  hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
  hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;	  // LORA SPI CS
  hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;	  // LORA SPI CLK
  hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;	  // LORA SPI MISO
  hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
  hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;	  // LORA SPI BUSY
  hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;	  // LORA SPI MOSI
  hwConfig.RADIO_TXEN = RADIO_TXEN;		  // LORA ANTENNA TX ENABLE
  hwConfig.RADIO_RXEN = RADIO_RXEN;		  // LORA ANTENNA RX ENABLE
  hwConfig.USE_DIO2_ANT_SWITCH = true;	  // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = true;			  // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;	  // Only Insight ISP4520 module uses DIO3 as antenna control

  // Initialize Serial for debug output
  Serial.begin(115200);

  Serial.println("=====================================");
  Serial.println("SX126x LoRaWAN");
  Serial.println("=====================================");

  // Initialize Scheduler and timer
  Serial.println("timers_init()");
  uint32_t err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
  }

  // Initialize LoRa chip.
  Serial.println("lora_hardware_init()");
  err_code = lora_hardware_init(hwConfig);
  if (err_code != 0)
  {
    Serial.printf("lora_hardware_init failed - %d\n", err_code);
  }

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);
  //lmh_setNwkSKey(nodeNwsKey);
  
  //lmh_setAppSKey(nodeAppsKey);
  //lmh_setDevAddr(nodeDevAddr);

  // Initialize LoRaWan
  Serial.println("lmh_init()");

  Serial.print("nodeDeviceEUI: ");
  for(int i=0; i<8; i++) {
    Serial.printf("%02X ", nodeDeviceEUI[i]);
  }
  Serial.println();

  Serial.print("nodeAppEUI:    ");
  for(int i=0; i<8; i++) {
    Serial.printf("%02X ", nodeAppEUI[i]);
  }
  Serial.println();

  Serial.print("nodeAppKey:    ");
  for(int i=0; i<16; i++) {
    Serial.printf("%02X ", nodeAppKey[i]);
  }
  Serial.println();
  
  err_code = lmh_init(&lora_callbacks, lora_param_init, true, CLASS_A, LORAMAC_REGION_AU915);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
  }

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use either
  // lmh_setSingleChannelGateway
  // or
  // lmh_setSubBandChannels
  //
  // DO NOT USE BOTH OR YOUR COMMUNICATION WILL MOST LIKELY NEVER WORK
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Setup connection to a single channel gateway
  // lmh_setSingleChannelGateway(0, DR_3);

  // For some regions we might need to define the sub band the gateway is listening to
  // This must be called AFTER lmh_init()
  /// \todo This is for Dragino LPS8 gateway. How about other gateways???
if (!lmh_setSubBandChannels(2))
  {
   Serial.println("lmh_setSubBandChannels failed. Wrong sub band requested?");
  }

  // Start Join procedure
  Serial.println("lmh_join()");
  lmh_join();
}

void loop()
{
  // Handle Radio events
  Radio.IrqProcess();

  // We are on FreeRTOS, give other tasks a chance to run
  // delay(100);
}

/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OVER_THE_AIR_ACTIVATION failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief LoRa function for handling HasJoined event.
*/
static void lorawan_has_joined_handler(void)
{
#if (OVER_THE_AIR_ACTIVATION != 0)
  Serial.println("Network Joined");
#else
  Serial.println("OVER_THE_AIR_ACTIVATION != 0");

#endif
  lmh_class_request(CLASS_A);

  TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
  TimerStart(&appTimer);
  // app_timer_start(lora_tx_timer_id, APP_TIMER_TICKS(LORAWAN_APP_TX_DUTYCYCLE), NULL);
}

/**@brief Function for handling LoRaWan received data from Gateway

   @param[in] app_data  Pointer to rx data
*/
static void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

  switch (app_data->port)
  {
    case 3:
      // Port 3 switches the class
      if (app_data->buffsize == 1)
      {
        switch (app_data->buffer[0])
        {
          case 0:
            lmh_class_request(CLASS_A);
            break;

          case 1:
            lmh_class_request(CLASS_B);
            break;

          case 2:
            lmh_class_request(CLASS_C);
            break;

          default:
            break;
        }
      }
      break;

    case LORAWAN_APP_PORT:
      // YOUR_JOB: Take action on received data
      break;

    default:
      break;
  }
}

static void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);

  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = LORAWAN_APP_PORT;
  lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

static void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    Serial.println("Did not join network, skip sending frame");
    return;
  }

  uint32_t i = 0;
  m_lora_app_data.port = LORAWAN_APP_PORT;
  m_lora_app_data.buffer[i++] = 'H';
  m_lora_app_data.buffer[i++] = 'e';
  m_lora_app_data.buffer[i++] = 'l';
  m_lora_app_data.buffer[i++] = 'l';
  m_lora_app_data.buffer[i++] = 'o';
  m_lora_app_data.buffer[i++] = ' ';
  m_lora_app_data.buffer[i++] = 'w';
  m_lora_app_data.buffer[i++] = 'o';
  m_lora_app_data.buffer[i++] = 'r';
  m_lora_app_data.buffer[i++] = 'l';
  m_lora_app_data.buffer[i++] = 'd';
  m_lora_app_data.buffer[i++] = '!';
  m_lora_app_data.buffsize = i;

  lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
  if (error == LMH_SUCCESS)
  {
  }
  Serial.printf("lmh_send result %d\n", error);
}

/**@brief Function for handling a LoRa tx timer timeout event.
*/
static void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
  TimerStart(&appTimer);
  Serial.println("Sending frame");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.

   @details Initializes the timer module. This creates and starts application timers.
*/
static uint32_t timers_init(void)
{
  appTimer.timerNum = 3;
  TimerInit(&appTimer, tx_lora_periodic_handler);

  // ret_code_t err_code;

  // APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  // // Initialize timer module.
  // err_code = app_timer_init();
  // VERIFY_SUCCESS(err_code);

  // // Initialize timers
  // err_code = app_timer_create(&lora_tx_timer_id, APP_TIMER_MODE_REPEATED, tx_lora_periodic_handler);
  // VERIFY_SUCCESS(err_code);

  return 0;
}
