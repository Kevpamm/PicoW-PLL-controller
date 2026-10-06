#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "GATT_service/service_implementation.h"
#include <string.h>
#include <assert.h>
#include <math.h>

#include <stdio.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "dc_supply/digipot_config.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

typedef struct
{
    bool is_active;
    bool led_state;
    int32_t delay_in_millisec;
    int8_t toggles_left;
    int32_t last_toggle_in_millisec;
    int32_t wait_time_in_millisec;
    int32_t last_cycle_in_millisec;
} LedController;

// This defines how INT and FRAC of the N divider are calculated
#define CONRAD_PLL_MATH // Conrad ADF1549 configuration
// #define KEVIN_PLL_MATH // Kevin ADF1549 configuration. Arguably more optimized

#if defined CONRAD_PLL_MATH
volatile uint32_t frequencyToPLL_inHz = 920000000; // in hz 900000000
uint32_t intVal = 36;                              // set to 900MHz initially
uint32_t fracVal = 0;

uint32_t muxVal = 0b0110;
uint32_t rampOn = 0;

uint32_t phaseAdj = 0;
uint32_t phaseVal = 0;
// r3
uint32_t negBld = 0b101;

uint32_t R0 = 0x30120000;
uint32_t R1 = 0x1;
uint32_t R2 = 0x721000A;
uint32_t R3 = 0x1430083;
uint32_t R41 = 0x180104;
uint32_t R42 = 0x180144;
uint32_t R51 = 0x5;
uint32_t R52 = 0x800005;
uint32_t R61 = 0x6;
uint32_t R62 = 0x800006;
uint32_t R7 = 0x7;

#elif defined KEVIN_PLL_MATH
// I MUST SET REFERENCE DOUBLER DB20 BIT ON
uint32_t frequencyToPLL_inHz = 920000000; // in hz 900000000
uint32_t intVal = 23;                     // set to 900MHz initially
uint32_t fracVal = 0;

uint32_t muxVal = 0b0110;
uint32_t rampOn = 0;

uint32_t phaseAdj = 0;
uint32_t phaseVal = 0;
// r3
uint32_t negBld = 0b110;

uint32_t R0 = 0x280B8000;
uint32_t R1 = 0x1;
uint32_t R2 = 0x721800A;
uint32_t R3 = 0x1830083;
uint32_t R41 = 0x180104;
uint32_t R42 = 0x180144;
uint32_t R51 = 0x5;
uint32_t R52 = 0x800005;
uint32_t R61 = 0x6;
uint32_t R62 = 0x800006;
uint32_t R7 = 0x7;

#endif

// Blue is ground
#define DATA_PIN 19  // red
#define CLOCK_PIN 18 // orange
#define LATCH_PIN 17 // yello

#define BORN_PIN1 13 // white
#define BORN_PIN2 14 // Brown
#define BORN_PIN3 15 // Green

#define SR_OE_PIN 9     // Black
#define SR_DATA_PIN 10  // red
#define SR_CLOCK_PIN 11 // orange
#define SR_LATCH_PIN 12 // yellow

#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_BAUDRATE 400000

// ADVERTISEMENT FLAGS
#define APP_AD_FLAGS 0x06 // This flag is for General Discoverable in advertising data, meaning everyone can discover our device and advertising_data
#define BUFFER_SIZE 100
#define BANDS 8
// CONNECTED BLE FLASG
volatile bool TESTING_FLAG = false;

volatile bool power_down_pll_flag = false;
volatile bool frequency_receipt_status = false;
volatile bool BLE_IS_CONNECTED = false;
volatile bool control_receipt_status = false;
volatile bool send_all_registers_flag = false;
volatile bool send_freq_registers_flag = false;
volatile bool restore_default_registers_flag = false;
volatile bool register_notification_first_on = false;
volatile bool hop_command_flag = false;
volatile bool led_flag = false;
volatile bool changeR2_flag = false;
volatile bool default_dc_mode_flag = false;
volatile bool power_saving_mode_flag = false;

volatile bool resistor_code_flag = false;
volatile bool receive_pin_1_flag = false;
volatile bool receive_pin_3_flag = false;
volatile bool receive_pin_12_flag = false;

volatile bool enable_command_flag = false;
volatile bool disable_pin_3_flag = false;
volatile bool disable_pin_12_flag = false;
volatile bool enable_pin_3_flag = false;
volatile bool enable_pin_12_flag = false;

bool POWER_STATUS = true;
bool is_hopping = false;
bool hop = false;
bool hop_complete = false;
bool FIRST_CONNECTION = true;
bool pin_1_second_time_send = false;
bool pin_3_second_time_send = false;
bool pin_12_second_time_send = false;

volatile uint32_t frequencyFromClient_inHz = 0;
const uint32_t defaultFrequency_inHz = 920000000; // 920 MHz
static uint8_t shift_register_state = 0b00000011;
// bool freqHopFlag = false;
// bool wasHopping = false;         // cleanup flag
// uint32_t fhDelay = 1000000;      // in us (microseconds)
// uint32_t fhStep = 50 * 1000000;  // in hz
// uint32_t fhSpan = 300 * 1000000; // in hz
// uint32_t fhStart = 90000000;     // default updated each freq send

volatile bool hop_delay_time_receipt_status = false;
volatile bool hop_step_frequency_receipt_status = false;
volatile bool span_frequency_receipt_status = false;
volatile int32_t delayTime_inMillisec = 0;
volatile uint32_t stepFrequency_inHz = 0;
volatile uint32_t spanFrequency_inHz = 0;
volatile uint32_t stopFrequency_inHz = 0;
volatile uint16_t charge_pump_current = 2500;
volatile uint8_t i2c_packet_to_send[3];

uint64_t lastHop = 0;

// NEW SUPPORTED FREQUENCY BANDS in Hz
const uint32_t frequencyBands[BANDS][2] = {{920000000, 949000000}, {970000000, 996000000}, {1074000000, 1106000000}, {1209000000, 1266000000}, {1270000000, 1323000000}, {1442000000, 1514000000}, {1695000000, 1811000000}, {2142000000, 2401000000}};

// OLD SUPPORTED FREQUENCY BANDS in MHz
// const uint32_t frequencyBands[BANDS][2] = {{890, 922},{951, 997},{1039, 1103},{1140,1204},{1196,1285},{1305,1406},{1524,1709},{1815,2105}};
const bool bornSets[BANDS][3] = {{1, 1, 1}, {1, 1, 0}, {0, 1, 1}, {0, 1, 0}, {1, 0, 1}, {1, 0, 0}, {0, 0, 1}, {0, 0, 0}};

// FUNCTION DEFINITION
void shiftOutFast(uint8_t val);
static int pico_led_init(void);
static void pico_set_led(bool led_on);
void startLed(LedController *led_controller_ptr, uint8_t repetition, int32_t delay_in_milli_sec, int64_t wait_in_millisec);
void updateLed(LedController *led_controller_ptr);
void calculateIntFrac(void);
void updateR0(void);
void sendPLLFreqRegisters(void);
void latchFast(void);
void updateR1(void);
void updateR2(volatile uint32_t *frequency_ptr);
void updateR2Debugging(volatile uint16_t *current_in_micro_amp);
void updateR3(volatile bool *power_down);
void changeBorn(int bandInput);
void storeRegisterValue(uint8_t *buffer, uint32_t *registerValues, uint16_t NumOfRegisters);
void sendPLLAllRegisters(void);
void restoreAllValues();
void frequencyHopOnce();
void i2c_setup(void);
bool i2c_write_register(volatile uint8_t *handler, bool nostop_or_not);
bool digipots_init(bool on_set_up);
bool i2c_write_ACR_register(volatile uint8_t *device_address, bool is_volatile);
bool i2c_write_ACR_register_non_vol_variable(uint8_t *device_address, bool is_volatile);
bool digipots_power_saving_mode(void);
void ldo_shift_register_init(void);
// void shift_register_write(uint8_t value);
void shift_register_write(uint8_t value);
// void ldo_set_enable(uint8_t output, bool enable);
// void handle_enable_command(uint8_t command);
/*********************************************************************************************************************************
 * THIS IS THE DATA PACKET THAT WE ADVERTISE
 * Bluetooth clients (laptops) and scanners discover this packet and learn info about our PICO W & its BLE service UUID
 *
 * 1st line:
 *      0x02: the next chunk is 2 bytes in size
 *      BLUETOOTH_DATA_TYPE_FLAGS: the next data is a flag that tells basic bluetooth type
 *      APP_AD_FLAGS: the advertising data is general discoverable, meaning anyone can see it.
 *
 * 2nd line:
 *      0x09: the next chunk is 9 bytes in size
 *      BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME: the next data is a complete name of the BLE server (Pico W)
 *      'P', 'L', 'L', etc: the name of the BLE server. This name will show up on the client's screen pre connecting
 *
 * 3rd line:
 *      0x11: the next chung is 17 bytes in size
 *      BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS : The following data is a 128 bit UUID for the service
 *
 ********************************************************************************************************************************/
static uint8_t advertising_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,

    0x09, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'L', 'L', '-', 'P', 'I', 'C', 'O',

    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,

    // SERVICE_UUID in little endian
    0x9E, 0x39, 0xC8, 0x47, 0x21, 0x41, 0xF0, 0xB2, 0x71, 0x44, 0x1D, 0xA2,
    0x00, 0x20, 0xE1, 0x50

};

// The total size of the advertising packet
// We need this because ad packet can only be less than 32 bytes
// Also, when setting up the payload to send over the wire, the function will need to know the size of the packet
static const uint8_t advertising_data_length = sizeof(advertising_data);

static btstack_packet_callback_registration_t hci_event_callback_registration;
// HCI Packet Handler
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    /*********************************************************************************************************************
     * Packet_handler declaration requires these parameters, but we don't need them for the code below so we UNUSED() them
     *********************************************************************************************************************/
    UNUSED(size);
    UNUSED(channel);
    /*********************************************************************************
     * 'local_addr' is empty here, but will store the bluetooth address of the Pico W
     * 'bd_addr_t' is a type struct created by BTstack to hold Bluetooth address
     *  Bluetooth address is 6 bytes.  Kinda looks like 'A1:B2:C3:D4:E5:F6'
     *********************************************************************************/
    bd_addr_t local_addr;
    if (packet_type != HCI_EVENT_PACKET)
        return;

    // Retrive event type from HCI packet
    uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type)
    {

    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) // The Bluetooth system of the Pico W has not booted properly
            return;
        // 'gap_local_bd_addr' stores bluetooth address of the Pico W into 'local_addr'
        gap_local_bd_addr(local_addr);
        printf("BTstack  is up and running on %s.\n", bd_addr_to_str(local_addr));
        /*********************************************************
         * SET UP ADVERTISMENT TIME INTERVAL
         * BTstack unit for advertising time interval is 0.625 ms
         * 800 x 0.625 = 500 ms between advertisements
         **********************************************************/
        uint16_t advertising_int_min = 160;
        uint16_t advertising_int_max = 160;

        /*******************************************************************************************************
         * CHOOSING ADVERTISING TYPES
         * Type 0: Connectable Undirected - used for General Advertising, allows any other device to connect.
         * Type 1: Connectable Directed - requests for a particular device with known address to connect.
         * Type 2: Scannable Undirected - broadcasts advertising data to active scanners.
         * Type 3: Nonconnectable Undirected - just broadcasts advertising data. Don't bother connecting.
         *****************************************************************************************************/
        uint8_t advertising_type = 0;

        /*************************************************************************************************************
         * This is the Bluetooth address of the specific client to which we want the Pico W to connect.
         * With 'advertising_type = 0', we accept any devices, so null_addr doesn't matter and is set to all 0 with memset().
         * However, if we choose advertising_type = 1, null_addr must be filled with the BT address of the target client.
         **********************************************************************************************************/
        bd_addr_t null_addr;
        memset(null_addr, 0, 6);
        gap_advertisements_set_params(advertising_int_min, advertising_int_max, advertising_type, 0, null_addr, 0x07, 0x00);

        /**********************************************************************
         * Double check that the advertising data is no greater than 31 bytes
         * Because the limit of advertising data is 32 bytes
         ***********************************************************************/
        assert(advertising_data_length <= 31);

        // Load the payload before advertising
        gap_advertisements_set_data(advertising_data_length, (uint8_t *)advertising_data);

        // Start advertising.
        gap_advertisements_enable(1);

        break;

    case HCI_EVENT_LE_META:
        if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE)
        {
            BLE_IS_CONNECTED = true;
            printf("\n\t>> BLE client connected!\n");
            gap_advertisements_enable(0);
        }
        break;
    // Disconnected from a client
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        BLE_IS_CONNECTED = false;
        gap_advertisements_enable(1);
        FIRST_CONNECTION = true;
        break;

    // Ready to send ATT
    case ATT_EVENT_CAN_SEND_NOW:
        break;
    default:
        break;
    }
}

// THESE ARE THE BUFFERS HOLDING THE VALUE OF CHARACTERISTICS
// BUFFER_SIZE IS CURRENTLY VERY GENEROUS (100)
// REDUCE THE BUFFER SIZE WHEN ADDING OTHER FEATURES TO THE PICO
static uint8_t characteristic_FREQUENCY_tx[BUFFER_SIZE];
static uint8_t characteristic_CONTROL_tx[BUFFER_SIZE];
static uint8_t characteristic_HOP_tx[BUFFER_SIZE];
static uint8_t characteristic_REGISTER_tx[BUFFER_SIZE];
static uint8_t characteristic_LED_tx[BUFFER_SIZE];
static uint8_t characteristic_SETTING_tx[BUFFER_SIZE];
static uint8_t characteristic_RESISTOR_tx[BUFFER_SIZE];
static uint8_t characteristic_ENABLE_tx[BUFFER_SIZE];

bool freqHop_timer_callback(struct repeating_timer *t)
{
    if (!hop_command_flag || frequencyToPLL_inHz >= stopFrequency_inHz)
    {
        is_hopping = false;
        hop_command_flag = false;
        hop_complete = true;
        return false; // Stop the timer
    }

    hop = true;
    return true; // continue the timer
}

int main()
{

    // Initialize stdio
    stdio_init_all();

    // Make the Pico wait for 3 seconds for users to set up the Serial Monitor
    sleep_ms(3000);

    gpio_init(DATA_PIN);
    gpio_set_dir(DATA_PIN, GPIO_OUT);

    gpio_init(CLOCK_PIN);
    gpio_set_dir(CLOCK_PIN, GPIO_OUT);

    gpio_init(LATCH_PIN);
    gpio_set_dir(LATCH_PIN, GPIO_OUT);

    gpio_init(BORN_PIN1);
    gpio_set_dir(BORN_PIN1, GPIO_OUT);

    gpio_init(BORN_PIN2);
    gpio_set_dir(BORN_PIN2, GPIO_OUT);

    gpio_init(BORN_PIN3);
    gpio_set_dir(BORN_PIN3, GPIO_OUT);

    gpio_put(BORN_PIN1, 0);
    gpio_put(BORN_PIN2, 0);
    gpio_put(BORN_PIN3, 0);

    gpio_put(DATA_PIN, 0);
    gpio_put(CLOCK_PIN, 0);
    gpio_put(LATCH_PIN, 0);
    // gpio_put(OE_PIN, 1);
    ldo_shift_register_init();

    if (pico_led_init())
    {
        printf("Failed to initialize cyw43_arch!\n");
        return -1;
    }

    // Initialize L2CAP and Security Manager
    l2cap_init();
    sm_init();

    // Initialize ATT server
    att_server_init(profile_data, NULL, NULL);

    // Instantiate our PLL Service Handler
    PLL_service_server_init(characteristic_FREQUENCY_tx,
                            characteristic_CONTROL_tx,
                            characteristic_HOP_tx,
                            characteristic_REGISTER_tx,
                            characteristic_LED_tx,
                            characteristic_SETTING_tx,
                            characteristic_RESISTOR_tx,
                            characteristic_ENABLE_tx);

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Register for ATT event
    att_server_register_packet_handler(packet_handler);

    // TURN THE BLUETOOTH ON!
    hci_power_control(HCI_POWER_ON);

    // Initiate I2C
    i2c_setup();

    // Set the digipots in DC Supply module the default values
    // Also toggle the flags to signal that in the next time sending data to those digipot,
    // users needs to send ACR Reg first, setting to volatile.
    if (digipots_init(true))
    {
        pin_1_second_time_send = true;
        pin_3_second_time_send = true;
        pin_12_second_time_send = true;
    };

    // Store default Frequency = 920 MHz into the Frequency Buffer
    storeFrequency(characteristic_FREQUENCY_tx, defaultFrequency_inHz);

    // Store default register values into Register buffer
    // Once scale up the system and many code is added to the pico
    // Avoid declaring buffer like this. Instead, write directly to the Buffer
    uint32_t AllRegisterValues[13] = {intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7};
    storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 13);

    struct repeating_timer timer;
    LedController led_controller = {0};
    while (1)
    {
        updateLed(&led_controller);

        if (BLE_IS_CONNECTED && FIRST_CONNECTION)
        {

            printf("\nDefault Frequency Buffer: ");
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                printf("%X ", characteristic_FREQUENCY_tx[i]);
            }
            printf("\n");

            notify_register_characteristic();
            printf("Default Register buffer: ");
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                printf("%X ", characteristic_REGISTER_tx[i]);
            }
            printf("\n");
            printf("Pico-W is waiting for the client to enable Register Notification...");

            while (!register_notification_first_on)
            {
                printf(".");
                pico_set_led(true);
                sleep_ms(500);
                pico_set_led(false);
                sleep_ms(500);
            }
            notify_register_characteristic();
            FIRST_CONNECTION = false;
        }
        if (!BLE_IS_CONNECTED)
        { // !BLE_IS_CONNECTED
            printf("\n\t>> BLE is disconnected from previous client. \n");
            printf("Pico-W is waiting for a client to connect...");
            // FLASH LIGHTS
            while (!BLE_IS_CONNECTED)
            {
                pico_set_led(true);
                printf(".");
                sleep_ms(500);
                pico_set_led(false);
                sleep_ms(500);
            }
        }

        if (frequency_receipt_status == true)
        {
            frequencyToPLL_inHz = frequencyFromClient_inHz;
            printf("\nFrequency Buffer after receiving frequency 1 %u: ", frequencyToPLL_inHz);
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                printf("%X ", characteristic_FREQUENCY_tx[i]);
            }
            printf("\n");

            for (int i = 0; i < BANDS; i++)
            { // check input against valid frequencies
                if (frequencyToPLL_inHz >= (frequencyBands[i][0]) && frequencyToPLL_inHz <= (frequencyBands[i][1]))
                {
                    calculateIntFrac();
                    updateR0();
                    updateR1();
                    // printf("R1: %X\n", R1);
                    updateR3(&power_down_pll_flag);
                    changeBorn(i);
                    sendPLLFreqRegisters();
                    startLed(&led_controller, 2, 300, 0);
                    // if (i == 0 || i == 1 || i == 2 || i == 4)
                    // {
                    //     sendPLLFreqRegisters();
                    //     startLed(&led_controller, 2, 300, 0);
                    // }
                    // else
                    // {
                    //     updateR2(&frequencyToPLL_inHz);
                    //     sendPLLAllRegisters();
                    //     startLed(&led_controller, 6, 300, 0);
                    // }
                    uint32_t AllRegisterValues[13] = {intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7}; // No need to update all 13. only three values in the buffer are changed: R0, R1, R3
                    storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 13);
                    // printf("\nCharacteristic Buffer after receiving frequency %u: ", frequencyToPLL_inHz);
                    // for (int i = 0; i < BUFFER_SIZE; i++)
                    // {
                    //     printf("%X ", characteristic_REGISTER_tx[i]);
                    // }
                    frequency_receipt_status = false;
                    if (hop_command_flag)
                    {
                        hop_command_flag = false;
                    }
                    break; // exit loop
                }
            }
            if (frequency_receipt_status == true)
            {
                printf("\n\t>> Pico-W received an unsupported frequency band from the User Interface.");
                printf("\n\t>> Frequency and Register is restored to default.\n");
                restoreAllValues();
                frequency_receipt_status = false;
            }
        }

        if (hop_step_frequency_receipt_status == true)
        {
            printf("Step Frequency received: %u\n", stepFrequency_inHz);
            hop_step_frequency_receipt_status = false;
        }
        if (hop_delay_time_receipt_status)
        {
            printf("Delay time receive: %u\n", delayTime_inMillisec);
            hop_delay_time_receipt_status = false;
        }
        if (span_frequency_receipt_status)
        {
            printf("Span receive: %u\n", spanFrequency_inHz);
            span_frequency_receipt_status = false;
        }

        if (hop_command_flag && !is_hopping)
        {
            printf("Added timer\n");
            int32_t delayTime_for_callback = -delayTime_inMillisec;
            printf("delay time that goes in the timer: %d\n", delayTime_for_callback);
            add_repeating_timer_ms(delayTime_for_callback, freqHop_timer_callback, NULL, &timer);
            is_hopping = true;
        }
        if (hop == true)
        {
            hop = false;
            frequencyHopOnce();
        }
        if (hop_complete)
        {
            printf("==HOP IS ENDED==\n");
            printf("ToSendFreq: %u\n", frequencyToPLL_inHz);
            storeFrequency(characteristic_FREQUENCY_tx, frequencyToPLL_inHz);
            notify_frequency_characteristic();
            uint32_t AllRegisterValues[6] = {intVal, fracVal, R0, R1, R2, R3};
            storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 6);
            hop_complete = false;
        }

        if (send_all_registers_flag == true)
        {
            sendPLLAllRegisters();
            startLed(&led_controller, 6, 300, 0);
            send_all_registers_flag = false;
            uint32_t AllRegisterValues[13] = {intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7}; // No need to update all 13. only three values in the buffer are changed: R0, R1, R3
            storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 13);
        }
        if (send_freq_registers_flag == true)
        {
            sendPLLFreqRegisters();
            startLed(&led_controller, 2, 300, 0);
            send_freq_registers_flag = false;
            uint32_t AllRegisterValues[13] = {intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7}; // No need to update all 13. only three values in the buffer are changed: R0, R1, R3
            storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 13);
        }
        if (changeR2_flag)
        {
            updateR2Debugging(&charge_pump_current);
            changeR2_flag = false;
        }
        if (restore_default_registers_flag == true)
        {
            restoreAllValues();
            restore_default_registers_flag = false;
        }

        if (power_down_pll_flag == true && POWER_STATUS)
        {
            updateR3(&power_down_pll_flag);
            sendPLLFreqRegisters();
            startLed(&led_controller, 4, 150, 0);
            POWER_STATUS = false;
        }
        else if (!power_down_pll_flag && !POWER_STATUS)
        {
            updateR3(&power_down_pll_flag);
            sendPLLFreqRegisters();
            startLed(&led_controller, 4, 150, 0);
            POWER_STATUS = true;
            printf("lo ");
        }
        if (default_dc_mode_flag)
        {
            if (digipots_init(false) == false)
            {
                printf("Failed to return digipots to defaults");
            }
            default_dc_mode_flag = false;
        }
        if (resistor_code_flag)
        {
            if (receive_pin_1_flag)
            {
                if (pin_1_second_time_send)
                {
                    i2c_write_ACR_register(i2c_packet_to_send, true);
                    i2c_write_register(i2c_packet_to_send, false);
                    pin_1_second_time_send = false;
                }
                else
                {
                    i2c_write_register(i2c_packet_to_send, false);
                }
                receive_pin_1_flag = false;
            }
            else if (receive_pin_3_flag)
            {
                if (pin_3_second_time_send)
                {
                    i2c_write_ACR_register(i2c_packet_to_send, true);
                    i2c_write_register(i2c_packet_to_send, false);
                    pin_3_second_time_send = false;
                }
                else
                {
                    i2c_write_register(i2c_packet_to_send, false);
                }
                receive_pin_3_flag = false;
            }
            else if (receive_pin_12_flag)
            {
                if (pin_12_second_time_send)
                {
                    i2c_write_ACR_register(i2c_packet_to_send, true);
                    i2c_write_register(i2c_packet_to_send, false);
                    pin_12_second_time_send = false;
                }
                else
                {
                    i2c_write_register(i2c_packet_to_send, false);
                }
                receive_pin_12_flag = false;
            }
            else
            {
                printf("No flag set up for this pin. Currently in main.");
            }
            resistor_code_flag = false;
        }
        if (power_saving_mode_flag)
        {
            if (!digipots_power_saving_mode())
            {
                printf("Failed to switch to power-saving mode");
            }
            power_saving_mode_flag = false;
        }   
        if (enable_command_flag)
        {
            if (disable_pin_3_flag) {
                shift_register_state = shift_register_state & ~(1 << 0);
                disable_pin_3_flag = false;
                printf("Command: disable pin 3 / Q0\n");
            }
            else if (disable_pin_12_flag) {
                shift_register_state = shift_register_state & ~(1 << 1);
                disable_pin_12_flag = false;
                printf("Command: disable pin 12 / Q1\n");
            }
            else if (enable_pin_3_flag) {
                shift_register_state = shift_register_state | 0b00000001;
                enable_pin_3_flag = false;
                printf("Command: enable pin 3 / Q0\n");
            }
            else if (enable_pin_12_flag) {
                shift_register_state = shift_register_state | 0b00000010;
                enable_pin_12_flag = false;
                printf("Command: enable pin 12 / Q1\n");
            }
            printf("After: shift_register_state = 0x%02X\n", shift_register_state);
            shift_register_write(shift_register_state);
            enable_command_flag = false;
        }
    }
}

void shiftOutFast(uint8_t value)
{

    for (int i = 7; i >= 0; i--)
    {
        // Write bits to the data pin
        if (value & (1 << i))
        {
            gpio_set_mask(1 << DATA_PIN);
        }
        else
        {
            gpio_clr_mask(1 << DATA_PIN);
        }

        gpio_set_mask(1 << CLOCK_PIN);
        for (int i = 0; i < 2; i++)
        {
            asm volatile("nop");
        }

        gpio_clr_mask(1 << CLOCK_PIN);
        for (int i = 0; i < 2; i++)
        {
            asm volatile("nop");
        }
    }
}

static int pico_led_init(void)
{
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    return cyw43_arch_init();
#endif
}

static void pico_set_led(bool led_on)
{
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

void startLed(LedController *led_controller_ptr, uint8_t repetition, int32_t delay_in_milli_sec, int64_t wait_in_millisec)
{
    if (led_controller_ptr == NULL)
    {
        return;
    }
    if (repetition <= 0)
    {
        led_controller_ptr->is_active = false;
        led_controller_ptr->led_state = false;
        led_controller_ptr->toggles_left = 0;
        pico_set_led(led_controller_ptr->led_state);
        return;
    }
    led_controller_ptr->is_active = true;
    led_controller_ptr->led_state = true;
    led_controller_ptr->delay_in_millisec = delay_in_milli_sec;
    led_controller_ptr->toggles_left = repetition * 2 - 1;
    led_controller_ptr->last_toggle_in_millisec = to_ms_since_boot(get_absolute_time());
    led_controller_ptr->wait_time_in_millisec = wait_in_millisec;
    led_controller_ptr->last_cycle_in_millisec = 0;
    pico_set_led(led_controller_ptr->led_state);
}

void updateLed(LedController *led_controller_ptr)
{
    if (led_controller_ptr == NULL)
    {
        return;
    }
    if (!led_controller_ptr->is_active)
        return;
    int32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - led_controller_ptr->last_toggle_in_millisec >= led_controller_ptr->delay_in_millisec) &&
        (now - led_controller_ptr->last_cycle_in_millisec >= led_controller_ptr->wait_time_in_millisec))
    {
        led_controller_ptr->last_toggle_in_millisec = now;
        led_controller_ptr->led_state = !led_controller_ptr->led_state;
        pico_set_led(led_controller_ptr->led_state);
        led_controller_ptr->toggles_left--;
        if (led_controller_ptr->toggles_left <= 0)
        {
            led_controller_ptr->is_active = false;
            led_controller_ptr->led_state = false;
            pico_set_led(false);
            return;
        }
        if (!led_controller_ptr->led_state)
        {
            led_controller_ptr->last_cycle_in_millisec = to_ms_since_boot(get_absolute_time());
        }
    }
}

void calculateIntFrac(void)
{
#if defined(CONRAD_PLL_MATH)
    intVal = frequencyToPLL_inHz / 25000000;
    fracVal = (uint32_t)(round((double)(frequencyToPLL_inHz - (intVal * 25000000)) * 1.34217727));
#elif defined(KEVIN_PLL_MATH)
    intVal = frequencyToPLL_inHz / 40000000;
    fracVal = (uint32_t)(round((double)(frequencyToPLL_inHz - (intVal * 40000000)) * 0.8388608));
#endif
}

void updateR0(void)
{
    R0 = (rampOn << 31) | (muxVal << 27) | (intVal << 15) | ((fracVal >> 13) << 3);
}
void updateR1(void)
{
    R1 = (phaseAdj << 28) | ((fracVal << 19) >> 4) | (phaseVal << 3) + 0b1;
}
void updateR2(volatile uint32_t *frequency_ptr)
{
    // This variable is too make sure the pointer doesn't change value during R2 being updated
    // If Pico memory is extremely tight, no need for this temp variable
    uint32_t frequency = *frequency_ptr;
    if (frequency <= 1106000000 || (frequency >= 1270000000 && frequency <= 1323000000))
    {
        R2 = 0x721000A;
    }
    else if (frequency >= 1209000000 && frequency <= 1266000000)
    {
        R2 = 0x0421000A;
    }
    else if (frequency >= 1442000000 && frequency <= 1514000000)
    {
        R2 = 0x0421000A;
    }
    else if (frequency >= 1695000000 && frequency <= 1811000000)
    {
        R2 = 0x0321000A;
    }
    else if (frequency >= 2142000000 && frequency <= 2401000000)
    {
        R2 = 0x0121000A;
    }
    else
    {
        return;
    }
}
void updateR2Debugging(volatile uint16_t *current_in_micro_amp)
{
    uint16_t CPcurrent = *current_in_micro_amp;
    switch (CPcurrent)
    {
    case 1570:
        R2 = 0x0421000A;
        break;
    case 630:
        R2 = 0x0121000A;
        break;
    case 2500:
        R2 = 0x0721000A;
        break;
    case 3750:
        R2 = 0x0B21000A;
        break;
    case 4800:
        R2 = 0x0F21000A;
        break;
    default:
        R2 = 0x0721000A;
        break;
    }
}
void sendPLLFreqRegisters(void)
{
    uint32_t ToSendRegisters[3] = {R3, R1, R0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 3; j >= 0; j--)
        { // increment from MS byte to LS byte
            shiftOutFast((ToSendRegisters[i] >> (j * 8)));
        }
        latchFast();
    }
}

void sendPLLAllRegisters(void)
{
    uint32_t ToSendRegisters[11] = {R7, R62, R61, R52, R51, R42, R41, R3, R2, R1, R0};
    for (int i = 0; i < 11; i++)
    {
        for (int j = 3; j >= 0; j--)
        { // increment from MS byte to LS byte
            shiftOutFast((ToSendRegisters[i] >> (j * 8)));
        }
        latchFast();
    }
}

void latchFast(void)
{

    // Latch high
    gpio_set_mask(1 << LATCH_PIN);
    for (int i = 0; i < 2; i++)
    {
        asm volatile("nop");
    }
    // Latch low
    gpio_clr_mask(1 << LATCH_PIN);
    for (int i = 0; i < 2; i++)
    {
        asm volatile("nop");
    }
}

void updateR3(volatile bool *power_down)
{
    if (frequencyToPLL_inHz <= 1370000000)
    {
        negBld = 0b101;
    }
    else
    {
        negBld = 0b100;
    }
    if (*power_down == true)
    {
        R3 = 0x300A3 | (negBld << 22);
    }
    //           negBld=100
    // 0b:  0000 0001 0000 0011 0000 0000 1010 0011
    // 0x:   0    1    0    3    0    0    A    3
    else
    {
        R3 = 0x30083 | (negBld << 22);
    }
    //           negBld=100
    // 0b:  0000 0001 0000 0011 0000 0000 1000 0011
    // 0x:   0    1    0    3    0    0    8    3
}
void changeBorn(int bandInput)
{
    if (bornSets[bandInput][0])
    {
        sio_hw->gpio_set = (1 << BORN_PIN1); // Set high
    }
    else
    {
        sio_hw->gpio_clr = (1 << BORN_PIN1); // Set low
    }

    if (bornSets[bandInput][1])
    {
        sio_hw->gpio_set = (1 << BORN_PIN2); // Set high
    }
    else
    {
        sio_hw->gpio_clr = (1 << BORN_PIN2); // Set low
    }

    if (bornSets[bandInput][2])
    {
        sio_hw->gpio_set = (1 << BORN_PIN3); // Set high
    }
    else
    {
        sio_hw->gpio_clr = (1 << BORN_PIN3); // Set low
    }
}

void storeRegisterValue(uint8_t *buffer, uint32_t *registerValues, uint16_t NumOfRegisters)
{
    for (int i = 0; i < NumOfRegisters; i++)
    {
        *(buffer + 4 * i) = (*(registerValues + i)) & 0x000000FF;
        *(buffer + 4 * i + 1) = (*(registerValues + i) >> 8) & 0x000000FF;
        *(buffer + 4 * i + 2) = (*(registerValues + i) >> 16) & 0x000000FF;
        *(buffer + 4 * i + 3) = (*(registerValues + i) >> 24) & 0x000000FF;
    }
    notify_register_characteristic();
}

void restoreAllValues()
{
    storeFrequency(characteristic_FREQUENCY_tx, defaultFrequency_inHz);
    frequencyToPLL_inHz = defaultFrequency_inHz;
    calculateIntFrac();
    changeBorn(0);
    updateR0();
    updateR1();
    updateR3(&power_down_pll_flag);
    uint32_t AllRegisterValues[13] = {intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7};
    storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 13);
    sendPLLAllRegisters();
}

void frequencyHopOnce()
{
    if (frequencyToPLL_inHz < stopFrequency_inHz)
    {
        frequencyToPLL_inHz += stepFrequency_inHz;
        for (int i = 0; i < BANDS; i++)
        {
            if (frequencyToPLL_inHz >= frequencyBands[i][0] && frequencyToPLL_inHz <= frequencyBands[i][1])
            {
                calculateIntFrac();
                changeBorn(i);
                updateR0();
                updateR1();
                updateR3(&power_down_pll_flag);
                // if (i == 0 || i == 1 || i == 2 || i == 4)
                // {
                //     sendPLLFreqRegisters();
                // }
                // else
                // {
                //     updateR2(&frequencyToPLL_inHz);
                //     sendPLLAllRegisters();
                // }
                sendPLLFreqRegisters();
                break;
            }
        }
    }
}

void i2c_setup(void)
{
    // Initialize I2C
    i2c_init(I2C_PORT, I2C_BAUDRATE);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    // Enable internal pull-ups. Still need external pull up.
    // Generic setup for I2C
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

bool i2c_write_register(volatile uint8_t *handler, bool nostop_or_not)
{
    // Send one byte to a specified IC register.
    // Returns true if both bytes were acknowledged.
    /*
    A 3-byte packet that the UI will send to pico looks like:

    [slave addr]   [register addr]     [data]
     1010 0000        0000 0001        1000 0000

    */
    uint8_t buffer[2];
    uint8_t device_address = handler[0];
    buffer[0] = handler[1]; // Register address
    buffer[1] = handler[2]; // Data to store

    int bytes_written = i2c_write_blocking(
        I2C_PORT,
        device_address,
        buffer,
        sizeof(buffer),
        nostop_or_not);
    printf("device address written: %x\n", device_address);
    printf("reg address written: %x\n", buffer[0]);
    printf("code written: %u\n", buffer[1]);
    return bytes_written == sizeof(buffer);
}
// bool i2c_write_pre_power_mode_registers(uint8_t *handler, size_t handler_size)
// {
//     if (handler_size != digipot_power_saving_count)
//     {
//         return false;
//     }
//     uint8_t buffer[2];
//     for (int i = 0; i < digipot_power_saving_count; i++)
//     {
//         buffer[0] = power_saving_digipot_values[i].register_addr;
//         buffer[1] = handler[i];
//         int bytes_written = i2c_write_blocking(
//             I2C_PORT,
//             power_saving_digipot_values[i].slave_addr,
//             buffer,
//             sizeof(buffer),
//             false
//         );
//         if (bytes_written != sizeof(buffer))
//         {
//             printf("failed rollback power-saving slave address: %x\n", power_saving_digipot_values[i].slave_addr,);
//             printf("failed rollback power-saving reg address: %x\n", buffer[0]);
//             printf("failed rollback power-saving code: %u\n", buffer[1]);
//             return false;
//         }
//     }
//     return true;
// }
bool digipots_power_saving_mode(void)
{
    uint8_t buffer[2];
    uint8_t slave_addr;
    for (int i = 0; i < digipot_power_saving_count; i++)
    {
        slave_addr = power_saving_digipot_values[i].slave_addr;
        buffer[0] = power_saving_digipot_values[i].register_addr;
        buffer[1] = power_saving_digipot_values[i].code;
        int bytes_written = i2c_write_blocking(
            I2C_PORT,
            slave_addr,
            buffer,
            sizeof(buffer),
            false);
        printf("Slave_addr written - power saving: %x\n", slave_addr);
        printf("Reg_addr written - power saving: %x\n", buffer[0]);
        printf("Code written - power saving: %x\n\n", buffer[1]);

        if (bytes_written != sizeof(buffer))
        {
            printf("failed power-saving slave address: %x\n", slave_addr);
            printf("failed power-saving reg address: %x\n", buffer[0]);
            printf("failed power-saving code: %u\n", buffer[1]);
            return false;
        }
    }
    return true;
}

bool digipots_init(bool on_set_up)
{
    // If this function is called in setup, we need to send ACR and write non-volatile.
    if (on_set_up)
    {
        for (int i = 0; i < digipot_default_count; i++)
        {
            uint8_t slave_addr = default_digipot_values[i].slave_addr;
            uint8_t buffer[2];
            buffer[0] = default_digipot_values[i].register_addr;
            buffer[1] = default_digipot_values[i].code;
            i2c_write_ACR_register_non_vol_variable(&slave_addr, false);

            int bytes_written = i2c_write_blocking(
                I2C_PORT,
                slave_addr,
                buffer,
                sizeof(buffer),
                false);

            if (bytes_written != sizeof(buffer))
            {
                return false;
            }
        }
        return true;
    }

    // If this is not the first time we set up the digipot, just send default code of all pots
    else
    {
        for (int i = 0; i < digipot_default_count; i++)
        {
            uint8_t buffer[2];
            buffer[0] = default_digipot_values[i].register_addr;
            buffer[1] = default_digipot_values[i].code;
            int bytes_written = i2c_write_blocking(
                I2C_PORT,
                default_digipot_values[i].slave_addr,
                buffer,
                sizeof(buffer),
                false);

            printf("bytes_written - default: %d\n", bytes_written);
            if (bytes_written != sizeof(buffer))
            {
                printf("failed df slave address: %x\n", default_digipot_values[i].slave_addr);
                printf("failed df reg address: %x\n", buffer[0]);
                printf("failed df code: %u\n", buffer[1]);
                return false;
            }
        }
        return true;
    }
}

bool i2c_write_ACR_register_non_vol_variable(uint8_t *device_address, bool is_volatile)
{
    if (device_address == NULL)
    {
        return false;
    }
    uint8_t buffer[2];
    buffer[0] = 0x10;
    if (is_volatile)
    {
        buffer[1] = 0xC0;
    }
    else
    {
        buffer[1] = 0x60;
    }
    int bytes_written = i2c_write_blocking(
        I2C_PORT,
        *device_address,
        buffer,
        sizeof(buffer),
        false);
    return bytes_written == sizeof(buffer);
}

bool i2c_write_ACR_register(volatile uint8_t *device_address, bool is_volatile)
{
    if (device_address == NULL)
    {
        return false;
    }
    uint8_t buffer[2];
    buffer[0] = 0x10;
    if (is_volatile)
    {
        buffer[1] = 0xC0;
    }
    else
    {
        buffer[1] = 0x40;
    }
    int bytes_written = i2c_write_blocking(
        I2C_PORT,
        *device_address,
        buffer,
        sizeof(buffer),
        false);
    return bytes_written == sizeof(buffer);
}
void ldo_shift_register_init(void)
{
    gpio_init(SR_DATA_PIN);
    gpio_set_dir(SR_DATA_PIN, GPIO_OUT);

    gpio_init(SR_CLOCK_PIN);
    gpio_set_dir(SR_CLOCK_PIN, GPIO_OUT);

    gpio_init(SR_LATCH_PIN);
    gpio_set_dir(SR_LATCH_PIN, GPIO_OUT);

    gpio_init(SR_OE_PIN);
    gpio_set_dir(SR_OE_PIN, GPIO_OUT);

    gpio_put(SR_DATA_PIN, 0);
    gpio_put(SR_CLOCK_PIN, 0);
    gpio_put(SR_LATCH_PIN, 0);

    // Disable Q outputs while loading startup state.
    gpio_put(SR_OE_PIN, 1);

    // Both UI switches are checked by default.
    shift_register_state = 0b00000011;

    shift_register_write(shift_register_state);

    // Enable Q0-Q7.
    // gpio_put(SR_OE_PIN, 0);

    printf("LDO shift register initialized: 0x%02X\n", shift_register_state);
}
// void shift_register_write(uint8_t value)
// {
//     // Shift MSB first.
//     // Sending bit 7 first means bit 0 eventually ends up at Q0.

//     for (int bit = 7; bit >= 0; bit--)
//     {
//         gpio_put(
//             SR_DATA_PIN,
//             (value >> bit) & 0x01);

//         // Rising edge = shift one bit
//         gpio_put(SR_CLOCK_PIN, 1);
//         sleep_us(1);

//         gpio_put(SR_CLOCK_PIN, 0);
//         sleep_us(1);
//     }

//     // Copy shift register to Q0-Q7
//     gpio_put(SR_LATCH_PIN, 1);
//     sleep_us(1);

//     gpio_put(SR_LATCH_PIN, 0);
// }

// void ldo_set_enable(uint8_t output, bool enable)
// {
//     if (output > 7)
//     {
//         return;
//     }

//     uint8_t mask = (1u << output);

//     if (enable)
//     {
//         shift_register_state |= mask;
//     }
//     else
//     {
//         shift_register_state &= ~mask;
//     }

//     shift_register_write(shift_register_state);

//     printf("74HC595 state: 0x%02X\n", shift_register_state);
// }

// void handle_enable_command(uint8_t command)
// {
//     switch (command)
//     {
//     case PIN3_DISABLE:
//         printf("Disabling Pin 3 / Q0\n");
//         ldo_set_enable(0, false);
//         break;

//     case PIN3_ENABLE:
//         printf("Enabling Pin 3 / Q0\n");
//         ldo_set_enable(0, true);
//         break;

//     case PIN12_DISABLE:
//         printf("Disabling Pin 12 / Q1\n");
//         ldo_set_enable(1, false);
//         break;

//     case PIN12_ENABLE:
//         printf("Enabling Pin 12 / Q1\n");
//         ldo_set_enable(1, true);
//         break;

//     default:
//         printf("Unknown enable command: %u\n", command);
//         break;
//     }
// }

void shift_register_write(uint8_t value) {
    gpio_clr_mask(1 << SR_LATCH_PIN);
    for (int i = 7; i >= 0; i--) {
        gpio_clr_mask(1 << SR_CLOCK_PIN);
        if (value & (1 << i)) {
            gpio_set_mask(1 << SR_DATA_PIN);
            for (int i = 0; i < 2; i++) {
                asm("nop");
            }
            gpio_set_mask(1 << SR_CLOCK_PIN);
        }
        else {
            gpio_clr_mask(1 << SR_DATA_PIN);
            for (int i = 0; i < 2; i++) {
                asm("nop");
            }
            gpio_set_mask(1 << SR_CLOCK_PIN);
        }
    }
    gpio_set_mask(1 << SR_LATCH_PIN);
        for (int i = 0; i < 2; i++) {
            asm("nop");
        }
    gpio_clr_mask(1 << SR_LATCH_PIN);
}