#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "GATT_service/service_implementation.h"
#include <string.h>
#include <assert.h>
#include <math.h>

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

// This defines how INT and FRAC of the N divider are calculated
#define CONRAD_PLL_MATH // Conrad ADF1549 configuration
// #define KEVIN_PLL_MATH // Kevin ADF1549 configuration. Arguably more optimized

#if defined CONRAD_PLL_MATH
volatile uint32_t ToSendFrequency = 920; // in hz 900000000

uint32_t intVal = 36; // set to 900MHz initially
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
uint32_t ToSendFrequency = 920; // in hz 900000000
uint32_t intVal = 23;           // set to 900MHz initially
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
#define BORN_PIN1 10
#define BORN_PIN2 11
#define BORN_PIN3 12

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
bool POWER_STATUS = true;
bool is_hopping = false;
bool hop = false;
bool FIRST_CONNECTION = true;

volatile uint32_t pending_frequency = 0;

// bool freqHopFlag = false;
// bool wasHopping = false;         // cleanup flag
// uint32_t fhDelay = 1000000;      // in us (microseconds)
// uint32_t fhStep = 50 * 1000000;  // in hz
// uint32_t fhSpan = 300 * 1000000; // in hz
// uint32_t fhStart = 90000000;     // default updated each freq send

volatile bool hop_delay_time_receipt_status = false;
volatile bool hop_step_frequency_receipt_status = false;
volatile bool span_frequency_receipt_status = false;
volatile uint32_t delayTime_inMillisec = 0;
volatile uint32_t stepFrequency = 0;
volatile uint32_t spanFrequency = 0;
volatile uint32_t stopFrequency = 0;

uint64_t lastHop = 0;

// NEW SUPPORTED FREQUENCY BANDS
const uint32_t frequencyBands[BANDS][2] = {{920, 949}, {970, 996}, {1074, 1106}, {1209, 1266}, {1270, 1323}, {1442, 1514}, {1695, 1811}, {2142, 2401}};

// OLD SUPPORTED FREQUENCY BANDS
// const uint32_t frequencyBands[BANDS][2] = {{890, 922},{951, 997},{1039, 1103},{1140,1204},{1196,1285},{1305,1406},{1524,1709},{1815,2105}};
const bool bornSets[BANDS][3] = {{1, 1, 1}, {1, 1, 0}, {0, 1, 1}, {0, 1, 0}, {1, 0, 1}, {1, 0, 0}, {0, 0, 1}, {0, 0, 0}};

// FUNCTION DEFINITION
void shiftOutFast(uint8_t val);
static int pico_led_init(void);
static void pico_set_led(bool led_on);
void calculateIntFrac(void);
void updateR0(void);
void sendPLLFreqRegisters(void);
void latchFast(void);
void updateR3(volatile bool *power_down);
void updateR1(void);
void changeBorn(int bandInput);
void storeRegisterValue(uint8_t *buffer, uint32_t *registerValues, uint16_t NumOfRegisters);
void sendPLLAllRegisters(void);
void restoreAllValues();
void frequencyHopOnce();

const uint32_t defaultFrequency = 920000000;
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

static uint8_t characteristic_FREQUENCY_tx[BUFFER_SIZE];
static uint8_t characteristic_CONTROL_tx[BUFFER_SIZE];
static uint8_t characteristic_HOP_tx[BUFFER_SIZE];
static uint8_t characteristic_REGISTER_tx[BUFFER_SIZE];
static uint8_t characteristic_LED_tx[BUFFER_SIZE];

// uint32_t catalyst = 0x000000FF; why did I name it catalyst? @.@
// for (int i = 0; i < 4; i++) {
//     characteristic_FREQUENCY_tx[i] = defaultFrequency & (catalyst << (8*i));
// }
// *HARD FOR OTHERS TO UNDERSTAND

bool freqHop_timer_callback(struct repeating_timer *t)
{
    if (!hop_command_flag || ToSendFrequency >= stopFrequency)
    {
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
    gpio_put(DATA_PIN, 0);

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
    PLL_service_server_init(characteristic_FREQUENCY_tx, characteristic_CONTROL_tx,
                            characteristic_HOP_tx, characteristic_REGISTER_tx,
                            characteristic_LED_tx);

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Register for ATT event
    att_server_register_packet_handler(packet_handler);

    // TURN THE BLUETOOTH ON!
    hci_power_control(HCI_POWER_ON);

    // Store default Frequency = 920 MHz into the Frequency Buffer
    storeFrequency(characteristic_FREQUENCY_tx, defaultFrequency);

    // Store default register values into Register buffer
    uint32_t AllRegisterValues[13] = {intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7};
    storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 13);

    struct repeating_timer timer;
    

    while (1)
    {

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
            ToSendFrequency = pending_frequency / 1000000;
            printf("\nFrequency Buffer after receiving frequency 1 %u: ", ToSendFrequency);
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                printf("%X ", characteristic_FREQUENCY_tx[i]);
            }
            printf("\n");

            for (int i = 0; i < BANDS; i++)
            { // check input against valid frequencies
                if (ToSendFrequency >= (frequencyBands[i][0]) && ToSendFrequency <= (frequencyBands[i][1]))
                {
                    calculateIntFrac();
                    updateR0();
                    updateR1();
                    printf("R1: %X\n", R1);
                    updateR3(&power_down_pll_flag);
                    changeBorn(i);
                    sendPLLFreqRegisters();
                    uint32_t AllRegisterValues[13] = {intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7}; // No need to update all 13. only three values in the buffer are changed: R0, R1, R3
                    storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 13);
                    printf("\nCharacteristic Buffer after receiving frequency %u: ", ToSendFrequency);
                    for (int i = 0; i < BUFFER_SIZE; i++)
                    {
                        printf("%X ", characteristic_REGISTER_tx[i]);
                    }
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
        // if (hop_command_flag && !frequency_receipt_status)
        // {
        //     uint64_t currentMicros = to_us_since_boot(get_absolute_time());
        //     if (currentMicros - lastHop >= delayTime_inMicro && ToSendFrequency < endHopFrequency)
        //     {
        //         ToSendFrequency += stepFrequency;
        //         lastHop = currentMicros;
        //         for (int i = 0; i < BANDS; i++)
        //         {
        //             if (ToSendFrequency >= (frequencyBands[i][0]) && ToSendFrequency <= (frequencyBands[i][1]))
        //             {
        //                 calculateIntFrac();
        //                 updateR0();
        //                 updateR1();
        //                 updateR3(&power_down_pll_flag);
        //                 changeBorn(i);
        //                 sendPLLFreqRegisters();
        //                 uint32_t AllRegisterValues[6] = {intVal, fracVal, R0, R1, R2, R3};
        //                 storeRegisterValue(characteristic_REGISTER_tx, AllRegisterValues, 6);
        //             }
        //         }
        //     }
        //     if (ToSendFrequency >= endHopFrequency)
        //     {
        //         hop_command_flag = false;
        //     }
        // }
        if (hop_step_frequency_receipt_status == true)
        {
            printf("Step Frequency received: %u\n", stepFrequency);
            hop_step_frequency_receipt_status = false;
        }
        if (hop_delay_time_receipt_status) {
            printf("Delay time receive: %u\n", delayTime_inMillisec);
            hop_delay_time_receipt_status = false;
        }
        if (span_frequency_receipt_status) {
            printf("Span receive: %u\n", spanFrequency);
            span_frequency_receipt_status = false;
        }

        if (hop_command_flag && !is_hopping)
        {
            printf("Add timer\n");
            add_repeating_timer_ms(delayTime_inMillisec, freqHop_timer_callback, NULL, &timer);
            is_hopping = true;
        }
        if (hop == true) {
            hop = false;
            frequencyHopOnce();
        }

        if (send_all_registers_flag == true)
        {
            sendPLLAllRegisters();
            send_all_registers_flag = false;
        }
        if (send_freq_registers_flag == true)
        {
            sendPLLFreqRegisters();
            send_freq_registers_flag = false;
        }
        if (restore_default_registers_flag == true)
        {
            restoreAllValues();
            restore_default_registers_flag = false;
        }
        // This logic is false
        if (power_down_pll_flag == true && POWER_STATUS)
        {
            updateR3(&power_down_pll_flag);
            sendPLLFreqRegisters();
            POWER_STATUS = false;
        }
        else if (!power_down_pll_flag && !POWER_STATUS)
        {
            updateR3(&power_down_pll_flag);
            sendPLLFreqRegisters();
            POWER_STATUS = true;
            printf("lo ");
        }

        pico_set_led(led_flag);
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

void calculateIntFrac(void)
{
#if defined(CONRAD_PLL_MATH)
    intVal = ToSendFrequency / 25;
    fracVal = (uint32_t)(round((double)(ToSendFrequency - (intVal * 25)) * 1000000 * 1.34217727));
#elif defined(KEVIN_PLL_MATH)
    intVal = ToSendFrequency / 40;
    fracVal = (uint32_t)(round((double)(ToSendFrequency - (intVal * 40)) * 1000000 * 0.8388608));
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
    for (int i = 0; i < 3; i++)
    {
        pico_set_led(true);
        sleep_ms(300);
        pico_set_led(false);
        sleep_ms(300);
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
    for (int i = 0; i < 3; i++)
    {
        pico_set_led(true);
        sleep_ms(300);
        pico_set_led(false);
        sleep_ms(300);
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
    if (ToSendFrequency <= 1370)
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
    storeFrequency(characteristic_FREQUENCY_tx, defaultFrequency);
    ToSendFrequency = 920;
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
    if (ToSendFrequency < stopFrequency)
    {
        ToSendFrequency += stepFrequency;
        for (int i = 0; i < BANDS; i++)
        {
            if (ToSendFrequency >= frequencyBands[i][0] && ToSendFrequency <= frequencyBands[i][1])
            {
                calculateIntFrac();
                changeBorn(i);
                updateR0();
                updateR1();
                updateR3(&power_down_pll_flag);
                sendPLLFreqRegisters();
                break;
            }
        }
    }
}
