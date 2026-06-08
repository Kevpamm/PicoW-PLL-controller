#ifndef SERVICE_IMPLEMENTATION_H
#define SERVICE_IMPLEMENTATION_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "btstack.h"
#include "ble/att_server.h"
#include "BLE_service.h"
#include "pico/bootrom.h"

static inline void storeFrequency(uint8_t *field, uint32_t frequency);
static inline uint32_t parseFrequency(uint8_t * buffer, uint16_t buffer_size);
static inline void notify_register_characteristic(void);

extern volatile bool frequency_receipt_status;
extern volatile uint32_t pending_frequency;

extern volatile bool send_all_registers_flag;
extern volatile bool send_freq_registers_flag;
extern volatile bool restore_default_registers_flag;
extern volatile bool register_notification_first_on;
extern volatile bool hop_frequency_flag;
extern volatile bool led_flag;

extern const uint32_t defaultFrequency;

enum ControlValue {
    EMPTY,
    SEND_ALL_REGISTERS__CONTROL,
    SEND_FREQ_REGISTERS_ONLY__CONTROL_ENUM,
    RESTORE_DEFAULT__CONTROL_ENUM,
    RESET_PICO__CONTROL_ENUM,
    BLE_DATA_ON__CONTROL_ENUM,
    FAST_PIN__CONTROL_ENUM
};
// This struct manages our service
typedef struct {

    hci_con_handle_t con_handle;

    //Frequency Characteristic Information
    uint8_t * characteristic_frequency_value;
    uint16_t characteristic_frequency_value_length;
    uint16_t characteristic_frequency_client_configuration;
    char * characteristic_frequency_user_description;

    //Control Characteristic Information
    uint8_t * characteristic_control_value;
    uint16_t characteristic_control_length;
    char * characteristic_control_user_description;

    // Hop Characteristic Information
    uint8_t * characteristic_hop_value;
    uint16_t characteristic_hop_value_length;
    uint16_t characteristic_hop_client_configuration;
    char * characteristic_hop_user_description;

    // Register Characteristic Information
    uint8_t * characteristic_register_value;
    uint16_t characteristic_register_value_length;
    uint16_t characteristic_register_client_configuration;
    char * characteristic_register_user_description;

    // LED Characteristic Information
    uint8_t * characteristic_LED_value;
    uint16_t characteristic_LED_value_length;
    char * characteristic_LED_user_description;

    // Frequency Characteristic Handle
    uint16_t characteristic_frequency_handle;
    uint16_t characteristic_frequency_client_configuration_handle;
    uint16_t characteristic_frequency_user_description_handle;

    // Control Characteristic Handle
    uint16_t characteristic_control_handle;
    uint16_t characteristic_control_user_description_handle;

    // Hop Characteristic Handle
    uint16_t characteristic_hop_handle;
    uint16_t characteristic_hop_client_configuration_handle;
    uint16_t characteristic_hop_user_description_handle;

    // Register Characteristic Handle
    uint16_t characteristic_register_handle;
    uint16_t characteristic_register_client_configuration_handle;
    uint16_t characteristic_register_user_description_handle;

    // LED Characteristic Handle
    uint16_t characteristic_LED_handle;
    uint16_t characteristic_LED_user_description_handle;

    btstack_context_callback_registration_t callback_Frequency;
    btstack_context_callback_registration_t callback_Hop;
    btstack_context_callback_registration_t callback_Register;
} PLL_service_t;

static att_service_handler_t service_handler;
static PLL_service_t service_object;

char characteristic_frequency[] = "Frequency";
char characteristic_control[] = "Control";
char characteristic_hop[] = "Hop Frequency";
char characteristic_register[] = "Register Value";
char characteristic_led[] = "LED status";

// semaphore_t BLUETOOTH_READY; <- this is replaced with flags

static void characteristic_frequency_callback(void * context){
    PLL_service_t * instance = (PLL_service_t *) context ;

    printf("Frequency notify callback fired\n");
    printf("Frequency notify handle: 0x%04x\n", instance->characteristic_frequency_handle);
    printf("Frequency notify length: %u\n", instance->characteristic_frequency_value_length);

    att_server_notify(instance->con_handle, 
        instance->characteristic_frequency_handle, 
        instance->characteristic_frequency_value, 
        instance->characteristic_frequency_value_length);
}
//uint8_t att_server_notify(hci_con_handle_t con_handle, uint16_t attribute_handle, const uint8_t *value, uint16_t value_len){
static void characteristic_hop_callback(void * context){
    PLL_service_t * instance = (PLL_service_t *) context ;
    att_server_notify(instance->con_handle, 
        instance->characteristic_hop_handle, 
        instance->characteristic_hop_value, 
        instance->characteristic_hop_value_length);
}

static void characteristic_register_callback(void * context){
    PLL_service_t * instance = (PLL_service_t *) context ;
    att_server_notify(instance->con_handle, 
        instance->characteristic_register_handle, 
        instance->characteristic_register_value, 
        instance->characteristic_register_value_length);
}

static uint16_t PLL_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
    UNUSED(con_handle);

    // Frequency Characteristic
    if (attribute_handle == service_object.characteristic_frequency_handle){
        return att_read_callback_handle_blob(service_object.characteristic_frequency_value, service_object.characteristic_frequency_value_length, offset, buffer, buffer_size);
    }
    else if (attribute_handle == service_object.characteristic_frequency_client_configuration_handle) {
        return att_read_callback_handle_little_endian_16(service_object.characteristic_frequency_client_configuration, offset, buffer, buffer_size);
    }
    else if (attribute_handle == service_object.characteristic_frequency_user_description_handle){
        return att_read_callback_handle_blob(service_object.characteristic_frequency_user_description, strlen(service_object.characteristic_frequency_user_description), offset, buffer, buffer_size);
    }
    //Hop Characteristic
    else if (attribute_handle == service_object.characteristic_hop_handle){
        return att_read_callback_handle_blob(service_object.characteristic_hop_value, service_object.characteristic_hop_value_length, offset, buffer, buffer_size);
    }
    else if (attribute_handle == service_object.characteristic_hop_client_configuration_handle) {
        return att_read_callback_handle_little_endian_16(service_object.characteristic_hop_client_configuration, offset, buffer, buffer_size);
    }
    else if (attribute_handle == service_object.characteristic_hop_user_description_handle){
        return att_read_callback_handle_blob(service_object.characteristic_hop_user_description, strlen(service_object.characteristic_hop_user_description), offset, buffer, buffer_size);
    }
    else
        return 0;
}

//Write Callback
static int PLL_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
	UNUSED(offset);

    // Frequency Characteristic - Enable/disable notifications
    if (attribute_handle == service_object.characteristic_frequency_client_configuration_handle){
        service_object.characteristic_frequency_client_configuration = little_endian_read_16(buffer, 0);
        service_object.con_handle = con_handle;
        printf("Frequency CCCD written: 0x%04x\n", service_object.characteristic_frequency_client_configuration);
        return 0;
    }

    
    // Frequency Characteristic - Write Value
    else if (attribute_handle == service_object.characteristic_frequency_handle){
        PLL_service_t * instance = &service_object;
        if (buffer_size == 4) {
            memcpy(instance->characteristic_frequency_value, buffer, 4);
            instance->characteristic_frequency_value_length = buffer_size;
            pending_frequency = parseFrequency(buffer, buffer_size);
            frequency_receipt_status = true;
        }
        else {
            printf("Pico-W received incorrect write format for Frequency characteristic!");
            return 1;
        }

        //Notify back the written frequency value
        if (instance->characteristic_frequency_client_configuration) {
            instance->callback_Frequency.callback = &characteristic_frequency_callback;
            instance->callback_Frequency.context = (void*) instance;
            att_server_register_can_send_now_callback(&instance->callback_Frequency, instance->con_handle);
        }
        return 0;
    }
    // Control Characteristisc - Write Value
    else if (attribute_handle == service_object.characteristic_control_handle){
        PLL_service_t *instance = &service_object;
        if (buffer_size == 0)
            return 0;
        else if (buffer_size != 1) {
            printf("Buffer size of the Control Characteristic Value is not 1!\n");
            printf("Control Value received from JS is: ");
            for (int i = 0; i < buffer_size - 1; i++) {
                printf("%u", *(buffer + i));
                printf(" ");
            }
            printf("%u", *(buffer + (buffer_size - 1)));
            return 1;
        }
        else {
            memcpy(instance->characteristic_control_value, buffer, 1);
            uint8_t ControlCommandReceived = *buffer;
            if (ControlCommandReceived == SEND_ALL_REGISTERS__CONTROL) {
                send_all_registers_flag = true;
                //sendAllRegister;
            }
            else if (ControlCommandReceived == SEND_FREQ_REGISTERS_ONLY__CONTROL_ENUM) {
                send_freq_registers_flag = true;
            }

            else if(ControlCommandReceived == RESTORE_DEFAULT__CONTROL_ENUM) {
                restore_default_registers_flag = true;
            }
            else if(ControlCommandReceived == RESET_PICO__CONTROL_ENUM) {
                reset_usb_boot(0,0);
            }
            else if(BLE_DATA_ON__CONTROL_ENUM) {
                printf("Pico-W feature \"BLE_data_on\" has not been added!\n");
            }
            else if(FAST_PIN__CONTROL_ENUM) {
                printf("Pico-W only sends data in Fast-pin mode. This feature cannot be turned off.\n");
            }
            return 0;

        }
    }
    else if (attribute_handle == service_object.characteristic_register_client_configuration_handle) {
        service_object.characteristic_register_client_configuration = little_endian_read_16(buffer, 0);
        service_object.con_handle = con_handle;
        printf("Register CCCD: 0x%04x\n", service_object.characteristic_register_client_configuration);
        register_notification_first_on = true;
        return 0;
    }

    // Hop Characteristic - Write
    else if (attribute_handle == service_object.characteristic_hop_handle) {
        PLL_service_t *instance = &service_object;
        if (buffer_size == 5) {
            memcpy(instance->characteristic_hop_value, buffer, 5);
            instance->characteristic_hop_value_length = buffer_size;
            hop_frequency_flag = true;

            //Notify the new Hop Frequency back to the client.
            if (instance->characteristic_frequency_client_configuration) {
                instance->callback_Hop.callback = &characteristic_hop_callback;
                instance->callback_Hop.context = (void*) instance;
                att_server_register_can_send_now_callback(&instance->callback_Hop, instance->con_handle);
            }
            return 0;
        }
    }
    // Hop Characteristic - Enable/Disable notifications
    else if(attribute_handle == service_object.characteristic_hop_client_configuration_handle) {
        service_object.characteristic_hop_client_configuration = little_endian_read_16(buffer, 0);
        service_object.con_handle = con_handle;
        return 0;
    }

    // LED Characteristic - Write Value
    else if (attribute_handle == service_object.characteristic_LED_handle) { //LED
        PLL_service_t *instance = &service_object;
        if (buffer_size == 0)
            return 0;
        else if (buffer_size != 1) {
            printf("Pico-W receives incorrect write value for LED characteristic. LED char only accepts 0/1");
            return 1;
        }
        else {
            memcpy(instance->characteristic_LED_value, buffer, 1);
            instance->characteristic_LED_value_length = buffer_size;
            if (*buffer == 0b00000001)
                led_flag = true;
            else
                led_flag = false;
        }
        return 0;
    }
}

// Initialize our PLL service handler:

void PLL_service_server_init(uint8_t * frequency_ptr, uint8_t * control_ptr, uint8_t * hop_ptr, uint8_t * register_ptr, uint8_t * led_ptr) {
    // Pointer to our service_object
    PLL_service_t * instance = &service_object;

    instance->characteristic_frequency_value = frequency_ptr;
    instance->characteristic_frequency_value_length = 4;

    instance->characteristic_control_value = control_ptr;
    instance->characteristic_control_length = 1;

    instance->characteristic_hop_value = hop_ptr;
    instance->characteristic_hop_value_length = 5;

    instance->characteristic_register_value = register_ptr;
    instance->characteristic_register_value_length = 52;

    instance->characteristic_LED_value = led_ptr;
    instance->characteristic_LED_value_length = 1;

    instance->characteristic_frequency_user_description = characteristic_frequency;
    instance->characteristic_control_user_description = characteristic_control;
    instance->characteristic_hop_user_description = characteristic_hop;
    instance->characteristic_register_user_description = characteristic_register;
    instance->characteristic_LED_user_description = characteristic_led;

    // Assigned handle values
    instance->characteristic_frequency_handle = ATT_CHARACTERISTIC_50e12001_a21d_4471_b2f0_412147c8399e_01_VALUE_HANDLE;
    instance->characteristic_frequency_client_configuration_handle = ATT_CHARACTERISTIC_50e12001_a21d_4471_b2f0_412147c8399e_01_CLIENT_CONFIGURATION_HANDLE;
    instance->characteristic_frequency_user_description_handle = ATT_CHARACTERISTIC_50e12001_a21d_4471_b2f0_412147c8399e_01_USER_DESCRIPTION_HANDLE;

    instance->characteristic_control_handle = ATT_CHARACTERISTIC_50e12002_a21d_4471_b2f0_412147c8399e_01_VALUE_HANDLE;

    instance->characteristic_hop_handle = ATT_CHARACTERISTIC_50e12003_a21d_4471_b2f0_412147c8399e_01_VALUE_HANDLE;
    instance->characteristic_hop_client_configuration_handle = ATT_CHARACTERISTIC_50e12003_a21d_4471_b2f0_412147c8399e_01_CLIENT_CONFIGURATION_HANDLE;
    instance->characteristic_hop_user_description_handle = ATT_CHARACTERISTIC_50e12003_a21d_4471_b2f0_412147c8399e_01_USER_DESCRIPTION_HANDLE;

    instance->characteristic_register_handle = ATT_CHARACTERISTIC_50e12004_a21d_4471_b2f0_412147c8399e_01_VALUE_HANDLE;
    instance->characteristic_register_client_configuration_handle = ATT_CHARACTERISTIC_50e12004_a21d_4471_b2f0_412147c8399e_01_CLIENT_CONFIGURATION_HANDLE;
    instance->characteristic_register_user_description_handle = ATT_CHARACTERISTIC_50e12004_a21d_4471_b2f0_412147c8399e_01_USER_DESCRIPTION_HANDLE;
    instance->characteristic_LED_handle = ATT_CHARACTERISTIC_50e12010_a21d_4471_b2f0_412147c8399e_01_VALUE_HANDLE;
    instance->characteristic_LED_user_description_handle = ATT_CHARACTERISTIC_50e12010_a21d_4471_b2f0_412147c8399e_01_USER_DESCRIPTION_HANDLE;

//     #define ATT_SERVICE_50e12000_a21d_4471_b2f0_412147c8399e_START_HANDLE 0x0007
// #define ATT_SERVICE_50e12000_a21d_4471_b2f0_412147c8399e_END_HANDLE 0x0019
    service_handler.start_handle = ATT_SERVICE_50e12000_a21d_4471_b2f0_412147c8399e_START_HANDLE;
    service_handler.end_handle = ATT_SERVICE_50e12000_a21d_4471_b2f0_412147c8399e_END_HANDLE;

    service_handler.read_callback = &PLL_service_read_callback;
    service_handler.write_callback = &PLL_service_write_callback;

    att_server_register_service_handler(&service_handler);
}


static inline uint32_t parseFrequency(uint8_t * buffer, uint16_t buffer_size) {
    if (buffer_size != 4) {
        printf("The frequency is expected to be 32bits! It's currently not!");
        return 0;
    }
    uint32_t eight_last_bits = *buffer;
    uint32_t eight_second_last_bits = *(buffer + 1);
    uint32_t eight_third_last_bits = *(buffer + 2);
    uint32_t eight_most_sig_bits = *(buffer + 3);
    return (eight_most_sig_bits << 24 | eight_third_last_bits << 16 | eight_second_last_bits << 8 | eight_last_bits);
}

static inline void storeFrequency(uint8_t *field, uint32_t frequency) {
    *field = frequency & 0x000000FF;
    *(field + 1) = (frequency >> 8) & 0x000000FF;
    *(field + 2) = (frequency >> 16) & 0x000000FF;
    *(field + 3) = (frequency >> 24) & 0x000000FF;
}

static inline void notify_register_characteristic(void) {
    PLL_service_t *instance = &service_object;
    if (instance->characteristic_register_client_configuration) {
        instance->callback_Register.callback = &characteristic_register_callback;
        instance->callback_Register.context = (void*) instance;
        att_server_register_can_send_now_callback(&instance->callback_Register, instance->con_handle);
    }
}

#endif