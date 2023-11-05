/* Fonte: https://github.com/Molorius/esp32-ads1115
 *
 * Adapted from Blake Felt's source code available at the link above. All
 * changes to source code are documented with a "-N" identifier at the end.
 */

#ifndef ADS1115_H
#define ADS1115_H

#ifdef __cplusplus
extern "C" {
#endif

// All libraries used throughout the project are included in this header file -N
#include "global_data.h"

/* Wrapped the code with this condition for easy deactivation of the device when
 * not using it for debug purposes -N */
#ifdef ADS1115_SENSOR
#define ADS1115_ADDR 0x48  // Sets the address of the device -N

/* Parameters of the analog input device (BPW34 photodiode connected to a
 * transimpedance amplifier) -N */
#define PD_AREA 7.02  // (mm2) -N
#define PD_SENS 0.62  // (A/W) -N
#define RF 50000      // (Ω) -N
                      /*  Ip    = Irrad * Area * Sens
                       *  Ip    = Vo / Rf
                       *  Irrad = Ip / (Area * Sens)
                       *  Irrad = Vo / (Rf * Area * Sens)
                       *
                       *  Ip    = Photodiode reverse current.
                       *  Irrad = Irradiance.
                       *  Area  = Radiant sensitive area.
                       *  Sens  = Spectral sensitivity of the chip.
                       *  Rf    = Feedback resistor.
                       *  Vo    = Output voltage.
                       *
                       *  Vo = Ip * Rf - ESP32 supports at max 3.3V as input.
                       *  At STC (1000 W/m2 irradiance) Ip will be:
                       *  Ip = 1000 * 7.02*1e-6 * 0.62 = 4.35 mA
                       *  So Rf has to be such as Vo = 4.35 mA * Rf (Vo < 3.3V)
                       *  Being 680 Ω the highest comercial value where this is true.
                       *
                       *  https://www.osram.com/ecat/com/en/class_pim_web_catalog_103489/prd_pim_device_2219534/
                       * -N */

typedef enum {  // register address
    ADS1115_CONVERSION_REGISTER_ADDR = 0,
    ADS1115_CONFIG_REGISTER_ADDR,
    ADS1115_LO_THRESH_REGISTER_ADDR,
    ADS1115_HI_THRESH_REGISTER_ADDR,
    ADS1115_MAX_REGISTER_ADDR
} ads1115_register_addresses_t;

typedef enum {  // multiplex options
    ADS1115_MUX_0_1 = 0,
    ADS1115_MUX_0_3,
    ADS1115_MUX_1_3,
    ADS1115_MUX_2_3,
    ADS1115_MUX_0_GND,
    ADS1115_MUX_1_GND,
    ADS1115_MUX_2_GND,
    ADS1115_MUX_3_GND,
} ads1115_mux_t;

typedef enum {  // full-scale resolution options
    ADS1115_FSR_6_144 = 0,
    ADS1115_FSR_4_096,
    ADS1115_FSR_2_048,
    ADS1115_FSR_1_024,
    ADS1115_FSR_0_512,
    ADS1115_FSR_0_256,
} ads1115_fsr_t;

typedef enum {  // samples per second
    ADS1115_SPS_8 = 0,
    ADS1115_SPS_16,
    ADS1115_SPS_32,
    ADS1115_SPS_64,
    ADS1115_SPS_128,
    ADS1115_SPS_250,
    ADS1115_SPS_475,
    ADS1115_SPS_860
} ads1115_sps_t;

typedef enum {
    ADS1115_MODE_CONTINUOUS = 0,
    ADS1115_MODE_SINGLE
} ads1115_mode_t;

typedef union {  // configuration register
    struct {
        uint16_t COMP_QUE : 2;   // bits 0..  1  Comparator queue and disable
        uint16_t COMP_LAT : 1;   // bit  2       Latching Comparator
        uint16_t COMP_POL : 1;   // bit  3       Comparator Polarity
        uint16_t COMP_MODE : 1;  // bit  4       Comparator Mode
        uint16_t DR : 3;         // bits 5..  7  Data rate
        uint16_t MODE : 1;       // bit  8       Device operating mode
        uint16_t PGA : 3;        // bits 9..  11 Programmable gain amplifier
                                 // configuration
        uint16_t MUX : 3;        // bits 12.. 14 Input multiplexer configuration
        uint16_t OS : 1;         // bit  15      Operational status or single-
                                 // shot conversion start
    } bit;
    uint16_t reg;
} ADS1115_CONFIG_REGISTER_Type;

typedef struct {
    bool in_use;                   // gpio is used
    gpio_num_t pin;                // ready pin
    QueueHandle_t gpio_evt_queue;  // pin triggered queue
    // Updated queue handler -N
} ads1115_rdy_pin_t;

typedef struct {
    ADS1115_CONFIG_REGISTER_Type config;
    i2c_port_t i2c_port;
    int address;
    ads1115_rdy_pin_t rdy_pin;
    ads1115_register_addresses_t last_reg;  // save last accessed register
    bool changed;          // save if a value was changed or not
    TickType_t max_ticks;  // maximum wait ticks for i2c bus
} ads1115_t;

// initialize device
ads1115_t ads1115_config(i2c_port_t i2c_port,
                         uint8_t address);  // set up configuration

// set configuration
void ads1115_set_rdy_pin(ads1115_t *ads,
                         gpio_num_t gpio);  // set up data-ready pin
void ads1115_set_mux(ads1115_t *ads, ads1115_mux_t mux);     // set multiplexer
void ads1115_set_pga(ads1115_t *ads, ads1115_fsr_t fsr);     // set fsr
void ads1115_set_mode(ads1115_t *ads, ads1115_mode_t mode);  // set read mode
void ads1115_set_sps(ads1115_t *ads, ads1115_sps_t sps);  // set sampling speed
void ads1115_set_max_ticks(
    ads1115_t *ads, TickType_t max_ticks);  // maximum wait ticks for i2c bus

// FreeRTOS task for initialization and data transmission -N
void ads1115_task(void *pvParameters);
int16_t ads1115_get_raw(ads1115_t *ads);     // get signal in bits
double ads1115_get_voltage(ads1115_t *ads);  // get signsl in volts

#ifdef __cplusplus
}
#endif
#endif /* ADS1115_SENSOR */
#endif /* ADS1115_H */