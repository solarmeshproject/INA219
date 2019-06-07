#ifndef INA219_H__
#define INA219_H__

/*
    Hello my frend!
    it's library for connect and work nrf51422 board and ina219 chip for i2c interface
    for all work you must chenge your uart pint in nrf51422 board and run ina219_init function 
    with actual params
    good luck.
*/
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#define SCL_PIN 7
#define SDA_PIN 30

#define RX_PIN_NUMBER 1
#define TX_PIN_NUMBER 2
#define RTS_PIN_NUMBER 3
#define CTS_PIN_NUMBER 4

#define D_SHUNT            0.1
#define D_V_BUS_MAX        32
#define D_V_SHUNT_MAX      0.2
#define D_I_MAX_EXPECTED   2

#define CNVR_B                       1                  // conversion ready bit in bus voltage register V_BUS_R 
#define OVF_B                        0                  // math overflow bit in bus voltage register V_BUS_R 
#define INA_RESET          0xFFFF     // send to CONFIG_R to reset unit


#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)
        
enum t_i2caddr{
      I2C_ADDR_40 = 0x40, ///< address 0x40 no jumpers required.
      I2C_ADDR_41 = 0x41, ///< address 0x41 bridge A0.
      I2C_ADDR_44 = 0x44, ///< address 0x44 bridge A1.
      I2C_ADDR_45 = 0x45 ///< address 0x45 bridge A0 & A1.
    };

enum t_gain{
        GAIN_1_40MV = 0,
        GAIN_2_80MV = 1,
        GAIN_4_160MV = 2,
        GAIN_8_320MV = 3
    };      
    
enum t_range{
        RANGE_16V = 0, ///< Range 0-16 volts
        RANGE_32V = 1  ///< Range 0-32 volts
    };      
        
enum t_adc{
        ADC_9BIT    = 0,  ///<  9bit converion time  84us.
        ADC_10BIT   = 1,  ///< 10bit converion time 148us.
        ADC_11BIT   = 2,  ///< 11bit converion time 2766us.
        ADC_12BIT   = 3,  ///< 12bit converion time 532us.
        ADC_2SAMP   = 9,  ///< 2 samples converion time 1.06ms.
        ADC_4SAMP   = 10, ///< 4 samples converion time 2.13ms.
        ADC_8SAMP   = 11, ///< 8 samples converion time 4.26ms.
        ADC_16SAMP  = 12, ///< 16 samples converion time 8.51ms
        ADC_32SAMP  = 13, ///< 32 samples converion time 17.02ms.
        ADC_64SAMP  = 14, ///< 64 samples converion time 34.05ms.
        ADC_128SAMP = 15, ///< 128 samples converion time 68.10ms.
    };      
        
enum t_mode{
        PWR_DOWN    = 0,
        /*
        TRIG_SH     = 1,
        TRIG_BUS    = 2,
        TRIG_SH_BUS = 3,
        */
        ADC_OFF     = 4,
        CONT_SH     = 5, ///<Shunt Continuous.
        CONT_BUS    = 6, ///<Bus Continuous.
        CONT_SH_BUS = 7  ///<Shunt and Bus, Continuous.
    };

enum t_reg{
        CONFIG_R  = 0x00,    ///< configuration register.
        V_SHUNT_R = 0x01,    ///< differential shunt voltage.
        V_BUS_R   = 0x02,    ///< bus voltage (wrt to system/chip GND).
        P_BUS_R   = 0x03,    ///< system power draw (= V_BUS * I_SHUNT).
        I_SHUNT_R = 0x04,    ///< shunt current.
        CAL_R     = 0x05     ///< calibration register.
    };      


//Init hardwares in your chip   
void uart_events_handler(app_uart_evt_t * p_event);
bool CHECK_BIT(uint8_t var, uint8_t pos);
void uart_config(void);	
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);  
void twi_init (void);
//---------------------------       

//init and calebrate function for your chip ina219      
void calibrate_ina219(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected);
void configure_ina219(enum t_range range,  enum t_gain gain,  enum t_adc  bus_adc,  enum t_adc shunt_adc,  enum t_mode mode);   

//This users functions      
void reset_ina219(void);
int16_t shuntVoltageRaw_ina219(void);
float shuntVoltage_ina219(void);
int16_t busVoltageRaw_ina219(void);
float busVoltage_ina219(void);
int16_t shuntCurrentRaw_ina219(void);
float shuntCurrent_ina219(void);
float busPower_ina219(void);
void reconfig_ina219(void);
void recalibrate_ina219(void);
bool ready_ina219(void);
bool overflow_ina219(void);
void ina219_init(float shunt_val, 
                                    float v_shunt_max, 
                                    float v_bus_max, 
                                    float i_max_expected, 
                                        enum t_range range,  
                                        enum t_gain gain,  
                                        enum t_adc  bus_adc,  
                                        enum t_adc shunt_adc, 
                                        enum t_mode mode);

//This privat function, for get number bit sets in the byte
bool IsBitSet(uint8_t b, int pos);
                                        
#endif // INA219_H__
