#include "ina219.h"


const uint8_t RST =	15;
const uint8_t BRNG = 13;
const uint8_t PG1 = 12;
const uint8_t PG0 = 11;
const uint8_t BADC4 = 10;
const uint8_t BADC3	= 9;
const uint8_t BADC2	= 8;
const uint8_t BADC1	= 7;
const uint8_t SADC4	= 6;
const uint8_t SADC3	= 5;
const uint8_t SADC2	= 4;
const uint8_t SADC1	= 3;
const uint8_t MODE3	= 2;
const uint8_t MODE2	= 1;
const uint8_t MODE1	= 0;
static const nrf_drv_twi_t twi_ina219 = NRF_DRV_TWI_INSTANCE(0);

bool _ready, _overflow;
float r_shunt, current_lsb, power_lsb;
uint16_t *config, *cal;		
		
void uart_events_handler(app_uart_evt_t * p_event){
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

 bool CHECK_BIT(uint8_t var, uint8_t pos){
	if (((var)>>(pos)) & 1) return true;
	else return false;
 }

void uart_config(void){
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context){   
    ret_code_t err_code;
	  uint8_t *datas;   
    
    switch(p_event->type){
        case NRF_DRV_TWI_EVT_DONE:
            if ((p_event->type == NRF_DRV_TWI_EVT_DONE) &&
                (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)){                
                err_code = nrf_drv_twi_rx(&twi_ina219, I2C_ADDR_40, datas, sizeof(datas));
                APP_ERROR_CHECK(err_code);
            }            
            break;
        default:
            break;        
    }   
}

void twi_init (void){
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&twi_ina219, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&twi_ina219);
}

void calibrate_ina219(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected) {
    uint16_t digits;
    ret_code_t err_code;	
    float min_lsb, swap;
    r_shunt     = shunt_val;
    min_lsb     = i_max_expected / 32767;
    current_lsb = min_lsb;
    digits      = 0;
    
    while( current_lsb > 0){//If zero there is something weird...
        if( (uint16_t)current_lsb / 1){
          current_lsb = current_lsb + 1;
					double tmp = 10;
					for (uint8_t i = 0; i <= digits; i++)
					tmp = tmp * tmp;
          current_lsb = current_lsb /tmp;
          break;
        }
        else{
          digits++;
          current_lsb = current_lsb * 10;
        }
    };

    swap      = (double)(0.04096)/(current_lsb*r_shunt);
    *cal       = (uint16_t)swap;
    power_lsb = current_lsb * 20;

    err_code  = nrf_drv_twi_tx(&twi_ina219, CAL_R, (uint8_t *)cal, sizeof(cal), false);  
    APP_ERROR_CHECK(err_code);    
}


void configure_ina219(enum t_range range,  enum t_gain gain,  enum t_adc  bus_adc,  enum t_adc shunt_adc,  enum t_mode mode) {
  ret_code_t err_code;		
  config = 0;
  config =  (uint16_t*) (0 | ((range << BRNG | gain << PG0 | bus_adc << BADC1 | shunt_adc << SADC1 | mode)));
  err_code = nrf_drv_twi_tx(&twi_ina219, CONFIG_R, (uint8_t *)config, sizeof(config), false);  
  APP_ERROR_CHECK(err_code); 
}

void reset_ina219(void){
	ret_code_t err_code;
  err_code = nrf_drv_twi_tx(&twi_ina219, CONFIG_R, (uint8_t*)INA_RESET, sizeof(INA_RESET), false);  
  APP_ERROR_CHECK(err_code);  
  nrf_delay_ms(5);    
}

int16_t shuntVoltageRaw_ina219(void){
  uint16_t *datas;
	ret_code_t err_code;
  err_code = nrf_drv_twi_rx(&twi_ina219, V_SHUNT_R, (uint8_t*)datas, sizeof(int16_t));
  APP_ERROR_CHECK(err_code);
  return *datas;
}

float shuntVoltage_ina219(void){  
	uint16_t *datas;
	ret_code_t err_code;
	err_code = nrf_drv_twi_rx(&twi_ina219, V_SHUNT_R, (uint8_t*)datas, sizeof(int16_t));  
  APP_ERROR_CHECK(err_code);	
	uint16_t tmp = *datas;
  return (tmp / 100000);
}

int16_t busVoltageRaw_ina219(void) {
  uint16_t *_bus_voltage_register;
	ret_code_t err_code;
  err_code = nrf_drv_twi_rx(&twi_ina219, V_BUS_R, (uint8_t*)_bus_voltage_register, sizeof(int16_t));  
  APP_ERROR_CHECK(err_code);  
  _overflow = CHECK_BIT(*_bus_voltage_register, OVF_B);     // overflow bit
  _ready    = CHECK_BIT(*_bus_voltage_register, CNVR_B);    // ready bit
  return *_bus_voltage_register;
}

float busVoltage_ina219(void) {
  int16_t temp;
  temp = busVoltageRaw_ina219();
  temp >>= 3;
  return (temp * 0.004);
}

int16_t shuntCurrentRaw_ina219(void){
  uint16_t *datas;
	ret_code_t err_code;
  err_code = nrf_drv_twi_rx(&twi_ina219, V_BUS_R, (uint8_t*)datas, sizeof(int16_t));  
  APP_ERROR_CHECK(err_code);
  return *datas;
}

float shuntCurrent_ina219(void){
  uint16_t *datas;
	ret_code_t err_code;
  err_code = nrf_drv_twi_rx(&twi_ina219, I_SHUNT_R, (uint8_t*)datas, sizeof(int16_t));  
  APP_ERROR_CHECK(err_code);  
	float tmp = *datas;
  return (tmp * current_lsb);
}

float busPower_ina219(void){
  uint16_t *datas;
	ret_code_t err_code;
  err_code = nrf_drv_twi_rx(&twi_ina219, P_BUS_R, (uint8_t*)datas, sizeof(int16_t));  
  APP_ERROR_CHECK(err_code);  
	float tmp = *datas;
  return (tmp * power_lsb);
}

void reconfig_ina219(void){
	ret_code_t err_code;
  err_code = nrf_drv_twi_tx(&twi_ina219, CONFIG_R, (uint8_t*)config, sizeof(config), false);  
  APP_ERROR_CHECK(err_code);    
}

void recalibrate_ina219(void){
	ret_code_t err_code;
  err_code = nrf_drv_twi_tx(&twi_ina219, CAL_R, (uint8_t*)cal, sizeof(cal), false);  
  APP_ERROR_CHECK(err_code);      
}

bool ready_ina219(void){
  return _ready;
}

bool overflow_ina219(void){
  return _overflow;
}

void ina219_init(float shunt_val, 
                  float v_shunt_max, 
                  float v_bus_max, 
                  float i_max_expected, 
                    enum t_range range,  
                    enum t_gain gain,  
                    enum t_adc  bus_adc,  
                    enum t_adc shunt_adc, 
                    enum t_mode mode){
  uart_config();  
  twi_init();
  calibrate_ina219(shunt_val, v_shunt_max, v_bus_max, i_max_expected);
  configure_ina219(range,  gain,  bus_adc,  shunt_adc,  mode);
};  
