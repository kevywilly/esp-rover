//
// Created by Kevin Williams on 11/20/22.
//

#ifndef ESPROVER_TOF_SENSOR_HPP
#define ESPROVER_TOF_SENSOR_HPP

#include <stdio.h>
#include <driver/i2c.h>
#include <esp_log.h>

#define TIMEOUT   (10/portTICK_PERIOD_MS) // I2C command timeout

#ifdef   CONFIG_VL53L0X_DEBUG
#define TOF_SENSOR_LOG   ESP_LOGI        // Set to allow I2C logging
#endif

#ifndef TOF_SENSOR_LOG
#define TOF_SENSOR_LOG(tag,...) err=err;
#endif

typedef struct
{
    uint8_t tcc:1;
    uint8_t msrc:1;
    uint8_t dss:1;
    uint8_t pre_range:1;
    uint8_t final_range:1;
} SequenceStepEnables;

typedef struct
{
    uint16_t pre_range_vcsel_period_pclks,
            final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks,
            pre_range_mclks,
            final_range_mclks;
    uint32_t msrc_dss_tcc_us,
            pre_range_us,
            final_range_us;
}
        SequenceStepTimeouts;

enum
{
    SYSRANGE_START = 0x00,

    SYSTEM_THRESH_HIGH = 0x0C,
    SYSTEM_THRESH_LOW = 0x0E,

    SYSTEM_SEQUENCE_CONFIG = 0x01,
    SYSTEM_RANGE_CONFIG = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

    GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

    SYSTEM_INTERRUPT_CLEAR = 0x0B,

    RESULT_INTERRUPT_STATUS = 0x13,
    RESULT_RANGE_STATUS = 0x14,

    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

    ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

    MSRC_CONFIG_CONTROL = 0x60,

    PRE_RANGE_CONFIG_MIN_SNR = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

    FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

    PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

    PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

    SYSTEM_HISTOGRAM_BIN = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

    FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

    MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

    I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

    SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
    IDENTIFICATION_MODEL_ID = 0xC0,
    IDENTIFICATION_REVISION_ID = 0xC2,

    OSC_CALIBRATE_VAL = 0xF8,

    GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

    GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

    ALGO_PHASECAL_LIM = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

typedef enum
{ VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

class TOFSensor {
public:
    TOFSensor(uint8_t port, uint8_t address, uint8_t xshut);
    const char * init();
    uint16_t readRangeContinuousMillimeters ();
    uint16_t readRangeSingleMillimeters ();
    uint16_t getTimeout ();
    int timeoutOccurred ();
    int i2cFail ();
    void end();
    void setAddress(uint8_t new_addr);
    uint8_t getAddress();
    void startContinuous (uint32_t period_ms);
    void stopContinuous ();

private:
    void writeReg8Bit(uint8_t reg, uint8_t val);
    void writeReg16Bit(uint8_t reg, uint16_t val);
    void writeReg32Bit(uint8_t reg, uint32_t val);
    uint8_t readReg8Bit(uint8_t reg);
    uint16_t readReg16Bit(uint8_t reg);
    uint32_t readReg32Bit(uint8_t reg);
    void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count);
    void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);
    const char * setSignalRateLimit(float limit_Mcps);
    float getSignalRateLimit ();
    const char *setMeasurementTimingBudget (uint32_t budget_us);
    uint32_t getMeasurementTimingBudget ();
    const char *getSpadInfo (uint8_t * count, int *type_is_aperture);
    const char *setVcselPulsePeriod (vcselPeriodType type, uint8_t period_pclks);
    uint8_t getVcselPulsePeriod (vcselPeriodType type);

    void getSequenceStepEnables (SequenceStepEnables * enables);
    void getSequenceStepTimeouts (SequenceStepEnables const *enables, SequenceStepTimeouts * timeouts);
    void setTimeout (uint16_t timeout);

    const char * performSingleRefCalibration (uint8_t vhv_init_byte);

    inline uint32_t millis() {return (esp_timer_get_time()/1000LL);}

    // Record the current time to check an upcoming timeout against
    inline void startTimeout() {timeout_start_ms = millis();}

    // Encode VCSEL pulse period register value from period in PCLKs
    // based on VL53L0X_encode_vcsel_period()
    static uint8_t encodeVcselPeriod(uint8_t period_pclks) {return ((period_pclks) >> 1) - 1;}

    inline bool checkTimeoutExpired() {
        // Check if timeout is enabled (set to nonzero value) and has expired
        return (io_timeout > 0 && ((uint16_t)(millis() - timeout_start_ms)) > io_timeout);
    }

    // Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
    // from register value
    // based on VL53L0X_decode_vcsel_period()
    static uint8_t decodeVcselPeriod(uint8_t reg_val) { return (((reg_val) + 1) << 1); }

    // Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
    // based on VL53L0X_calc_macro_period_ps()
    // PLL_period_ps = 1655; macro_period_vclks = 2304
    static uint32_t calcMacroPeriod(uint32_t vcsel_period_pclks) {
        return ((((uint32_t) 2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
    }

    // Decode sequence step timeout in MCLKs from register value
    // based on VL53L0X_decode_timeout()
    // Note: the original function returned a uint32_t, but the return value is
    // always stored in a uint16_t.
    static uint16_t decodeTimeout (uint16_t reg_val)
    {
        // format: "(LSByte * 2^MSByte) + 1"
        return (uint16_t) ((reg_val & 0x00FF) << (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
    }

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
    static uint32_t timeoutMclksToMicroseconds (uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
        uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
    }

    // Encode sequence step timeout register value from timeout in MCLKs
    // based on VL53L0X_encode_timeout()
    // Note: the original function took a uint16_t, but the argument passed to it
    // is always a uint16_t.
    static uint16_t encodeTimeout (uint16_t timeout_mclks)
    {
        // format: "(LSByte * 2^MSByte) + 1"

        uint32_t ls_byte = 0;
        uint16_t ms_byte = 0;

        if (timeout_mclks > 0)
        {
            ls_byte = timeout_mclks - 1;

            while ((ls_byte & 0xFFFFFF00) > 0)
            {
                ls_byte >>= 1;
                ms_byte++;
            }

            return (ms_byte << 8) | (ls_byte & 0xFF);
        } else
        {
            return 0;
        }
    }

    // Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
    static uint32_t timeoutMicrosecondsToMclks (uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
    {
        uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);

        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    }
    
public:
    uint8_t port;
    uint8_t address;
    uint8_t xshut;
    uint16_t io_timeout;
    uint8_t io_2v8:1;
    uint8_t did_timeout:1;
    uint8_t i2c_fail:1;

private:
    uint8_t stop_variable;
    uint16_t timeout_start_ms;
    uint32_t measurement_timing_budget_us;


};

static esp_err_t
Done (TOFSensor * v, i2c_cmd_handle_t i)
{
    i2c_master_stop (i);
    esp_err_t err = i2c_master_cmd_begin (v->port, i, TIMEOUT);
    if (err)
        v->i2c_fail = 1;
    i2c_cmd_link_delete (i);
#ifdef tBUF
    usleep (tBUF);
#endif
    return err;
}

static i2c_cmd_handle_t Read (TOFSensor * v, uint8_t reg) {                               // Set up for read
    i2c_cmd_handle_t i = i2c_cmd_link_create ();
    i2c_master_start (i);
    i2c_master_write_byte (i, (v->address << 1), 1);
    i2c_master_write_byte (i, reg, 1);
    Done (v, i);
    i = i2c_cmd_link_create ();
    i2c_master_start (i);
    i2c_master_write_byte (i, (v->address << 1) + 1, 1);
    return i;
}

static i2c_cmd_handle_t Write (TOFSensor * v, uint8_t reg) {                               // Set up for write
    i2c_cmd_handle_t i = i2c_cmd_link_create ();
    i2c_master_start (i);
    i2c_master_write_byte (i, (v->address << 1), 1);
    i2c_master_write_byte (i, reg, 1);
    return i;
}
#endif //ESPROVER_TOF_SENSOR_HPP
