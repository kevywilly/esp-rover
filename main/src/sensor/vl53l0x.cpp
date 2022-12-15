//
// Created by Kevin Williams on 11/20/22.
//

#include <thread>
#include "vl53l0x.h"

static const char * TAG = "VL53L0X";

VL53L0X::VL53L0X(uint8_t port, uint8_t address, gpio_num_t xshut) : port(port), address(address), xshut(xshut) {
    io_2v8 = 1;
    did_timeout = 0;
    i2c_fail = 0;
    io_timeout = 100;
}

const char * VL53L0X::init() {
    const char *err;
    // Set up the VL53L0X
                      // XSHUT or power control
    gpio_set_level ((gpio_num_t)xshut, 0);     // Off
    usleep (100000);
    gpio_set_level ((gpio_num_t)xshut, 1);     // On
    usleep (10000);           // Plenty of time to boot (data sheet says 1.2ms)

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if (io_2v8)
        writeReg8Bit( VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, readReg8Bit(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);   // set bit 0
    // "Set I2C standard mode"
    writeReg8Bit( 0x88, 0x00);

    writeReg8Bit( 0x80, 0x01);
    writeReg8Bit( 0xFF, 0x01);
    writeReg8Bit( 0x00, 0x00);
    stop_variable = readReg8Bit(0x91);
    writeReg8Bit( 0x00, 0x01);
    writeReg8Bit( 0xFF, 0x00);
    writeReg8Bit( 0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    writeReg8Bit( MSRC_CONFIG_CONTROL, readReg8Bit (MSRC_CONFIG_CONTROL) | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    if ((err = setSignalRateLimit (0.25)))
        return err;

    writeReg8Bit( SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin

    uint8_t spad_count;
    int spad_type_is_aperture;
    if ((err = getSpadInfo (&spad_count, &spad_type_is_aperture)))
        return err;
    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    writeReg8Bit( 0xFF, 0x01);
    writeReg8Bit( DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    writeReg8Bit( DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    writeReg8Bit( 0xFF, 0x00);
    writeReg8Bit( GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;       // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x00, 0x00);

    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x09, 0x00);
    writeReg8Bit (0x10, 0x00);
    writeReg8Bit (0x11, 0x00);

    writeReg8Bit (0x24, 0x01);
    writeReg8Bit (0x25, 0xFF);
    writeReg8Bit (0x75, 0x00);

    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x4E, 0x2C);
    writeReg8Bit (0x48, 0x00);
    writeReg8Bit (0x30, 0x20);

    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x30, 0x09);
    writeReg8Bit (0x54, 0x00);
    writeReg8Bit (0x31, 0x04);
    writeReg8Bit (0x32, 0x03);
    writeReg8Bit (0x40, 0x83);
    writeReg8Bit (0x46, 0x25);
    writeReg8Bit (0x60, 0x00);
    writeReg8Bit (0x27, 0x00);
    writeReg8Bit (0x50, 0x06);
    writeReg8Bit (0x51, 0x00);
    writeReg8Bit (0x52, 0x96);
    writeReg8Bit (0x56, 0x08);
    writeReg8Bit (0x57, 0x30);
    writeReg8Bit (0x61, 0x00);
    writeReg8Bit (0x62, 0x00);
    writeReg8Bit (0x64, 0x00);
    writeReg8Bit (0x65, 0x00);
    writeReg8Bit (0x66, 0xA0);

    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x22, 0x32);
    writeReg8Bit (0x47, 0x14);
    writeReg8Bit (0x49, 0xFF);
    writeReg8Bit (0x4A, 0x00);

    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x7A, 0x0A);
    writeReg8Bit (0x7B, 0x00);
    writeReg8Bit (0x78, 0x21);

    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x23, 0x34);
    writeReg8Bit (0x42, 0x00);
    writeReg8Bit (0x44, 0xFF);
    writeReg8Bit (0x45, 0x26);
    writeReg8Bit (0x46, 0x05);
    writeReg8Bit (0x40, 0x40);
    writeReg8Bit (0x0E, 0x06);
    writeReg8Bit (0x20, 0x1A);
    writeReg8Bit (0x43, 0x40);

    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x34, 0x03);
    writeReg8Bit (0x35, 0x44);

    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x31, 0x04);
    writeReg8Bit (0x4B, 0x09);
    writeReg8Bit (0x4C, 0x05);
    writeReg8Bit (0x4D, 0x04);

    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x44, 0x00);
    writeReg8Bit (0x45, 0x20);
    writeReg8Bit (0x47, 0x08);
    writeReg8Bit (0x48, 0x28);
    writeReg8Bit (0x67, 0x00);
    writeReg8Bit (0x70, 0x04);
    writeReg8Bit (0x71, 0x01);
    writeReg8Bit (0x72, 0xFE);
    writeReg8Bit (0x76, 0x00);
    writeReg8Bit (0x77, 0x00);

    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x0D, 0x01);

    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x80, 0x01);
    writeReg8Bit (0x01, 0xF8);

    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x8E, 0x01);
    writeReg8Bit (0x00, 0x01);
    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x80, 0x00);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    writeReg8Bit( SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    writeReg8Bit( GPIO_HV_MUX_ACTIVE_HIGH, readReg8Bit(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    writeReg8Bit( SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    measurement_timing_budget_us = getMeasurementTimingBudget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    writeReg8Bit( SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    if ((err = setMeasurementTimingBudget (measurement_timing_budget_us)))
        return err;

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    writeReg8Bit( SYSTEM_SEQUENCE_CONFIG, 0x01);
    if ((err = performSingleRefCalibration (0x40)))
        return err;
    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    writeReg8Bit( SYSTEM_SEQUENCE_CONFIG, 0x02);
    if ((err = performSingleRefCalibration (0x00)))
        return err;
    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    writeReg8Bit( SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end
    if (i2cFail())
        return "I2C fail";

    return NULL;
}

void VL53L0X::end() {
    i2c_driver_delete (port);
}

void VL53L0X::setAddress(uint8_t new_addr) {
    writeReg8Bit(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    address = new_addr;
}

uint8_t VL53L0X::getAddress() {
    return address;
}

void VL53L0X::writeReg8Bit(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t i = Write (reg);
    i2c_master_write_byte (i, val, true);
    esp_err_t err = Done (i);
    TOF_SENSOR_LOG(TAG, "W %02X=%02X %s", reg, val, esp_err_to_name (err));
}

void VL53L0X::writeReg16Bit(uint8_t reg, uint16_t val) {
    i2c_cmd_handle_t i = Write (reg);
    i2c_master_write_byte (i, val >> 8, true);
    i2c_master_write_byte (i, val, true);
    esp_err_t err = Done (i);
    TOF_SENSOR_LOG (TAG, "W %02X=%04X %s", reg, val, esp_err_to_name (err));
}

void VL53L0X::writeReg32Bit(uint8_t reg, uint32_t val) {
    i2c_cmd_handle_t i = Write (reg);
    i2c_master_write_byte (i, val >> 24, 1);
    i2c_master_write_byte (i, val >> 16, 1);
    i2c_master_write_byte (i, val >> 8, 1);
    i2c_master_write_byte (i, val, 1);
    esp_err_t err = Done (i);
    TOF_SENSOR_LOG (TAG, "W %02X=%08X %s", reg, val, esp_err_to_name (err));
}

uint8_t VL53L0X::readReg8Bit(uint8_t reg) {
    uint8_t buf[1] = { };
    i2c_cmd_handle_t i = Read (reg);
    i2c_master_read_byte (i, buf + 0, I2C_MASTER_LAST_NACK);
    esp_err_t err = Done (i);
    TOF_SENSOR_LOG (TAG, "R %02X=%02X %s", reg, buf[0], esp_err_to_name (err));
    return buf[0];
}

uint16_t VL53L0X::readReg16Bit(uint8_t reg) {
    uint8_t buf[2] = { };
    i2c_cmd_handle_t i = Read (reg);
    i2c_master_read_byte (i, buf + 0, I2C_MASTER_ACK);
    i2c_master_read_byte (i, buf + 1, I2C_MASTER_LAST_NACK);
    esp_err_t err = Done (i);
    TOF_SENSOR_LOG (TAG, "R %02X=%02X%02X %s", reg, buf[0], buf[1], esp_err_to_name (err));
    return (buf[0] << 8) + buf[1];
}

uint32_t VL53L0X::readReg32Bit(uint8_t reg) {
    uint8_t buf[4] = { };
    i2c_cmd_handle_t i = Read (reg);
    i2c_master_read_byte (i, buf + 0, I2C_MASTER_ACK);
    i2c_master_read_byte (i, buf + 1, I2C_MASTER_ACK);
    i2c_master_read_byte (i, buf + 2, I2C_MASTER_ACK);
    i2c_master_read_byte (i, buf + 3, I2C_MASTER_LAST_NACK);
    esp_err_t err = Done (i);
    TOF_SENSOR_LOG (TAG, "R %02X=%02X%02X%02X%02X %s", reg, buf[0], buf[1], buf[2], buf[3], esp_err_to_name (err));
    return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

void VL53L0X::writeMulti(uint8_t reg, const uint8_t *src, uint8_t count) {
    i2c_cmd_handle_t i = Write (reg);
    i2c_master_write (i, (uint8_t *) src, count, 1);
    esp_err_t err = Done (i);
    TOF_SENSOR_LOG (TAG, "W %02X (%d) %s", reg, count, esp_err_to_name (err));
}

void VL53L0X::readMulti(uint8_t reg, uint8_t *dst, uint8_t count) {
    i2c_cmd_handle_t i = Read (reg);
    if (count > 1)
        i2c_master_read (i, dst + 0, count - 1, I2C_MASTER_ACK);
    i2c_master_read_byte (i, dst + count - 1, I2C_MASTER_LAST_NACK);
    esp_err_t err = Done ( i);
    TOF_SENSOR_LOG (TAG, "R %02X (%d) %s", reg, count, esp_err_to_name (err));
}

const char *VL53L0X::setSignalRateLimit(float limit_Mcps) {
    if (limit_Mcps < 0 || limit_Mcps > 511.99)
        return "Bad rate";
    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    writeReg16Bit (FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
    return NULL;
}

float VL53L0X::getSignalRateLimit() {
    return (float) readReg16Bit ( FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

const char *VL53L0X::getSpadInfo(uint8_t *count, int *type_is_aperture) {
    uint8_t tmp;

    writeReg8Bit (0x80, 0x01);
    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x00, 0x00);

    writeReg8Bit (0xFF, 0x06);
    writeReg8Bit (0x83, readReg8Bit (0x83) | 0x04);
    writeReg8Bit (0xFF, 0x07);
    writeReg8Bit (0x81, 0x01);

    writeReg8Bit (0x80, 0x01);

    writeReg8Bit (0x94, 0x6b);
    writeReg8Bit (0x83, 0x00);
    startTimeout ();
    while (readReg8Bit (0x83) == 0x00)
    {
        if (checkTimeoutExpired ())
            return "SPAD Timeout";
    }
    writeReg8Bit (0x83, 0x01);
    tmp = readReg8Bit (0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    writeReg8Bit (0x81, 0x00);
    writeReg8Bit (0xFF, 0x06);
    writeReg8Bit (0x83, readReg8Bit (0x83) & ~0x04);
    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x00, 0x01);

    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x80, 0x00);

    return NULL;
}

const char *VL53L0X::setMeasurementTimingBudget(uint32_t budget_us) {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget)
        return "Low budget";

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables (&enables);
    getSequenceStepTimeouts (&enables, &timeouts);

    if (enables.tcc)
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

    if (enables.dss)
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc)
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

    if (enables.pre_range)
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us)
            return "High budget";  // "Requested timeout too big."

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks (final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
            final_range_timeout_mclks += timeouts.pre_range_mclks;

        writeReg16Bit (FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return NULL;
}

uint32_t VL53L0X::getMeasurementTimingBudget() {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables (&enables);
    getSequenceStepTimeouts (&enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us;    // store for internal reuse
    return budget_us;
}

const char *VL53L0X::setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) {
    return nullptr;
}

uint8_t VL53L0X::getVcselPulsePeriod(vcselPeriodType type) {
    if (type == VcselPeriodPreRange)
    {
        return decodeVcselPeriod (readReg8Bit (PRE_RANGE_CONFIG_VCSEL_PERIOD));
    } else if (type == VcselPeriodFinalRange)
    {
        return decodeVcselPeriod (readReg8Bit (FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    } else
    {
        return 255;
    }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X::startContinuous(uint32_t period_ms) {
    writeReg8Bit (0x80, 0x01);
    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x00, 0x00);
    writeReg8Bit (0x91, stop_variable);
    writeReg8Bit (0x00, 0x01);
    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x80, 0x00);

    if (period_ms != 0)
    {
        // continuous timed mode

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

        uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);

        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }

        writeReg32Bit (SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        writeReg8Bit (SYSRANGE_START, 0x04);   // VL53L0X_REG_SYSRANGE_MODE_TIMED
    } else
    {
        // continuous back-to-back mode
        writeReg8Bit (SYSRANGE_START, 0x02);   // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void VL53L0X::stopContinuous() {
    writeReg8Bit (SYSRANGE_START, 0x01);      // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x00, 0x00);
    writeReg8Bit (0x91, 0x00);
    writeReg8Bit (0x00, 0x01);
    writeReg8Bit (0xFF, 0x00);
}

uint16_t VL53L0X::readRangeContinuousMillimeters() {
    startTimeout ();
    while ((readReg8Bit (RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired ())
        {
            did_timeout = 1;
            return 0;
        }
    }
    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    uint16_t range = readReg16Bit (RESULT_RANGE_STATUS + 10);
    writeReg8Bit (SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range;
}

uint16_t VL53L0X::readRangeSingleMillimeters() {
    writeReg8Bit (0x80, 0x01);
    writeReg8Bit (0xFF, 0x01);
    writeReg8Bit (0x00, 0x00);
    writeReg8Bit (0x91, stop_variable);
    writeReg8Bit  ( 0x00, 0x01);
    writeReg8Bit (0xFF, 0x00);
    writeReg8Bit (0x80, 0x00);

    writeReg8Bit (SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    startTimeout ();
    while (readReg8Bit(SYSRANGE_START) & 0x01)
    {
        if (checkTimeoutExpired ())
        {
            did_timeout = 1;
            return 0;
        }
    }

    return readRangeContinuousMillimeters();
}

void VL53L0X::setTimeout(uint16_t timeout) {
    io_timeout = timeout;
}

uint16_t VL53L0X::getTimeout() {
    return io_timeout;
}

int VL53L0X::timeoutOccurred() {
    int tmp = did_timeout;
    did_timeout = 0;
    return tmp;
}

int VL53L0X::i2cFail() {
    int tmp = i2c_fail;
    i2c_fail = 0;
    return tmp;
}

const char *VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte) {
    writeReg8Bit (SYSRANGE_START, 0x01 | vhv_init_byte);      // VL53L0X_REG_SYSRANGE_MODE_START_STOP
    startTimeout ();
    while ((readReg8Bit (RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired ())
            return "CAL Timeout";
    }
    writeReg8Bit (SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg8Bit (SYSRANGE_START, 0x00);
    return NULL;
}

void VL53L0X::getSequenceStepEnables(SequenceStepEnables *enables) {
    uint8_t sequence_config = readReg8Bit (SYSTEM_SEQUENCE_CONFIG);

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

void VL53L0X::getSequenceStepTimeouts(const SequenceStepEnables *enables, SequenceStepTimeouts *timeouts) {
    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod (VcselPeriodPreRange);

    timeouts->msrc_dss_tcc_mclks = readReg8Bit (MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds (timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks = decodeTimeout (readReg16Bit (PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us = timeoutMclksToMicroseconds (timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod (VcselPeriodFinalRange);

    timeouts->final_range_mclks = decodeTimeout (readReg16Bit (FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeoutMclksToMicroseconds (timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

esp_err_t VL53L0X::Done(i2c_cmd_handle_t i) {
    i2c_master_stop (i);
    esp_err_t err = i2c_master_cmd_begin (port, i, TIMEOUT);
    if (err)
        i2c_fail = 1;
    i2c_cmd_link_delete (i);
#ifdef tBUF
    usleep (tBUF);
#endif
    return err;
}

i2c_cmd_handle_t VL53L0X::Read(uint8_t reg) {
    i2c_cmd_handle_t i = i2c_cmd_link_create ();
    i2c_master_start (i);
    i2c_master_write_byte (i, (address << 1), 1);
    i2c_master_write_byte (i, reg, 1);
    Done (i);
    i = i2c_cmd_link_create ();
    i2c_master_start (i);
    i2c_master_write_byte (i, (address << 1) + 1, 1);
    return i;
}

i2c_cmd_handle_t VL53L0X::Write(uint8_t reg) {
    i2c_cmd_handle_t i = i2c_cmd_link_create ();
    i2c_master_start (i);
    i2c_master_write_byte (i, (address << 1), 1);
    i2c_master_write_byte (i, reg, 1);
    return i;
}
