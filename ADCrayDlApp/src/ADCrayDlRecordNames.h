#ifndef ADCRAYDL_RECORDNAMES_H
#define ADCRAYDL_RECORDNAMES_H

namespace adcraydl
{

/*
 * Driver acquisition records.
 */
static const char *READOUT_MODE_STR             = "Craydl_ReadoutMode";
static const char *ACQUIRE_PEDESTAL_STR         = "AcquirePedestal";
static const char *PEDESTAL_NUM_IMG_STR         = "PedestalNumImages";
static const char *PEDESTAL_TIMESTAMP_STR       = "PedestalTimestamp";
static const char *SHUTTER_STATUS_STR           = "ShutterStatus";
static const char *ENABLE_DETECTOR_QUERYING_STR = "EnableDetectorQuerying";
static const char *BINNING_STR                  = "Binning";

/*
 * Cooling records.
 */
static const char *COOLER_STR               = "Cooler";
static const char *CCD_TEMP_SETPOINT_STR    = "CCDTempSetpoint";
static const char *COOLER_TEMP_SETPOINT_STR = "CoolerTempSetpoint";
static const char *MEAN_SENSOR_TEMP_STR     = "MeanSensorTemp";
static const char *MAX_SENSOR_TEMP_STR      = "MaxSensorTemp";
static const char *MIN_SENSOR_TEMP_STR      = "MinSensorTemp";
static const char *MEAN_COOLER_TEMP_STR     = "MeanCoolerTemp";
static const char *MAX_COOLER_TEMP_STR      = "MaxCoolerTemp";
static const char *MIN_COOLER_TEMP_STR      = "MinCoolerTemp";
static const char *COOLER_ENABLED_STR       = "CoolerEnabled";
static const char *COOLER_RUNNING_STR       = "CoolerRunning";

/*
 * Vacuum records.
 */
static const char *VACUUM_VALVE_STR         = "VacuumValve";
static const char *LINE_PRESSURE_STR        = "LinePressure";
static const char *CHAMBER_PRESSURE_STR     = "ChamberPressure";
static const char *VACUUM_VALVE_OPEN_STR    = "VacuumValveOpen";
static const char *VACUUM_VALVE_ENABLED_STR = "VacuumValveEnabled";
static const char *VACUUM_PUMP_RUNNING_STR  = "VacuumPumpRunning";
static const char *VACUUM_PUMP_IGNORED_STR  = "VacuumPumpIgnored";

} // namespace adcraydl

#endif // ADCRAYDL_RECORDNAMES_H
