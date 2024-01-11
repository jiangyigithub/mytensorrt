######################################
### Bosch Radar Gen5 Sensing State ###
######################################

# Flags for measurement_state
uint8 MEASUREMENT_STATE_INVALID = 0
uint8 MEASUREMENT_STATE_VALID = 1

uint8 OPERATION_MODE_MEASUREMENT = 20

uint8 operation_mode
uint8 measurement_state

# distance range scaling factor for FoV from modulation
#            (distance degradation due to thermal degradation, noise increase)
#            1: full range
#            0.93: range reduction if 75% of ramps are used
#            0.84: range reduction if 50% of ramps are used
float32 thermal_degredation_factor

Blindness blindness
FieldOfView field_of_view
InterferenceStatus interference_status
Misalignment misalignment
ModulationPerformance modulation_performance 
