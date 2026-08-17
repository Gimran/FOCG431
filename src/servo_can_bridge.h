#pragma once
#include <SimpleFOC.h>
#include "comms/can/CANCommander.h"

void initServoCANBridge(CANCommander& commander, uint32_t default_can_id);