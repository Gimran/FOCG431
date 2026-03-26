# FOCG431
siplefoc g431 driver

## Custom Commands

The firmware provides several custom commands via the serial interface for controlling and configuring the motor and driver.

### `M` - Motor Commands

This command provides access to the standard SimpleFOC motor commands. It acts as a gateway to configure motor parameters like PID values, limits, and motion control settings.

**Usage:** `M<command><value>`

**Example:**
*   `MTP10` - Set the target velocity to 10 rad/s.
*   `MP1.2` - Set the P-gain of the velocity PID controller to 1.2.

Refer to the SimpleFOC documentation for a complete list of available motor commands.

### `V` - Set Velocity

Sets the target velocity for the motor in Revolutions Per Minute (RPM).

**Usage:** `V<rpm>`

**Example:**
*   `V300` - Sets the target velocity to 300 RPM.
*   `V-100` - Sets the target velocity to -100 RPM (counter-clockwise).

### `R` - DRV8323 Registers

This command is used to interact with the registers of the DRV8323 motor driver.

**Usage:**
*   `RA` - Read all registers from the DRV8323 and print their values.
*   `R<addr>` - Read a specific register at the given address (0-6).
*   `1` - Set the current sense amplifier gain to 40 V/V.
*   `2` - Set the current sense amplifier gain to 20 V/V.

**Examples:**
*   `RA` - Dumps all register values.
*   `R0` - Reads the "Fault Status 1" register.
*   `R6` - Reads the "CSA Control" register.

### `D` - Driver Control

This command controls the DRV8323 driver settings.

**Usage:**
*   `DE1` - Enable the motor driver.
*   `DE0` - Disable the motor driver.
*   `DR1` - Clear any existing fault flags on the driver.

**Examples:**
*   `DE1` - Turns on the gate driver.
*   `DR1` - Resets fault status.
