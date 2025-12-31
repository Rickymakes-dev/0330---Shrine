# Shrine Pusher Control System

Arduino-based CNC shield pusher control with two stepper motor axes for automated extend/retract cycles.

## Hardware Requirements

- Arduino Uno/Nano
- CNC Shield V3
- 2x Stepper motors with drivers (A4988/DRV8825)
- 2x Limit switches (normally open)
- 1x Trigger button
- 1x Emergency stop button (optional)
- 12V/24V power supply

## Wiring

### CNC Shield Connections

| Component | CNC Shield Pin | Arduino Pin |
|-----------|----------------|-------------|
| **X Axis (Horizontal)** | X port | Step: 2, Dir: 5, Enable: 8 |
| **Y Axis (Elevation)** | Y port | Step: 3, Dir: 6, Enable: 8 |
| **X Limit Switch** | -X | Pin 9 (with internal pullup) |
| **Y Limit Switch** | -Y | Pin 10 (with internal pullup) |
| **Trigger Button** | CoolEn/SpnEn | A3 (with internal pullup) |
| **E-Stop** | Abort | A0 (with internal pullup) |

### Switch Wiring
All switches use internal pullups - wire between the pin and GND:
- **Limit switches**: Normally Open (NO), closes when triggered
- **Trigger button**: Normally Open, closes when pressed
- **E-Stop**: Normally Open, closes when pressed

## Configuration

Edit these values in `shrine_pusher.ino`:

### Operation Modes
```cpp
const bool TEST_MODE = false;      // true = Serial test mode, false = Normal operation
const bool AUTO_HOMING = true;     // true = Home on startup, false = Wait for trigger
```

### Travel Distance
```cpp
const long X_MAX_TRAVEL_MM = 81;   // Horizontal travel distance (mm)
const long Y_MAX_TRAVEL_MM = 43;   // Elevation travel distance (mm)
```

### Motor Settings
```cpp
const long STEPS_PER_MM = 80;      // Steps per mm (depends on motor/driver/microstepping)
```

### Speed Settings (delay in microseconds between steps)
```cpp
const unsigned int HOMING_SPEED_DELAY = 2000;   // ~6mm/s - Slow for safe homing
const unsigned int NORMAL_SPEED_DELAY = 1500;   // ~8mm/s - Normal operation
const unsigned int TEST_SPEED_DELAY = 1500;     // ~8mm/s - Test mode
const unsigned int START_SPEED_DELAY = 3000;    // Starting delay for acceleration
const unsigned int ACCEL_STEPS = 200;           // Steps to accelerate/decelerate
```

> **Note**: If motors whine but don't move, increase the delay values.  
> If motors skip steps (inconsistent distance), increase delays and check driver current.

### Direction Settings
Swap HIGH/LOW if a motor moves in the wrong direction:
```cpp
#define DIR_X_IN HIGH      // X toward limit (retract)
#define DIR_X_OUT LOW      // X away from limit (extend)
#define DIR_Y_DOWN HIGH    // Y toward limit (down)
#define DIR_Y_UP LOW       // Y away from limit (up)
```

### Timing
```cpp
const unsigned long MOTOR_IDLE_TIMEOUT = 2000;  // Disable motors after 2s idle (reduces heat)
```

## Operation Sequence

### Normal Mode (`TEST_MODE = false`)

1. **Startup**: System homes both axes (if `AUTO_HOMING = true`)
2. **Ready**: Wait for trigger button press
3. **Extend**: 
   - Y axis moves up first (elevation)
   - X axis moves out (horizontal)
4. **Wait**: System waits for another button press
5. **Retract**:
   - X axis retracts first (horizontal) - moves to limit
   - Y axis lowers (elevation) - moves to limit
6. **Ready**: Returns to step 2

### Homing Order (Safety)
- **Homing**: X (horizontal) first, then Y (elevation)
- **Extend**: Y (elevation) up first, then X (horizontal) out
- **Retract**: X (horizontal) in first, then Y (elevation) down

If a limit switch is already pressed during homing, that axis is considered homed immediately.

## Test Mode Commands

Set `TEST_MODE = true` and use Serial Monitor (115200 baud):

| Command | Action |
|---------|--------|
| `1` | X out 10mm |
| `2` | X in 10mm |
| `3` | Y up 10mm |
| `4` | Y down 10mm |
| `5` | X out 50mm |
| `6` | X in 50mm |
| `7` | Y up 50mm |
| `8` | Y down 50mm |
| `h` | Home all axes |
| `s` | Show switch status |
| `w` | Watch inputs continuously |
| `m` | Measure travel distance |
| `e` | Run extend sequence |
| `r` | Run retract sequence |
| `?` | Show help |

### Measuring Travel Distance
1. Manually position axis away from limit
2. Press `m`, then `x` or `y`
3. Motor moves until limit switch triggers
4. Note the reported distance and update `X_MAX_TRAVEL_MM` or `Y_MAX_TRAVEL_MM`

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Motor whines but doesn't move | Increase delay values (try 2000+) |
| Motor skips steps / inconsistent distance | Increase delays, add acceleration ramp, check driver current |
| Motor/driver overheating | Motors auto-disable after 2s idle; reduce driver current via potentiometer |
| Button not detected | Check wiring (should connect pin to GND when pressed) |
| Wrong direction | Swap HIGH/LOW in direction definitions |
| E-Stop triggers unexpectedly | Check E-Stop wiring or set pin to unused if not using E-Stop |

## Serial Output

At 115200 baud, the system outputs:
- Startup messages
- Homing progress
- Movement status with dot progress indicators
- Button state (in normal mode, every second)
- Error messages

## Safety Features

- **E-Stop**: Immediately disables motors and halts system
- **Motor idle timeout**: Disables motors after 2 seconds to reduce heat
- **Acceleration ramp**: Prevents missed steps by gradually ramping speed
- **Limit switch protection**: Prevents movement past limits during retract
