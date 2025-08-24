# JIRA Project Information - ESP32-S3 Matrix Nearby Radar

## Project Summary for JIRA
**Project Name**: ESP32-S3 Matrix Nearby Radar  
**Course**: COMP4500  
**Project Type**: Interactive Art Installation  
**Status**: Development Complete, Ready for Testing  

## Epic Description
Create an interactive art installation that visualizes nearby devices using ESP32-S3 microcontrollers and LED matrices. The system creates a "radar-like" display showing the signal strength of surrounding devices in real-time.

## User Stories

### Story 1: Device Discovery
**As an** art installation viewer  
**I want** the system to automatically detect nearby devices  
**So that** I can see a real-time visualization of the digital environment around me  

**Acceptance Criteria:**
- [x] System broadcasts heartbeat packets every 1000ms
- [x] Devices automatically discover each other via ESP-NOW
- [x] Support for up to 8 simultaneous devices
- [x] Automatic removal of offline devices after 10 seconds

### Story 2: Signal Strength Visualization
**As an** art installation viewer  
**I want** to see signal strength represented visually  
**So that** I can understand the relative proximity and connection quality of devices  

**Acceptance Criteria:**
- [x] RSSI values mapped to bar chart heights (0-8 levels)
- [x] Color gradient from red (weak signal) to green (strong signal)
- [x] Real-time updates with smoothing algorithm
- [x] Clear visual feedback when no devices are detected

### Story 3: System Reliability
**As an** installation maintainer  
**I want** the system to be stable and self-recovering  
**So that** the art piece can run continuously without intervention  

**Acceptance Criteria:**
- [x] Robust error handling for network initialization
- [x] Automatic peer cleanup to prevent memory leaks
- [x] Visual startup self-test sequence
- [x] Configurable brightness and display parameters

## Technical Tasks

### Task 1: Hardware Configuration
- [x] Configure ESP32-S3 with NeoPixel matrix on GPIO 14/48
- [x] Set up proper color order (RGB) for display
- [x] Implement matrix orientation (bottom-left origin)
- [x] Add startup self-test sequence

### Task 2: Communication Protocol
- [x] Implement ESP-NOW peer-to-peer communication
- [x] Design packet structure with magic bytes and sequence numbers
- [x] Add RSSI smoothing algorithm (alpha = 0.7)
- [x] Implement heartbeat and timeout mechanisms

### Task 3: Visualization Engine
- [x] Create bar chart rendering system
- [x] Implement color mapping for signal strength
- [x] Add sorting by signal strength (strongest first)
- [x] Create fallback animation for no-peer state

### Task 4: Code Quality
- [x] Translate all comments from Chinese to English
- [x] Add comprehensive inline documentation
- [x] Implement proper error handling
- [x] Optimize performance with 80ms update cycles

## Testing Checklist
- [ ] Single device startup test
- [ ] Multi-device discovery test
- [ ] Signal strength accuracy verification
- [ ] Long-term stability test (24+ hours)
- [ ] Edge case handling (device join/leave scenarios)

## Deployment Notes
1. Flash firmware to all ESP32-S3 devices
2. Ensure all devices use the same WiFi channel (Channel 1)
3. Place devices in proximity for optimal visualization
4. Monitor for proper LED matrix orientation and brightness
5. Verify color accuracy matches signal strength expectations

## Risk Assessment
- **Low Risk**: Hardware compatibility issues (GPIO pin alternatives available)
- **Medium Risk**: Network interference in crowded WiFi environments
- **Low Risk**: Power consumption for extended installations

## Success Metrics
- All devices successfully discover each other within 5 seconds
- Accurate signal strength representation with smooth updates
- Zero crashes during 24-hour continuous operation
- Visually appealing and responsive art installation experience
