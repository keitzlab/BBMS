# Bunsen Burner Monitoring System

A comprehensive IoT-based monitoring system for laboratory Bunsen burners, developed at The University of Texas at Austin by Yang Gao from the McKetta Department of Chemical Engineering.

## Overview

This system provides real-time monitoring and safety management for laboratory Bunsen burners through a network of connected sensor nodes, a central monitoring hub, and desktop display units. The system can monitor gas valve positions, environmental conditions, and provide automatic safety shutoffs when equipped with motorized valves.

## Features

- **Real-time Monitoring**: Continuous monitoring of burner valve positions and environmental conditions
- **Distributed Architecture**: Multiple sensor nodes communicating with a central hub via UDP
- **Visual Display**: E-ink display nodes showing system status at desktop locations
- **Safety Features**: Optional automatic gas shutoff with motorized valves
- **Temperature Monitoring**: Environmental temperature and humidity sensing
- **Network Time Sync**: Accurate timestamping using NTP
- **Web Interface**: Monitor hub provides web-based status monitoring
- **Auto Communication Recovery in DHCP**: EEPROM-based IP storage

## System Architecture

### Components

1. **Sensor Nodes** (`SensorNode/`)
   - Monitor individual burner valve positions
   - Audio and visual alarm with countdown timer
   - Optional motorized valve control for automatic shutoff
   - Temperature and humidity sensing (AHT20 sensor)
   - OLED display for local status
   - WiFi connectivity for data transmission

2. **Monitor Hub** (`MonitorHub/`)
   - Central server collecting data from all sensor nodes
   - Controls additional dynamic signs to promote awareness
   - Web interface for system monitoring
   - UDP communication with display nodes

3. **Display Nodes** (`DisplayNode/`)
   - Desktop e-ink displays
   - Real-time status visualization
   - Additional visual alarm for sensor timeout

## Hardware Requirements

Please refer to the BOM files.

## Project Structure

```
├── ArduinoCodes/           # Arduino source code
│   ├── SensorNode/         # Sensor node firmware
│   ├── DisplayNode/        # Display node firmware
│   └── MonitorHub/         # Monitor hub firmware
├── BOM/                    # Bill of Materials (Excel files)
├── AssemblyInstructionDrawing/  # Assembly guides and PCB files
├── Centroid/               # PCB centroid files
├── garber/                 # Gerber files for PCB manufacturing
└── Documentation/          # Additional documentation files
```

## Installation

Please refer to SI_BunsenBurnerMonitoringSystem.docx and videos.

## Configuration

Please refer to SI_BunsenBurnerMonitoringSystem.docx and videos.

## Usage

Please refer to Bunsen_Burner_Monitoring_System_QuickStart Guide_V3.docx and videos.


## Bill of Materials

Detailed component lists are available in the `BOM/` directory:
- `BunsenBurner_DisplayNode.xlsx` - Display node components
- `BunsenBurner_Sensor_Monitor.xlsx` - Sensor and monitor hub components
- `SystemBOM.xlsx` - Complete system BOM

## PCB Files

Custom PCB designs are included:
- **Gerber Files**: `garber/` directory contains manufacturing files
- **Assembly Drawings**: `AssemblyInstructionDrawing/` contains assembly guides
- **Centroid Files**: `Centroid/` directory for pick-and-place machines

## Version History

- **Version 2.6** (October 30, 2025) - Current version
- Enhanced stability and performance
- Updated documentation
- Improved network reliability

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

This project was developed at The University of Texas at Austin. For contributions or questions, please contact the McKetta Department of Chemical Engineering.

## Acknowledgments

- **Developer**: Yang Gao, McKetta Department of Chemical Engineering
- **Institution**: The University of Texas at Austin
- **Date**: 2025

## Troubleshooting

Please refer to Bunsen_Burner_Monitoring_System_QuickStart Guide_V3.docx. 

## Support

For technical support or questions about this system, please refer to the documentation files included in this repository or contact the development team at UT Austin.
