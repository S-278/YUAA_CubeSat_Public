# Yale Undergraduate Aerospace Association's Bouchet Low-Earth Alpha/Beta Space Telescope (BLAST), 2015-present
This repo publishes some parts of the flight software currently under development for Yale's first CubeSat - the Bouchet Low-Earth Alpha/Beta Space Telescope (BLAST), developed entirely by students. I've led the software development since 2021 and the whole project team since 2022.
## Context
Yale Undergraduate Aerospace Association (YUAA) is developing a CubeSat in partenrship with the NASA CubeSat Launch Initiative. Yale will launch a 2U CubeSat to Low Earth Orbit (deployed from the International Space Station) with a twofold mission:
* Gather position- and time-resolved data on the energies and event frequncies of cosmic rays
* Demonstrate a gravity gradient boom (GGB) as a passive attitude stabilization technology

For more details, please visit the team's official website: https://yaleaerospace.org/main/cubesat
## What's in this repo?
This repo is a limited public version of the flight software that will run on BLAST's STM32 on-board computer. We do not publish the full software as it integrates proprietary code from our equipment manufacturer, but this repo contains all the student-written code. 

This repo is maintained for publicity, and is only updated sporadically. Almost all student-written code is still Work-In-Progress, so don't be surprised by rough edges!
### Feature overview
See also the [Software Structure Overview](https://github.com/S-278/YUAA_CubeSat_Public/blob/main/Software%20Structure%20Overview.png).

The BLAST flight software is developed within the [STM32Cube](https://www.st.com/en/ecosystems/stm32cube.html) open ecosystem for an __STM32 microcontroller__ (in between migrating from an STM32F4 to an STM32H7) with a single Cortex core. The __FreeRTOS real-time kernel__ facilitates up to 11 concurrent threads, the most important of which implement:
* __Attitude Determination and Control System (ADCS)__ including a state machine controller thread, a synchronizing measurement acquisition thread, and an active detumbling thread implementing the Bdot magnetorquer algorithm
* State machine __radio controller__ supporting execution of uplinked commands, public beacon, and unsolicited downlink over targeted ground stations tracked using an on-board orbital model
* Dynamic __power distribution algorithm__ monitoring actual solar generation and system consumption and prioritizing subsystems in real time
* Scientific instrument readout routine
* System health monitoring routines (component checks)

Shared libraries support:
* Scientific and diagnostic data generation, storage, and search
* On-board orbital model based on the SGP4 propagator used for determining satellite position from Two-Line Elements
* TRIAD attitude determination algorithm implemented using the CMSIS-DSP fast math library and using NOAA's World Magnetic Model for lookup of the local magnetic field vector
* Thread-safe slave and transaction management
* Drivers for all satellite subsystems
### LICENSE
The following included software is reused by the YUAA. Where no hyperlink is provided, the components are taken from [STM32CubeF4](https://github.com/STMicroelectronics/STM32CubeF4).
| Component | Copyright | License
|-----------|-----------|---------
|CMSIS      |ARM Limited|Apache License 2.0
|CMSIS Device|ARM Limited - STMicroelectronics|Apache License 2.0
|STM32F4 HAL|STMicroelectronics|BSD-3-Clause
|FreeRTOS kernel|Amazon.com, Inc. or its affiliates|MIT
|FatFS|ChaN - STMicroelectronics|BSD-3-Clause
|[SGP4](https://github.com/aholinch/sgp4)|None, authored by aholinch|None (public domain)
|[NOAA WMM](https://www.ncei.noaa.gov/products/world-magnetic-model)|None, authored by U.S. NOAA|None (public domain)

All other components are developed solely by YUAA students and are released into the public domain.
### Contents
See also the [Software Components Overview](https://github.com/S-278/YUAA_CubeSat_Public/blob/main/Software%20Components%20Overview.png). 
#### Appl/
Application-layer code written entirely by YUAA students, including: Attitude Determination and Control System (ADCS), subsystem check routines, dynamic power distribution algorithm, radio control algorithm, telecommand parser.
#### Drivers/
Drivers for satellite subsystems written by YUAA students.
#### Middlewares/
Shared libraries written by YUAA students: SD memory log generation and searching, orbit model, telecommand implementations, TRIAD attitude determination algorithm; as well as headers for third-party components, including FatFS, the FreeRTOS real-time kernel, the NOAA WMM, SGP4 orbit propagator.
## Can I compile/reuse/contribute to/discuss this code?
Since this is not the full flight software, this repo will most certainly not compile. That said, you are free to use this codebase as inspiration, directly reuse the YUAA-written code (though we kindly ask you to attribute in any form you want if possible), or ask us about our work! The YUAA is always on the lookout for partners and mentors!
