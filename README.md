# interpolation mode

                                            +---------------+
                                            | interpolation |
                                            |🔲----◯----🔲|
                                            +---------------+
                            +Squares are points and circle are interpolated point+
## Overview

**interpolation mode**  Interpolation is a mathematical technique used to estimate unknown values within the range of a discrete set of known data points.In this mode, interpolation is done between both points. For simplicity, in this code, the points of a sinusoidal motion are interpolated.but you can change the code(while part) as you want🏋️. 

---

## Features

- ⚡ **Suitable for discrete movements**: Every continuous motion is caused by an infinity of discrete points. you can model any continuous motion with this mode.
- 🎯 **Trajectory**: With the interpolation mode, any trajectory can be implemented on the robot. 

---

## Getting Started

To set up the project locally, follow these steps.

### Prerequisites
- **STM32CubeIDE**: Ensure you have [STM32CubeIDE version 1.13.0(recommended)]
- **CANopen stack**: you need this stack for your communication 
i have used this stack in the code, you can also download and including it from [github website](https://github.com/CANopenNode/CANopenNode)

### Run code
1. **open new project**: 
    go to file --> new --> STM32 project
2. **add the stack**: 
    Go to project --> properties --> paths and symbols --> includs --> add --> file systrm
    Then choose CANopenNode and core/CANopenNode_STM32
    Ensure there is CANopenNode in the source location(if this file does not exist in the source location, first apply and close and build the project, then come to the source location again.)
3. **EDS editor**: you need this software to map your PDOs message.follow these steps:
    open EDSEditor software --> create a new OD index --> mapping TPDO --> set COB-ID in communication Specific parameters
3. **run**:
    now, you can use this mode by upload the code to micro.

---

## License
- you can download
 *CANopenNode* document from this link(https://drive.google.com/file/d/1Lg_B5r14P_CGMcFEsO6PIyp6nOz_elqh/view?usp=drive_link)
- you can download
 *EDSEditor* software from this link(https://drive.google.com/file/d/1vxwXvSOTEiWtaTg4LeJ1j8IzjS9KLwnI/view?usp=drive_link)
---

## Contact
- **Email**: bestbs.1382@gmail.com
- **Github**: Amir-SB82