<img width="1171" height="185" alt="image" src="https://github.com/user-attachments/assets/29ecff0a-2024-46b1-92a9-f47bb43196b0" />

[![Compile Arduino Sketch](https://github.com/Niall-Sharma/WESOCode/actions/workflows/compile.yml/badge.svg)](https://github.com/Niall-Sharma/WESOCode/actions/workflows/compile.yml)
[![.github/workflows/lint.yml](https://github.com/Niall-Sharma/WESOCode/actions/workflows/lint.yml/badge.svg)](https://github.com/Niall-Sharma/WESOCode/actions/workflows/lint.yml)

## About 
This is the codebase for the Iowa State Wind Energy Student Organization. 

## Linting 
To use linting, run `bin/arduino-lint` in the terminal within the code folder for the current WESO year.

## Github Actions
All actions must pass for code to push to main. 

## To do
- [ ] Test Linear Actuator 
- [ ] Get IR Sensor and write code to measure RPM.
- [ ] Mock state machine with transitions
- [ ] QBlade simulations
- [ ] Refactor current code -- upon refactor decide on either camelCase, or snake_case. Likely going with camelCase, as that is what a lot of arduino libraries use.
- [ ] Bug hunt and fix current code


## How to use
- Step 1
- ...
- Change AIR_DENSITY (~line 57) to match the location in which you are testing at. Iowa: 1.197f. Colorado: 1.045f;