# YAGSL Swerve Configuration

Place your YAGSL-generated JSON files here.

Expected structure:
```
swerve/
  swerve.json                   <- main drive config (IMU, max speed, etc.)
  modules/
    frontleft.json
    frontright.json
    backleft.json
    backright.json
    physicalproperties.json     <- wheel diameter, gear ratios
    pidfproperties.json         <- drive & angle PID gains
```

Generate these using the YAGSL configurator at:
https://yagsl.com/configurator  (or the offline tool)

Then copy the output into this directory before deploying.
