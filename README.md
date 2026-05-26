# ZMK AZ1BALL Driver

This module provides a ZMK input driver for the AZ1BALL trackball hall sensor unit.
The device is read over I2C and reports relative X/Y movement plus a button state.

## Wiring / Firmware Assumptions
The driver expects the AZ1BALL I2C firmware to expose a 5-byte report:

- byte0: left count
- byte1: right count
- byte2: up count
- byte3: down count
- byte4: button bit (bit7)

This matches the `tiny424_trackball.ino` sample firmware in `sample/`.

## Devicetree Example

```dts
&i2c0 {
    az1ball: az1ball@0a {
        compatible = "az1ball,az1ball";
        reg = <0x0A>;
        polling-interval-ms = <5>;
        count-type = <1>;      // 0=normal, 1=AZ
        invert-y;
        scale-x = <1>;
        scale-y = <1>;
    };
};
```

## Kconfig
Enable the driver:

```
CONFIG_ZMK_INPUT_AZ1BALL=y
```
