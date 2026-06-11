# GPIO and I2C Map

GPIO, I2C, UART, and other low-level pin assignments.

## I2C device map

| Device             | Address | Bus    | Used by                      | Purpose                    |
| ------------------ | ------- | ------ | ---------------------------- | -------------------------- |
| UPS HAT fuel gauge | `0x36`  | I2C-1  | `savo-core`, `savo-edge`     | Battery/voltage monitoring |
