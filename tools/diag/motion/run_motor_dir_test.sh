#!/usr/bin/env bash
python3 tools/diag/motion/motor_direction_test.py \
  --chip 4 --no-pullup \
  --driver pca --i2c-bus 1 --pwm-addr 0x40 --pwm-freq 1000 \
  --m1-pwm-ch 8  --m1-in1-ch 9  --m1-in2-ch 10 \
  --m2-pwm-ch 11 --m2-in1-ch 12 --m2-in2-ch 13 \
  --m3-pwm-ch 2  --m3-in1-ch 3  --m3-in2-ch 4  \
  --m4-pwm-ch 5  --m4-in1-ch 6  --m4-in2-ch 7  \
  --left-group 0,2 --right-group 1,3 \
  --duty "${1:-0.60}" --seconds "${2:-1.2}"
