# MS5611
Basic efficient ms5611 driver for Arduino

Oversampling ratio set to most accurate, but slowest

Calculating temperature and pressure with floats is more efficient than with 64 bit integers on the 328p. This saves almost 1kb of flash (probably only if floating calculation are already used) and is around 11% faster
