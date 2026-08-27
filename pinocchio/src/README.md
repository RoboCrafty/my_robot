### Create build dir:
``` cmake -B build ```

### Compile:
``` cmake --build build --parallel ```

### Launch rpi app with:
```sudo chrt -f 90 ./build/parolController /dev/ttyUSB1```