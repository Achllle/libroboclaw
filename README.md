# libroboclaw

Interface for roboclaw motor drivers. Cleaner rewrite of the serial interface with a focus on easy access to common methods
with solid documentation.

based on [this repo](https://github.com/csvance/roboclaw).


## Trying out the library

```
git clone https://github.com/Achllle/libroboclaw.git
mkdir build && cd build
cmake ..
make
```

Test the library:
```
./connect /dev/serial0 115200 128
```

You should see
```
attempting to establish connection to port /dev/serial0 with baud rate 115200 at address 128...
encoders: M1: 0, M2: 0
```

Your encoder values might be different.
