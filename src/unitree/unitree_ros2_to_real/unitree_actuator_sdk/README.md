## go-m8010-6

support motor: go-m8010-6 motor
not support motor: a1 motor、 b1 motor (check a1b1 branch for support)

```sh
cmake -Bbuild .
cmake --build build
sudo ./motorctrl
```

### Notice

support motor: GO-M8010-6 motor、A1 motor、 B1 motor

gcc >= 5.4.0 (for x86 platform)
gcc >= 7.5.0 (for Arm platform)

run gcc --version command to check your gcc version

### Build

```bash
mkdir build
cd build
cmake ..
make
```

### Run

If the compilation is successful, many C++ example executable files will be generated in the build folder. Then run the examples with 'sudo', for example:

```bash
sudo ./example_a1_motor
```

If you need to run the Python example, please enter the "python" folder. Then run the examples with 'sudo', for example:

```sh
sudo python3 example_a1_motor.py
```
