## About
The following is a Linux user-space application for RC car controls. This includes anything from GPS navigation to machine vision for the RC autonomous car.

## How to run
1. Clone the repository:
   ```bash
   git clone
2. Navigate to the project directory:
   ```bash
   cd rc_car_control
   ```
3. Build the project:
   ```bash
   make
   ```
4. Run the application (qemu):
   ```bash
   qemu-aarch64 -L /opt/poky/4.0.26/sysroots/armv8a-poky-linux ./build/src/rc-car-nav
   ```