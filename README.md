# SE09 Autonomous Robotic Car

Pico W firmware for the Robo Pico, written in C on the course micro T-Kernel 3.0 template. The vehicle follows the line, reads Code 39 barcodes in either direction, executes their assigned navigation actions, measures humps, profiles obstacles, and reports mission status over MQTT.

## Build and flash

Use GNU Make, Arm GNU Toolchain, a host C++ compiler, and Pico C SDK 2.3.1. Windows users can run the build in WSL. The SDK needs its TinyUSB, CYW43 and lwIP submodules.

```sh
git clone --branch 2.3.1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
export PICO_SDK_PATH="$PWD/pico-sdk"
# From this repository
cp config/wifi_secrets.example.h config/wifi_secrets.h
# Edit SSID, password and broker IPv4 in config/wifi_secrets.h
make -C build_make -j8
```

The default image is `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`. To flash it, hold BOOTSEL while connecting USB and copy the UF2 to the `RPI-RP2` volume. The USB CDC console prints calibration instructions. For a radio-free build use `make -C build_make WIFI=none NET=none MQTT=0`. Use the USB console because GP0 and GP1 are reserved for the progress version's encoder wiring.

Use a 2.4 GHz network and put the car and broker on the same LAN. The example header contains placeholders. Set the broker's real IPv4 address and, if configured, MQTT username/password locally. The car uses plain MQTT on the course LAN.

## Arm and run

1. Check [TEAM.md](TEAM.md) and use the bench checks in [testing/README.md](testing/README.md). Verify wheel polarity, 3.3 V logic levels, servo direction and the stop button with the wheels raised.
2. Power up on a flat, stationary surface and apply power to the IMU and sensors. Place all three IR sensors over matte white and tap GP21. When the console prompts, cover all three with matte black and tap GP21 again.
3. Place GP26 and GP28 over the edges of the 18 mm track line. Mount the GP27 barcode reader on the **left of the car**, over the offset lane. The firmware decodes either bar order if the print passes under GP27. With a fixed left-side reader, arrange each approach so the printed lane is on the car's left.
4. In the dashboard, assign the course letters A–Z to Left, Right, Straight, U-turn or Unassigned. Wait for confirmation from the car.
5. Tap GP21 to start. GP20 applies the local emergency brake. The MQTT dashboard can request a stop. To resume after an operator stop, release GP20, align the car on the line and tap GP21 again. Reset the board after an impact, encoder, timing or IMU fault.

A decoded letter arms its configured action. The car continues to the next optical junction, advances the measured IR-to-axle distance, then executes the maneuver. Because the actual barcode-to-junction spacing was unavailable, the default requires an optical junction within 145 mm after decode. Measure and set `BARCODE_TO_JUNCTION_MM` to enable the encoder fallback on a course without a detectable broad marking. Set `IR_TO_AXLE_MM` to the measured distance on the car.

## Dashboard

```sh
python -m pip install -r dashboard/requirements.txt
python dashboard/app.py --broker 192.168.1.100
```

Open [the dashboard](http://127.0.0.1:8080). Use the same broker and topic root as the firmware. The dashboard listens on localhost. Set `MQTT_USERNAME` and `MQTT_PASSWORD` in its environment if your broker requires credentials.

The dashboard shows mission state, line, speed, distance, last barcode, peak hump, obstacle and faults. To change A to Left, select **A**, select **Left**, then select **Save mapping**. A confirmation follows the car's published acknowledgment. The broker retains the complete mapping and replays it to the car on reconnect or restart. Enable broker persistence if it must survive a broker restart. A maneuver already selected by the controller keeps its original action.

[MQTT topic and payload reference](dashboard/README.md) · [Data representation and measured sizes](testing/DATA_REPRESENTATION.md) · [Verification and bench procedure](testing/README.md)
