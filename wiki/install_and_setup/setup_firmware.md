# Firmware and channel
The firmware of a crazyflie is updated by the teacher. Additionally, the teacher can set the channel of the crazyflies to prevent interference.<br>

## Firmware
Instructions for flashing the firmware with the Crazyflie client can be found either here:
<br>
https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index#firmware_upgrade
<br>
or here:
<br>
https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#update-fw
<br>
The compiled versions of the Crazyflie firmware that are compatible with the `D-FaLL-System` can be found in the `crazyflie-firmware` folder of the repository.
<br>
<br>
If you have installed the Crazyflie Client properly, as described in the installation section, it can be started via a terminal window by typing `cfclient`.
<br>
<br>
Flashing is done wirelessly over the CrazyRadio link, both for the STM32 main processor chip and the NRF bluetooth chip. You need to specify the correct address in the Crazyflie Client prior to following the steps below, i.e., the 0xE7E7E7E701 type address. For powering the Crazyflie during flashing, it is convenient to connect the Crazyflie to the computer using a USB cable.
<br>
<br>
The steps to flash the crazyflie are:
1. Start the Crazyflie Client from terminal using the command `cfclient`<br>
2. Turn the Crazyflie off<br>
3. Start the Crazyflie in bootloader mode by pressing the ON/OFF button for 3 second. Two blue LEDs will start blinking to indicate the the Crazyflie has powered on into bootloader mode<br>
4. In the Crazyflie Client, select `Connect -> Bootloader` from the top menu. This causes a window titled `Crazyflie Service` to appear<br>
5. In the `Crazyflie Service` window, press the `Initiate bootloader cold boot` button<br>
6. Once the status says `Connected to bootloader`, click the `Browse` button and select the file you wish to flash on the Crazyflie. Typically this file will be something like `cf2.bin` for flashing only the STM32 main processor, or `crazyflie-firmware.zip` for flashing both the NRF and STM32 processors<br>
7. Click the `Program` button. The progress bar will go from 0% to 100% one time for each of the processors to be flashed<br>
8. Wait until the uploading and writing of the new firmware is complete<br>
9. Click the `Restart in firmware mode` button. This causes the Crazyflie to reboot and the new firmware is now running<br>
10. Turn off the Crazyflie<br>
11. Either click the `Cancel bootloading` button or simply close the `Crazyflie Service` window<br><br>

## Channel changing
This is also described on the page linked above. Use the following format: 0/__xx__/2M where __xx__ stands for the radio channel.<br>
The crazyflie has to be restared for the changes to take effect.<br>
![channel_config](https://wiki.bitcraze.io/_media/doc:crazyflie:client:pycfclient:cfclient_cf2_config.png?w=500&tok=74d1d3)
