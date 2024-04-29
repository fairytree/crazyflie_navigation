# Instructions for Applying Firmware Patches

The repository that these patches are applicable for is:<br>
https://github.com/bitcraze/crazyflie-firmware

However, it is easiest to locate the official realease versions of the firmware from here:<br>
https://github.com/bitcraze/crazyflie-release/releases

The complete instructions for applying the patch are given separately for each firmware version (currently only one firmware version).





## Install Compilation Toolchain

For compilation on Ubuntu 18.04, the toolchain is installed with the following commands:<br>
`sudo apt install gcc-arm-none-eabi`<br>

For compilation on Ubuntu 16.04, the toolchain is installed with the following commands:<br>
`sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa`<br>
`sudo apt-get update`<br>
`sudo apt-get install libnewlib-arm-none-eabi`<br>





## Firware Version 2020-02

The hash of the exact commit to which this patch applies is:<br>
`e27cd170b7eb8be7ed1cbbc9f5622b469d335d97`<br>
As commited on Mon Feb 3 15:04:52 2020 +0100

To apply the patch, use the following sequence of commands:

**Step 1:** Clone the repository and checkout the appropriate commit:<br>
`git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git`<br>
`cd crazyflie-firmware`<br>
`git checkout e27cd170b7eb8be7ed1cbbc9f5622b469d335d97`<br>
Note that it is also possible to checkout this commit using the commit tag, i.e., using `git checkout 2020.02`


**Step 2:** Apply the patch:<br>
`git apply <folder_to_dfall-system_repository>/crazyflie_firmware/crazyflie-firmware-version-2020-20/firmware_modifications_for_version_2020-20.patch`<br>

Now the repository is ready to compile. To do this, first make sure that you have
installed the necessary toolchain as described above ([Install Compilation Toolchain](#install-compilation-toolchain)).

**Step 3:** In the `crazyflie-firmware` folder where you applied the patch, compile the firmware using the following command:<br>
`make PLATFORM=cf2`

If everything is successful, you will see something like this:<br>

<img src="success_building_for_version_2020-02.png" style="width: 400px;"/> <br><br>

This should have created the binary file `cf2.bin`.

**Step 4:** upload the binary to the crazyflie processor by following the instructions given here:<br>
https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#update-fw<br>
Instructions are also given in the wiki of this repository under:
`/wiki/setup.md#firmware`

The pre-compiled binary files are already included in this repository. To update the firmware of just the main processor, select the file:
`<folder_to_dfall-system_repository>/crazyflie_firmware/crazyflie-firmware-version-2020-02/cf2-2020.02_withDfallPatch.bin`<br>

To update the firmware of both the main processor and the nrf processor, select the file:<br>
`<folder_to_dfall-system_repository>/crazyflie_firmware/crazyflie-firmware-version-2020-02/firmware-cf2-2020.02_withDfallPatch.zip`<br>