# Use Crazyradio with WSL2

> Credit to Isaac Lin

Attach the Crazyradio USB dongle to WSL2, then use the Crazyflie client (`cfclient`) inside WSL2 to check that it can detect the Crazyflie quadcopter over radio.

Install the Crazyflie client in your WSL2 distribution before starting. [Crazyflie client installation instructions](https://github.com/bitcraze/crazyflie-clients-python#installation)

1. On Windows, plug in the Crazyradio, launch Zadig, select the Crazyradio, and install the `libusb-win32` driver. [Download Zadig](https://zadig.akeo.ie/)
2. Install `usbipd-win` on Windows if needed. [usbipd-win v4.2.0 installer](https://github.com/dorssel/usbipd-win/releases/tag/v4.2.0)
3. In a Windows command prompt, find the Crazyradio in the output of `usbipd list` and copy its bus ID.
4. In a Windows command prompt running as administrator, share the Crazyradio. Replace `<BUSID>` with the value from the previous step.

   ```powershell
   usbipd bind --busid <BUSID>
   ```

5. Run `usbipd list` again and verify that the Crazyradio is shared. Keep a WSL2 shell open and run the attach command from the Windows command prompt.

   ```powershell
   usbipd attach --wsl --busid <BUSID>
   ```

6. Run `usbipd list` in Windows and verify that the Crazyradio is attached. With the Crazyflie powered on, run `cfclient` in the WSL2 shell and check whether the client detects the quadcopter.

Repeat steps 5 and 6 after unplugging and reconnecting the Crazyradio. Run `usbipd list` again to check its current bus ID before attaching it.
