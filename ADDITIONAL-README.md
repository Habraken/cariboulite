#Install CaribouLite with Raspberry pi OS Trixie (64bit) [WIP]
All of the steps for Bookworm still apply, however there was a policy change with Debian 13 that tightend defaults for
- mlock
- RT scheduling
- unprivileged DMA-style workloads

Therefore some addtional steps on Trixie are required so that the cariboulite_test_app will run without the sudo -E option.

##Step 1
```
bash

sudo mkdir -p /etc/systemd/system.conf.d
```
```
bash

sudo tee /etc/systemd/system.conf.d/99-memlock.conf >/dev/null <<'EOF'
[Manager]
DefaultLimitMEMLOCK=infinity
EOF
```
##Step 2
```
bash

sudo mkdir -p /etc/systemd/user.conf.d
```
```
bash

sudo tee /etc/systemd/user.conf.d/99-memlock.conf >/dev/null <<'EOF'
[Manager]
DefaultLimitMEMLOCK=infinity
EOF
```
##Step 3
```
bash

sudo reboot
```
##Step 4
```
bash

ulimit -l
cat /proc/self/limits | grep -i "Max locked memory"
```

#Install CaribouLite with Raspberry Pi OS Bookworm (Lite) (64bit)

My goal is to install the cariboulite on a Raspberry Pi0_2W and/or a Pi4. Requirements:  
- Connecetd to the Pi over etehernet and ultimately provide power to the Pi with POE  
- Use SDR++ server and SDR++  
- Use gnu-radio

All of the software fits on a sdcard of 16GByte. The CarubouLite uses SoapySDR as an hardware abstraction layer and needs to be installed as well. This can be done manually or automatically with the CaribouLite install script.

This 'recipe' was create and tested on a Pi0 and a Pi4 on May 1st 2025.  
 
###Initial steps
Download & install the latest version of PiOS using the Raspberry Pi Imager. Ensure that you have enabled ssh under services and that you have provided basic information to the general settings of the OS Customisation.  

I make the assumption that all of this is familiar to you and you know how to ssh over the local network to your Pi and know how to work with nano the text editor.

After you have logged on, update the Pi OS:

```
sudo apt upate
sudo apt upgrade
```
###Modifying the boot config.txt file
Modify the `/boot/firmware/config.txt` file to ensure the parameters are set so that the cariboulite can communicate with the CPU. The last two lines disable Bluetooth and WiFi, to minimise RF interference. So the Pi needs to be connected to the local network through an ethernet cable, else you can no longer communicate with the Pi after the next reboot. 

`sudo nano /boot/firmware/config.txt`

Add the lines below just above the line that reads:   
`# Enable audio (loads snd_bcm2835)`

```
dtparam=i2c_arm=off
dtparam=spi=off

# CaribouLite
dtparam=i2c_vc=on
dtoverlay=spi1-3cs

# Disable WiFi and Bluetooth
dtoverlay=disable-wifi
dtoverlay=disable-bt
```

Save the file and reboot

###Install linux kernel header files:

```sudo apt install linux-headers-rpi-v8```
>Note: I think this step is required when you have to rebuild the smi driver and if you want build SDR++.

###Monitoring the Pi's activity
The next steps will often take a while, during which you might want to periodically check that everything is still moving along.
In separate terminals watch what is going on during any of the following steps with `htop` and/or `dmesg`. This is how I discovered that `make` ran out of memory. 

```
htop
sudo dmesg -wH
```
###Install `git` & `cmake`

```
sudo apt install git cmake
```

#SoapySDR (optional)
If you want to have more control over where SoapySDR en SoapyRemote are installed, execute these steps first. If you use these steps, you should say `No` to the `install SoapySDR` question from the CaribouLite install script.

Create a directory where you want to keep your source code. I call mine: `src`. 

```
mkdir src && cd src
``` 

Get the source code:

```
git clone https://github.com/pothosware/SoapySDR.git
git clone https://github.com/pothosware/Soapyremote.git

```
Or:

```
git clone https://github.com/Habraken/SoapySDR.git
git clone https://github.com/Habraken/SoapyRemote.git
```
The following library not available by default and SoapyRemakemote will complain but not fail:

```
sudo apt install libavahi-client-dev
```

In both repositories run the below commands to build and install SoapySDR and SoapyRemote starting with SoapySDR as SoapyRemote depends on it.

```
mkdir build && cd build
cmake .. -DENABLE_PYTHON3=ON
make -j`nproc`
sudo make install -j`nproc`
sudo ldconfig #needed on debian systems
SoapySDRUtil --info
```

#CaribouLite
Create a directory where you want to keep your source code. I call mine: `src`. 

```
mkdir src && cd src
``` 

Get the source code:

```
git clone https://github.com/cariboulabs/cariboulite.git
```

Or if you don't want to patch yourself:

```
git clone https://github.com/Habraken/cariboulite.git
```

To make the nbfm_tx and nbfm_rx examples work you also need to install the dev tools for ALSA:

```
sudo apt install -y libasound2-dev pkg-config
```

Before you can install anything make the following three changes to `~/src/cariboulite/driver/smi_stream_dev.c`.

```
cd ~/src/cariboulite/driver
sudo nano smi_stream_dev.c
```

Add this include statement near the other include statements: `#include <linux/vmalloc.h>`

Then Hit Ctl+W and type `‘create sysfs’` and change the line with the `-` in front to the line with the `+` in front.

```
// Create sysfs entries with "smi-stream-dev"
-smi_stream_class = class_create(THIS_MODULE, DEVICE_NAME);
+smi_stream_class = class_create(DEVICE_NAME);
```

Then Hit Ctl+W and type ‘smi_stream_dev_remove’ and change the line with the - in front to the line with the + in front and remove the return statement with the - in front.

```
*   smi_stream_remove - called when the driver is unloaded.
*
***************************************************************************/
 
-static int smi_stream_dev_remove(struct platform_device *pdev)
+static void smi_stream_dev_remove(struct platform_device *pdev)
 {
     //if (inst->reader_thread != NULL) kthread_stop(inst->reader_thread);
     //inst->reader_thread = NULL;	
     
     device_destroy(smi_stream_class, smi_stream_devid);
     class_destroy(smi_stream_class);
     cdev_del(&smi_stream_cdev);
     unregister_chrdev_region(smi_stream_devid, 1);
 
     dev_info(inst->dev, DRIVER_NAME": smi-stream dev removed");
-    return 0;
 }
```

Instead of making the changes above yourself, you could also 'checkout' one of my recent feature branches, that has these chnages applied already. In addtion you get my 'Monitor Modem Status and NBFM TX and RX options:

```
cd ~/src/cariboulite
git checkout feature/nbfm_rx
```

In the `cariboulite/driver` directory:

```
./install.sh install
```

Then in the cariboulite directory:

```
./install.sh
```
This takes ‘a while’ on a Pi0. At a certain point the script will ask if you want to `install SoapySDR`, if you did not install it before. Say `Yes`. 

After the script has completed, reboot the Pi. 
```
sudo reboot
```

Now we need to check with SMI driver is working correctly.

```
lsmod | grep smi
```
The ouput should look like this:

```
pi@pi0b-bookworm:~ $ lsmod | grep smi
smi_stream_dev         16384  0
bcm2835_smi            20480  1 smi_stream_dev
pi@pi0b-bookworm:~ $ 
```
Also check if the permissions are set correclty:

```
ls -l /dev/smi
```
the ouput shoudl look like this:

```
pi@pi0b-bookworm:~ $ ls -l /dev/smi
crw-rw-rw- 1 root root 239, 0 May  1 13:40 /dev/smi
pi@pi0b-bookworm:~ $ 
```

###Testing SoapySDR

As SoapySDR was install by the install script of cariboulite it should be possible to communicate with the CaribouLite.

```
SoapySDRUtil --find
``` 
The ouput should look like this:

```
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

[INFO] SoapyCaribouliteSession, sessionCount: 0
05-01 13:51:14.339   627   627 I CARIBOU_PROG caribou_prog_configure_from_buffer@caribou_prog.c:260 Sending bitstream of size 32220
05-01 13:51:16.644   627   627 I CARIBOU_PROG caribou_prog_configure_from_buffer@caribou_prog.c:292 FPGA programming - Success!

Printing 'findCariboulite' Request:
Found device 0
  channel = S1G
  device_id = 0
  driver = Cariboulite
  label = CaribouLite S1G[1e3bc7c2]
  name = CaribouLite RPI Hat
  serial = 1e3bc7c2
  uuid = 11884996-87f4-4b07-9961-a38c1b78fa08
  vendor = CaribouLabs LTD
  version = 0x0001

Found device 1
  channel = HiF
  device_id = 1
  driver = Cariboulite
  label = CaribouLite HiF[1e3bc7c3]
  name = CaribouLite RPI Hat
  serial = 1e3bc7c3
  uuid = 11884996-87f4-4b07-9961-a38c1b78fa08
  vendor = CaribouLabs LTD
  version = 0x0001
```

Now it is also possible to interact with the CaribouLite board with the provided `cariboulite_test_app` in the `~/src/cariboulite/build` directory.

```
./cariboulite_test_app
```
At the end of a lot of ouput there should be something like this:

```

	   ____           _ _                 _     _ _         
	  / ___|__ _ _ __(_) |__   ___  _   _| |   (_) |_ ___   
	 | |   / _` | '__| | '_ \ / _ \| | | | |   | | __/ _ \  
	 | |__| (_| | |  | | |_) | (_) | |_| | |___| | ||  __/  
	  \____\__,_|_|  |_|_.__/ \___/ \__,_|_____|_|\__\___|  


 Select a function:
 [0]  Hard reset FPGA
 [1]  Soft reset FPGA
 [2]  Print board info and versions
 [3]  Program FPGA
 [4]  Perform a Self-Test
 [5]  FPGA Digital I/O
 [6]  FPGA RFFE control
 [7]  FPGA SMI fifo status
 [8]  Modem transmit CW signal
 [9]  Modem receive I/Q stream
 [10]  Synthesizer 85-4200 MHz
 [99]  Quit
    Choice:
```
>Note: If you try option `9` and then select option `2` there will errors that look like this:
>
```
FF D0 FF F4 FF 02 C0 D8  FF C0 FF FC FF C2 FF E6  |  ................ 
05-01 14:00:45.038   644   648 E CARIBOULITE Radio cariboulite_radio_read_samples@cariboulite_radio.c:1276 SMI data synchronization failed
FF E0 FF D2 C0 12 C0 1A  C0 DA FF 02 C0 00 C0 EE  |  ................ 
05-01 14:00:45.046   644   648 E CARIBOULITE Radio cariboulite_radio_read_samples@cariboulite_radio.c:1276 SMI data synchronization failed
C0 C0 C0 C4 C0 FA C0 E6  C0 30 C0 22 C0 C0 C0 CC  |  .........0.".... 
05-01 14:00:45.055   644   648 E CARIBOULITE Radio cariboulite_radio_read_samples@cariboulite_radio.c:1276 SMI data synchronization failed
```
>I have just discovered that this behaviour is in fact due to a defective cariboulite board. It appears to function normally until you try to start the smi stream. Wait on feedback fromCaribouLabs. (May 1st 2025)

#feature/nbfm_rx
If you decide to checkout the feature/nbfm_rx the menu after running cariboulite_test_app (I usualy run the app like this: ``` build/cariboulite_test_app 2> debug.log``` from the ```cariboulite``` directory, so that all debug logging goes to ```debug.log```) should look like this:
```
	   ____           _ _                 _     _ _         
	  / ___|__ _ _ __(_) |__   ___  _   _| |   (_) |_ ___   
	 | |   / _` | '__| | '_ \ / _ \| | | | |   | | __/ _ \  
	 | |__| (_| | |  | | |_) | (_) | |_| | |___| | ||  __/  
	  \____\__,_|_|  |_|_.__/ \___/ \__,_|_____|_|\__\___|  


 Select a function:
 [ 0]  Hard reset FPGA
 [ 1]  Soft reset FPGA
 [ 2]  Print board info and versions
 [ 3]  Program FPGA
 [ 4]  Perform a Self-Test
 [ 5]  FPGA Digital I/O
 [ 6]  FPGA RFFE control
 [ 7]  FPGA SMI fifo status
 [ 8]  Modem transmit CW signal
 [ 9]  Modem receive I/Q stream
 [10]  Synthesizer 85-4200 MHz
 [11]  NBFM TX Tone
 [12]  NBFM RX
 [13]  NBFM modem Self-Test
 [14]  Monitor Modem Status
 [99]  Quit
    Choice:   

```
>note: when the ```cariboulite_test_app```is started, it loads the fpga with the original firmware. This firmware has no TX path! To use the TX options you need to upload the updated firmware first, that is part of the feature/nbfm_tx_tone and/or feature/nbfm_rx branches. You can do this by executing option ```[ 0] Hard reset FPGA``` and option ```[ 3] Program FPGA```.

>note: for the options 12 and 14 (R) to work correctly you need to have an audio device attached and configured correctly. It uses ALSA and some of these settings are hard-coded. This is WIP. 14 (T) TX tone should work though, transmitting a 650Hz tone, FM modulated on 430.100 MHz.

This is how the monitor modem status output should look like:
```
CaribouLite Radio    [T]=TX ON/OFF  [R]=RX ON/OFF  [Q]=QUIT  [X]=RES  1766328674
    TX Loopback: off    TX Frequency: 430100000 Hz    TX Power: -3 d     0.00361
Modem Status Registers:
    RF_CFG:0x08  RF_CLKO:0x1A
    IQIFC0:0x33  IQIFC1:0x11  IQIFC2:0x0B
    RF09-RXFDE :0x81  RF24-RXDFE :0x81
    RF09-TXFDE :0x81  RF24-TXDFE :0x81
    RF09-PADFE :0x40  RF24-PADFE :0x40
    RF09-PAC   :0x6B  RF24-PAC   :0x72
    RF09-IRQM  :0x3F  RF24-IRQM  :0x3F
    RF09-IQRS  :0x00  RF24-IRQS  :0x00
    RF09-STATE :0x02  RF24-STATE :0x02
    RF09-TXDACI:0x7E  RF24-TXDACI:0x00
    RF09-TXDACQ:0x3F  RF24-TXDACQ:0x00
FPGA SMI info (0x05):
    RX FIFO EMPTY: 1
    TX FIFO FULL : 0
    SMI CHANNEL  : 1    // 0=RX09 1=RX24
    SMI DIRECTION: 0    // 0=TX   1=RX
    DEBUG = 0, MODE: 'Low Power (0)'
IQ Data Stream:
    TX_I:0x00000FA1  TX_Q:0x00000013
    RX_I:0x00000000  RX_Q:0x00000000
Linux TX FIFO:
    depth: 64/64 (100%), min:64 max:64
    puts:64 gets:0 drops:0 tO_put:0 tO_get:0
    rate: puts 0.0/s, gets 0.0/s  (expect ~100 fps @ 10ms)
Linux RX FIFO:
    depth: 0/128 (0%), min:128 max:0
    puts:0 gets:0 drops:0 tO_put:0 tO_get:0
    rate: puts 0.0/s, gets 0.0/s  (expect ~100 fps @ 10ms)
```
with ```T``` you enable TX (650 Hz tone) with ```R``` you enable RX. 

>Caution! There is no squelch yet, so when no carrier is present, loud noise is emitted from the speaker or headphones! Reduce your volume before you try this!   

#Modifying the firmware (FPGA)

You need some addtional tools such as icestorm, nextpnr and yosys to convert verilog (verilog-2005) in to a file that can be programmed to the Lattice ICE40 FPGA. 

Please install the pre-requisites, icestorm, nextpnr and yosys.
You can follow the instructions here:https://prjicestorm.readthedocs.io/en/latest/overview.html#where-are-the-tools-how-to-install

```
@MISC{IceStorm,
    author = {Claire Wolf and Mathias Lasser},
    title = {Project IceStorm},
    howpublished = "\url{https://prjicestorm.readthedocs.io/}"
}
```
Installing prerequisites (this command is for Ubuntu 14.04):

```
sudo apt-get install build-essential clang bison flex libreadline-dev \
                     gawk tcl-dev libffi-dev git mercurial graphviz   \
                     xdot pkg-config python python3 libftdi-dev \
                     qt5-default python3-dev libboost-all-dev cmake libeigen3-dev
```

Installing prerequisites (this command is for Debian 13):

```
sudo apt install build-essential clang bison flex libreadline-dev \
                     gawk tcl-dev libffi-dev git mercurial graphviz   \
                     xdot pkg-config python python3 libftdi-dev \
                     qtbase5-dev qt5-qmake python3-dev libboost-all-dev \
                     cmake libeigen3-dev
```

On Fedora 24 the following command installs all prerequisites:

```
sudo dnf install make automake gcc gcc-c++ kernel-devel clang bison \
                 flex readline-devel gawk tcl-devel libffi-devel git mercurial \
                 graphviz python-xdot pkgconfig python python3 libftdi-devel \
                 qt5-devel python3-devel boost-devel boost-python3-devel eigen3-devel
```

Note: All tools will be installed relative to ```/usr/local```

Installing the IceStorm Tools (icepack, icebox, iceprog, icetime, chip databases):

```
git clone https://github.com/YosysHQ/icestorm.git icestorm
cd icestorm
make -j$(nproc)
sudo make install
```

Installing Arachne-PNR (place&route tool, predecessor to NextPNR):

```
git clone https://github.com/YosysHQ/arachne-pnr.git arachne-pnr
cd arachne-pnr
make -j$(nproc)
sudo make install
```

Installing NextPNR (place&route tool, Arachne-PNR replacement):

```
git clone --recursive https://github.com/YosysHQ/nextpnr nextpnr
cd nextpnr
cmake . -B build -DARCH=ice40 -DCMAKE_INSTALL_PREFIX=/usr/local && cmake --build build
make -j$(nproc)
cd build
sudo make install
```

Installing Yosys (Verilog synthesis):

```
git clone https://github.com/YosysHQ/yosys.git yosys
cd yosys
make -j$(nproc)
sudo make install
```

Both place and route tools (Arachne-PNR & NextPNR) convert the IceStorm text chip databases into the respective PNR binary chip databases during build. Always rebuild the PNR tools after updating your IceStorm installation.

Once the tools have been installed, you can modify the firmware and load the program to the FPGA. I typically use these two commands:

```
make clean
make build
```
and then I use the ```cariboulite_test_app``` to program the FPGA with new firmware through optinions  ```[ 0] Hard reset FPGA``` and ```[ 3] Program FPGA```.


#SDR++

###Increase the swapfile size (Pi Zero only?)
During the SDR++ build `make` ran out of memory (on the Pi Zero?). Hence I increased the swap file from 512 to 1024.

```
sudo dphys-swapfile swapoff  
sudo nano /etc/dphys-swapfile 
```
Change the line `CONF_SWAPSIZE=512` to `CONF_SWAPSIZE=1024`. Save the file.

```
sudo dphys-swapfile setup
sudo dphys-swapfile swapon  
```

Or:

```
sudo reboot
```

Install dependencies for SDR++:

```
sudo apt install libglfw3 // maybe  not needed.
sudo apt install libglfw3-dev
sudo apt install libfftw3-dev
sudo apt install libvolk-dev // libvolk2-dev for bookworm?
sudo apt install libvolk-bin
sudo apt install libzstd-dev
sudo apt install librtaudio-dev
```

or in one go:

```
sudo apt install libglfw3-dev libfftw3-dev libvolk-dev libvolk-bin libzstd-dev librtaudio-dev

```
### Install via the nightly build package

The easiest method is to download and install the nightly builds of SDR++. First download the package:

```
curl https://github.com/AlexandreRouma/SDRPlusPlus/releases/download/nightly/sdrpp_debian_bookworm_aarch64.deb
```

Then run the following:

```
sudo apt install ~/Downloads/sdrpp_debian_bookworm_aarch64.db
```

### Build from source (only option if you want Soapy and/or SDRplay)
Prior to building from source, I believe it is nessecary to atleast once run ```SoapySDRUtil --find``` else the build process for SDR++ doesn't find the required libraries and will skill the inclusion of Soapy_Source.
If you are interested in using the SDRplay devices, you need to ensure the prorietary API library is installed, before proceding with the build

Get the source code:

```
git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
```

```
git clone https://github.com/Habraken/SDRPlusPlus.git
```
In the `~/src/SDRPlusPLus` directory:

```
mkdir build && cd build
```

```
cmake .. -DOPT_BUILD_SOAPY_SOURCE=ON -DOPT_BUILD_HERMES_SOURCE=ON -DOPT_BUILD_SDRPLAY_SOURCE=OFF -DOPT_BUILD_PLUTOSDR_SOURCE=OFF -DOPT_BUILD_AIRSPY_SOURCE=OFF -DOPT_BUILD_AIRSPYHF_SOURCE=OFF -DOPT_BUILD_HACKRF_SOURCE=OFF -DOPT_BUILD_RTL_SDR_SOURCE=OFF
```

```
make -j2
```

```
cd ..
```
```
sh ./create_root.sh
```
```
cd build
```
```
sudo make install
```
```
sudo ldconfig
```


>Note: There is no need to start the SoapySDR service! Instead we will use the SDR++ Server. For gnu-radio the Soapy server is needed though...

###Desktop version SDR++
Before the `soapy_source` is available it needs to be added, in the ‘Module Manager’ of SDR++ if you are running the desktop app on the Pi4. In the 'module manager' select ‘soapy_source’, add a name to the left e.g. ‘Soapy Source’ and click on the tiny plus sign on the right…

Or...

###Headless version SDR++
If you are working from the cli you need to first run SDR++ so that the default config files are created.

```
sdrpp --server
```
Hit ^c to stop the server again.

Edit the `~/.config/sdrpp/config.json` and add the soapy_source module:

```
        ,
        "Soapy Source": {
            "enabled": true,
            "module": "soapy_source"
        }
```

>Note: mind the comma above the double quote!

Save the file and start the SDR++ server.

```
sdrpp --server
```
When the server starts watch the cli output and confirm that the 'Soapy Source' is loaded.

On your remote computer you should now start SDR++ and select SDR++ Server as Source. Provide the correct IP address of the Pi running the SDR++ server. Press `Connect`. At the `Source [REMOTE]` use the drop down menu to select `SoapySDR`. You ay have to press the `Refresh` button first. In the dropdown menu below that make sure you select the device appropriate for your selected frequency.

>Observations: The Pi4 can easily handle 4 MSPS but somehow when the `bandwidth` setting is set to `auto` there is no data coming from the radio. When the bandwidth setting is changed to 2 MHz data is streaming again. However the display bandwidth is 4 MHz!
>The analog bandwidth is only 2.5 MHz according to the datasheet. With `SoapySDRUtil --probe` you'll find a max. sample rate = 4 MSPS and the max. filter bandwidth = 2 MHz. 


#Trouble shooting faulty board

###System

```
uname -a
```
```
Linux pi0c-bookworm 6.12.20+rpt-rpi-v8 #1 SMP PREEMPT Debian 1:6.12.20-1+rpt1~bpo12+1 (2025-03-19) aarch64 GNU/Linux
```

###SoapySDRUtil

```
SoapySDRUtil --find
```

```
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

[INFO] SoapyCaribouliteSession, sessionCount: 0
05-02 07:55:11.142   644   644 I FPGA caribou_fpga_program_to_fpga@caribou_fpga.c:210 FPGA already operational - not programming (use 'force_prog=true' to force update)
Printing 'findCariboulite' Request:
Found device 0
  channel = S1G
  device_id = 0
  driver = Cariboulite
  label = CaribouLite S1G[1e3bc7c2]
  name = CaribouLite RPI Hat
  serial = 1e3bc7c2
  uuid = 11884996-87f4-4b07-9961-a38c1b78fa08
  vendor = CaribouLabs LTD
  version = 0x0001

Found device 1
  channel = HiF
  device_id = 1
  driver = Cariboulite
  label = CaribouLite HiF[1e3bc7c3]
  name = CaribouLite RPI Hat
  serial = 1e3bc7c3
  uuid = 11884996-87f4-4b07-9961-a38c1b78fa08
  vendor = CaribouLabs LTD
  version = 0x0001
```
      
```
SoapySDRUtil --probe
```

```
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

Probe device 
[INFO] SoapyCaribouliteSession, sessionCount: 0
05-02 08:03:21.048   712   712 I FPGA caribou_fpga_program_to_fpga@caribou_fpga.c:210 FPGA already operational - not programming (use 'force_prog=true' to force update)
Printing 'findCariboulite' Request:
[INFO] Initializing DeviceID: 0, Label: CaribouLite S1G[1e3bc7c2], ChannelType: S1G
[INFO] Creating SampleQueue MTU: 131072 I/Q samples (524288 bytes)

----------------------------------------------------
-- Device identification
----------------------------------------------------
  driver=Cariboulite
  hardware=Cariboulite Rev2.8
  device_id=0
  fpga_revision=1
  hardware_revision=0x0001
  product_name=CaribouLite RPI Hat
  serial_number=253617121
  vendor_name=CaribouLabs LTD

----------------------------------------------------
-- Peripheral summary
----------------------------------------------------
  Channels: 1 Rx, 1 Tx
  Timestamps: NO

----------------------------------------------------
-- RX Channel 0
----------------------------------------------------
  Full-duplex: NO
  Supports AGC: YES
  Stream formats: CS16, CS8, CF32, CF64
  Native format: CS16 [full-scale=4095]
  Antennas: TX/RX Sub1GHz
  Full gain range: [0, 69] dB
    Modem AGC gain range: [0, 69] dB
  Full freq range: [389.5, 510], [779, 1020] MHz
    RF freq range: [389.5, 510], [779, 1020] MHz
  Sample rates: 4, 2, 1.33333, 1, 0.8, 0.666667, 0.5, 0.4 MSps
  Filter bandwidths: 0.02, 0.05, 0.1, 0.16, 0.2, 0.8, 1, 1.25, 1.6, 2 MHz
  Sensors: RSSI, ENERGY, PLL_LOCK_MODEM
     * RSSI (RX RSSI):[-127, 4] 0.000000
        Modem level RSSI measurment
     * ENERGY (RX ENERGY):[-127, 4] 0.000000
        Modem level ENERGY (EDC) measurment
     * PLL_LOCK_MODEM (PLL Lock Modem): 1.000000
        Modem PLL locking indication

----------------------------------------------------
-- TX Channel 0
----------------------------------------------------
  Full-duplex: NO
  Supports AGC: NO
  Stream formats: CS16, CS8, CF32, CF64
  Native format: CS16 [full-scale=4095]
  Antennas: TX/RX Sub1GHz
  Full gain range: [0, 31] dB
    Modem PA gain range: [0, 31] dB
  Full freq range: [389.5, 510], [779, 1020] MHz
    RF freq range: [389.5, 510], [779, 1020] MHz
  Sample rates: 4, 2, 1.33333, 1, 0.8, 0.666667, 0.5, 0.4 MSps
  Filter bandwidths: 0.08, 0.1, 0.125, 0.16, 0.2, 0.4, 0.5, 0.625, 0.8, 1 MHz
  Sensors: PLL_LOCK_MODEM
     * PLL_LOCK_MODEM (PLL Lock Modem): 1.000000
        Modem PLL locking indication
```

###Self test

Error messages during the selftest using the `cariboulite_test_app`:

```
05-02 07:38:08.605   622   622 D CARIBOULITE Setup
cariboulite_self_test@cariboulite_setup.c:480 Testing modem communication and versions
05-02 07:38:08.606   622   622 W AT86RF215_Main
at86rf215_print_version@at86rf215.c:294 MODEM Version: not AT86RF215 IQ capable modem (product number: 0x0d, versions 03)
05-02 07:38:08.606   622   622 E CARIBOULITE Setup
cariboulite_self_test@cariboulite_setup.c:486 The assembled modem is not AT86RF215 / IQ variant (product number: 0x0d)
05-02 07:38:08.606   622   622 D CARIBOULITE Setup
cariboulite_self_test@cariboulite_setup.c:495 Testing mixer communication and versions
05-02 07:38:08.608   622   622 E CARIBOULITE Setup
cariboulite_self_test@cariboulite_setup.c:513 Self-test process finished with errors
```

I have also tried all the other options such as hard reset of the fpga, soft reset of the fpga, reprogramming of the fpga. During the receive test there are always smi sync errors.

#ALSA

ALSA is complicated. To have some flexibility for testing the nbfm modulator and demodulator it can be helpfull to not have to depend on a physical audio device, such as a sound card.This can be done with a ALSA utility: snd_aloop. What I describe here is based on this article:

based on: https://linuxvox.com/blog/linux-without-hardware-soundcard-capture-audio-playback-and-record-it-to-file/

###Prerequisits:

Ensure 
```alsa-utils```  ```ffmpeg``` ```sox``` 
are installed.

note :both alsa-utils and ffmpeg, are already part of the Raspberry OS usualy.

###Making the Loopback Module Persistent

The modprobe command loads the module temporarily (it will unload on reboot). To make it persistent:

Create a configuration file in /etc/modules-load.d/ to load the module at boot:

sudo nano /etc/modules-load.d/alsa-loopback.conf  
Add the following line and save the file:

```snd-aloop```  
Reboot to test persistence (optional, but recommended):

sudo reboot  
After reboot, re-run lsmod | grep snd_aloop to confirm the module is loaded.

### Testing TX

To test if the ALSA Loopback device works you can start the cariboulite_test_app, enable tx and in a separate terminal run the following command: ```speaker-test -D plughw:Loopback,0,1 -c 1 -t sine -f 440 -r 48000```. You should now here a 440 Hz tone on a narrow band fm receiver tuned to the correct frequency.

If you want to route the microphone audio of your USB audio device to the caribloulite instead of a test tone, you can use this command: ```arecord -D plughw:4,0 -f S16_LE -c 1 -r 48000 | aplay  -D plughw:Loopback,0,1 -f S16_LE -c 1 -r 48000```.

For some obscure reason the alsaloop command doesn't work for this route.

### Testing RX

Similary you can test rx with the following command to 'route' the audio signal from the Loopback device to the plugged in sound card: ``` arecord -D plughw:Loopback,1,0 -f S16_LE -c 1 -r 48000 | aplay  -D plughw:4,0 -f S16_LE -c 1 -r 48000```.

If faced with ```overruns!!!``` one could also use:

```alsaloop -C plughw:Loopback,1,0 -P plughw:4,0 -r 48000 -c 1 -f S16_LE -t 50000``` 

or 

```arecord -D plughw:Loopback,1,0 -f S16_LE -c 1 -r 48000 --buffer-time=100000 --period-time=25000 | aplay  -D plughw:4,0 -f S16_LE -c 1 -r 48000 --buffer-time=100000 --period-time=25000```



# GNU-RADIO (WIP)
```
sudp apt install gnuradio
```
