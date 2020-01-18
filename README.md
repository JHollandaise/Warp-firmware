# Simple weigh scale. Based of the firmware for the [Warp](https://github.com/physical-computation/Warp-hardware) family of hardware platforms
This is the firmware for a simple weigh scale/load cell measurement system with the ability to connect to a 96x64 SSD1331 OLED or to the terminal via JLink (see below). It is based off the [Warp hardware](https://github.com/physical-computation/Warp-hardware) and it is publicly available and unpublished derivatives. This firmware also runs on the Freescale/NXP FRDM KL03 evaluation board which we use for teaching at the University of Cambridge. When running on platforms other than Warp, only the sensors available in the corresponding hardware platform are accessible.

**Prerequisites:** You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., via `apt-get` on Linux or via [MacPorts](https://www.macports.org) on macOS). You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

## 1.  Compiling the Firmware

### 1.1 Enviroment variables
Make sure the environment variable `ARMGCC_DIR` is set correctly (you can check whether this is set correctly, e.g., via `echo $ARMGCC_DIR`; if this is unfamiliar, see [here](http://homepages.uc.edu/~thomam/Intro_Unix_Text/Env_Vars.html) or [here](https://www2.cs.duke.edu/csl/docs/csh.html)). If your `arm-none-eabi-gcc` is in `/usr/local/bin/arm-none-eabi-gcc`, then you want to set  `ARMGCC_DIR` to `/usr/local`. If your shell is `tcsh`:

	setenv ARMGCC_DIR <full path to the directory containing bin/arm-none-eabi-gcc>

Alternatively, if your shell is `bash`

	export ARMGCC_DIR=<full path to the directory containing bin/arm-none-eabi-gcc>

(You can check what your shell is, e.g., via `echo $SHELL`.) Second, edit the jlink command file, `tools/scripts/jlink.commands` to include the correct path.

### 1.2 User Interface Compilation Setting
Depending on whether you wish to compile for the terminal interface or the OLED interface, two files must be altered to set the correct compilation:

In `build/ksdk1.1/build.sh` uncomment line 14:

	cp ../../src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot-segger-connect.c		work/demos/Warp/src/

for the terminal interface or uncomment line 16-17:

	cp ../../src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot-oled-connect.c		work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/menustates.h			work/demos/Warp/src/

for the OLED interface.

In `src/boot/CMakeLists.txt` uncomment line 92:

	"${ProjDirPath}/../../src/warp-kl03-ksdk1.1-boot-segger-connect.c"

for the terminal interface.

or uncomment line 94:

	"${ProjDirPath}/../../src/warp-kl03-ksdk1.1-boot-oled-connect.c"

for the OLED interface

### 1.3 Build Firmware
You should be able to build the Warp firmware by

	cd build/ksdk1.1/
	./build.sh

This copies the files from `Warp/src/boot/ksdk1.1.0/` into the KSDK tree, builds, and converts the binary to SREC. See 	`Warp/src/boot/ksdk1.1.0/README.md` for more. _When editing source, edit the files in `Warp/src/boot/ksdk1.1.0/`, not the files in the build location, since the latter are overwritten during each build._

### 1.4 (optional) Connect to Terminal Interface
You will need two terminal windows. In one shell window, run the firmware downloader:

	JLinkExe -device MKL03Z32XXX4 -if SWD -speed 100000 -CommanderScript ../../tools/scripts/jlink.commands

In the other shell window, launch the JLink RTT client<sup>&nbsp;<a href="#Notes">See note 1 below</a></sup>:

	JLinkRTTClient

## 2. Using the Warp firmware on the FRDM KL03
The SEGGER firmware allows you to use SEGGER’s JLink software to load your own firmware to the board, even without using their specialized JLink programming cables. You can find the SEGGER firmware at the SEGGER Page for [OpenSDA firmware](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/opensda-sda-v2/).


## 3.  Editing the firmware
The firmware is currently all in `src/boot/ksdk1.1.0/`, in particular, see `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c` and the per-sensor drivers in `src/boot/ksdk1.1.0/dev*.[c,h]`.

The firmware builds on the Kinetis SDK. You can find more documentation on the Kinetis SDK in the document [doc/Kinetis SDK v.1.1 API Reference Manual.pdf](https://github.com/physical-computation/Warp-firmware/blob/master/doc/Kinetis%20SDK%20v.1.1%20API%20Reference%20Manual.pdf).

The firmware is designed for the Warp hardware platform, but will also run on the Freeacale FRDM KL03 development board. In that case, the only driver which is relevant is the one for the MMA8451Q. For more details about the structure of the firmware, see [src/boot/ksdk1.1.0/README.md](src/boot/ksdk1.1.0/README.md).

## 4.  Interacting with the terminal interface menu
When the firmware boots, you will be dropped into a menu with a rich set of commands. The Warp boot menu allows you to conduct most of the experiments you will likely need without modifying the firmware:
````
[ *				J	a	n	k	(rev. 1)			* ]
[  				      Cambridge / egdirbmaC   				  ]

[  		 HX711 Load Cell Testing Platform 		  ]

Select:
- '1': Read Raw data.
- '2': Read Calibrated data (mg).
- '3': Calibrate Sensor.
- '4': Tare Sensor.
- '5': Set Gain.
- '6': Set Number of reads.
- '7': Set raw output type.
- '8': Set moving average length.
Enter selection>
````

### 4.1: Read Raw data
Option 1 sends a stream of readings of the form

	timestamp, reading

'q' to quit.
### 4.2: Read Calibrated data
Option 2 is similar to option 1 but the values are shifted and multiplied according to set calibration factors (see 3 and 4)

### 4.3: Calibrate Sensor
Option 3 will present you with an option to enter an 8 digit calibration mass (mg)

	Enter chosen calibration mass (mg) and ensure scale is EMPTY (e.g '00500000')>

Ensure that upon entering the eighth digit the scale has no mass applied, as a taring process will take place immediately afterwards. Then a request to place the calibration mass on the scale will be made. After which any key can be pressed (enter is requested).

This process will set a tare zero offset and a multiplier scale offset to calibrate the scale to the requested mass.

### 4.4 Tare sensor

Option 4 will tare (zero) the scale to the currently applied mass.

### 4.5 Set Gain
Option 5 allows you to select one of the three gain options provided by the HX711:

	'1'=32; '2'=64; '3'=128 >

NOTE: gain 32 reads from input B on the HX711 whereas 64 and 128 reads from input A (default is 128).

### 4.6  Set number of reads
Option 6 allows you to select the number of reads before termination in Options 1 and 2:

	Enter number of value reads (e.g '1000' ) ('0000' is endless)>

### 4.7 Set raw output type
Option 7 allows the switching between HEX and DEC output for Option 1 (default is HEX)

	'1'=HEX; '2'=DEC >


### 4.8 Set moving average length

Option 8 allows you to select the size of the moving average buffer (default is 1):

	Enter number of readings to average (e.g '1000' ) ('0020' is max)>

NOTE: any value greater than 20 will simply set the buffer size to 20.

## 5.  Interacting with the OLED interface menu

A two button interface has been developed to access all the same features as discussed in section 4. If you are using the KL03 (SW2 = button 1; SW3 = button 2) in the default configuration.

Upon booting, the OLED will display the following


    +-----------------+
    | Raw DEC         | (1)-->: Open settings screen
    | Reading value   |
    |                 | (2)-->: Cycle between:
    +-----------------+         1) Raw DEC               2)   Raw HEX
                                3) Calibrated DEC (mg)   4)   Calibrated HEX

This will display a continuous measurement stream, with button 2 cycling the reading type.

Upon pressing button 1, following will be shown

	+-----------------+
	|                 | (1)-->: Return to readings stream
	|    Settings?    |
	|                 | (2)-->: 
	+-----------------+

## 6.  To update your fork
From your local clone:

	git remote add upstream https://github.com/physical-computation/Warp-firmware.git
	git fetch upstream
	git pull upstream master

----

### If you use Warp in your research, please cite it as:
Phillip Stanley-Marbell and Martin Rinard. “A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. ArXiv e-prints (2018). arXiv:1804.09241.

**BibTeX:**
```
@ARTICLE{1804.09241,
  author = {Stanley-Marbell, Phillip and Rinard, Martin},
  title = {A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation},
  journal = {ArXiv e-prints},
  archivePrefix = {arXiv},
  eprint = {1804.09241},
  year = 2018,
}
```

### Acknowledgements
This research is supported by an Alan Turing Institute award TU/B/000096 under EPSRC grant EP/N510129/1, by Royal Society grant RG170136, and by EPSRC grants EP/P0012476/1 and EP/R022534/1.

----
### Notes
<sup>1</sup>&nbsp; On some Unix platforms, the `JLinkRTTClient` has a double echo of characters you type in. You can prevent this by configuring your terminal program to not echo the characters you type. Alternatively, rather than using the `JLinkRTTClient`, you can use a `telnet` program: `telnet localhost 19021`. This avoids the JLink RTT Client's "double echo" behavior but you will then need a carriage return (&crarr;) for your input to be sent to the board. Also see [Python SEGGER RTT library from Square, Inc.](https://github.com/square/pylink/blob/master/examples/rtt.py) (thanks to [Thomas Garry](https://github.com/tidge27) for the pointer).
