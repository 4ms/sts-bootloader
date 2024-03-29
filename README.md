# STS-bootloader

## Audio bootloader for the Stereo Trigged Sampler

The core of this bootloader is from the stm-audio-bootloader from [pichenettes](https://github.com/pichenettes/eurorack). The bootloader.cc file has been modified, and interface/driver files have been added for use with the hardware UI of the Stereo Trigged Sampler (STS).

The STS project is located [here](https://github.com/4ms/STS)

This bootloader works with STS code that's less than 480kB in size. The reason for this limit is because the audio file is flashed into the upper sectors first (kStartReceiveAddress), and then copied over to the execution sectors (kStartExecutionAddress) only once all data has been received.

## Setting up your environment

Set up the environment exactly as you would in the [STS project](https://github.com/4ms/STS)

## Compiling

To compile, simply run:
	
	make

This creates bootloader.bin and bootloader.elf files in the build/ directory

---

To flash the bin file to your DLD hardware using an st-link programmer:

	make flash
	
This will program the first sector with the bootloader (0x08000000).
Note that in the DLD, the second sector (0x08004000) is used for system settings, so the bootloader must fit into the first 16kB sector or else the DLD code will need to be altered to prevent overwriting the bootloader.

---

To combine the bootloader with the STS application code into one bin file:

	make combo

This requires ../STS/build/main.bin to be already present. A new file combo.bin will be created.

---

To flash combo.bin using an st-link programmer:

	make combo_flash
	
