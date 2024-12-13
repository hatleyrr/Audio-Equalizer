# Audio-Equalizer
ECE 287 Final Project for Section B - Authors: Royce Hatley &amp; Samuel Kanaan
***

This project is a digital audio equalizer that uses Verilog, Quartus, and an FPGA to filter audio signals stored in a sample (sample recording module made by Dr. Peter Jamieson). This code was used on a DE1-SOC FPGA, I cannot guarantee that it will work on other models.

Control is included in a README.txt file in the project files, but I will include it here too:
- Royce Hatley and Samuel Kanaan - Tested on DE1-SOC - Dec 13th, 2024

Headphones connected to Green out RCA jack
Laptop connected to Pink in RCA jack through an auxiliary cable

Instructions for the sample recorder made by Peter Jamieson are included below. 

To use the equalizer, follow these steps:
Step 1: Record a sample into the FPGA
Step 2: Turn a switch on depending on the desired frequency adjustment. The control is:
	SW[9] - Low-bass
	SW[8] - Bass
	SW[7] - High-bass
	SW[6] - Low-mid
	SW[5] - Mid
	SW[4] - High-mid
	SW[3] - Low-treble
	SW[2] - Treble
	SW[1] - High-treble
Step 3: Hold down KEY[3] (indicated by LEDR[9] being on) while pressing KEY[2] to play the filtered audio through the line out.

- Peter Jamieson - Tested on DE1-SOC - Nov 10th, 2024

Hooked up mic to Green out RCA jack
Hooked up RCA cable between my little radio and Pink in RCA jack

- To operate there are 2 modes

Mode 1 - Playthrough sound = SW[0] is set to 0

Mode 2 - Sample sound into 64K = SW[0] is set to 1
	To play (pre-recorded) press KEY[2] - note this will need to be programmed again once you record over it
	To record signal press KEY[1] and release - note the CLUDGE in the system that it seems the system records at 16.6666kHz rate
	To play back sample press KEY[2] and release
	
Note that this uses 50% of the on-chip memory!!!  Also, the signal tap stuff I setup takes away some of the memory.

Also, in this is some C code to convert a .wav file at PCM-16 to a hex init file.  It works now to convert the file into a .hex init file with proper checksums.
