ddTrackerArduinoDue
===================

C/C++/INO code for Arduino Due
This is incomplete software and is a test harness at the moment. It transmits fast scan
SSDV (at AFSK 1200 baud or 4800 baud raised cosine pulse) from a Serial Cameral and dummy telemetry information into a transmitter
module. Most of the hardwork has been done by Phil Karn (Reed Solomon encoder) and Philip 
Heron (SSDV breaking up of JPEG files for tranmission and reassembly).

My work has been mainly to get the comms quicker and more reliable on the telemetry and SSDV 
by using correlation tags at the begining of the packet so that start of frame can be 
detected and then the reed solomon encoder/decoder can do the hard work of keeping
the transmission reliable.

There is also a 4800 baud experimental modulator that has been commented out but does
work. Need to set the timer to / by 458 and re-lable the interrupt vector if you want to try 
this out.

Since it is a mixed source of files, there have been a few Kludges done to get it to compile
under the arduino IDE. This is running on an Arduino Due.
Cheers /DD VK3TBC

