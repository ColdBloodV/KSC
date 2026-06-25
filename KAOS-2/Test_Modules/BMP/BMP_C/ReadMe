BMP_C Folder contains files for the bmp module operated by the Raspberry Pi Pico. 

BMP.c is our main file for the folder. Before the pico will start up the BMP sensor and initilize at an altitude that was not reflecting the relative altitude 
of our launch altitude probably due to noise start up. So I changed up the how the altitude was calculated, added a procedure to find a sea level 
and added a FOR loop to filter out the startup noise when powering up. After that the altitude sits at a 0 with a margin of error within the decimals 
and loops forever until shutdown. 

Disclaimer: I am writing this long after i finished the software. But I remember the numbers drifting slightly up after a while of it running. 
I have not gone back to figure out why that happens. 
