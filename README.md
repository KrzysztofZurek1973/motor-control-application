# Motor Controller Test Application #

-------------------------------------------------------

## version 2022-05-10 ##

### Gdansk University of Technology ###

#### Krzysztof Zurek ####

-------------------------------------------------------
__Main features__:
* simulation of UWV communication (Morswin)
* set constant speed,
* set serial number,
* set Modbus address
* start bootloader
	
__Options__:
* -r - read
* -w - write
* -a - address
* -s - serial number
* -d - diodes test
* -v - speed
* -b - start bootloader
* -o - activate nBOOT1 bit (1), deactivate (0)
* -h - this help

__Examples__:
* -a 1			communication only with motor address 1
* -r -a 0			read Modbus address
* -r -a 1 -s 0	read serial number
* -w -a 0			reset Modbus address (set as 0)
* -w -a 2			set Modbus address as 2, possible only if current address is 0
* -w -a 1 -s 1200	write serial number as 1200
* -w -a 1 -v 1000	set speed as 1000 (for tests)
* -a 1 -b			start bootloader on motor with address 1
* -a 1 -o 1		set nBOOT1 bit on motor with address 1
