# IMCTFD
IMCTFD: Improved Microchip CAN Teensy FlexData Library (MCP2517FD)

  Welcome to IMCTFD!

    This library was developed based on IFCT, libraries developed for and used on the Teensy 3.x platform.
There are several benefits that are not even covered in the current Microchip API, or any other libraries.

1) The library uses teensyThreads by ftrias (https://github.com/ftrias/TeensyThreads)
   It uses it as a background process to control the object's interrupt pin.

2) Connect multiple MCP2517FD's ( good for bridging / network segmenting )
3) Use any SPI interface, SPI, SPI1, SPI2, 2 or more modules can even share the same bus. (Uses SPI Transactions by Paul Stoffregen https://github.com/PaulStoffregen/SPI)

4) Automatic filters and masking! No more hand written filter setup for the user. Input your IDs, and let the library configure it all for you! A filter printout is available for the Serial monitor to see which filters exist, and what channels they affect.

5) First time ever, to introduce 12 bit Standard ID implementation in a user interface. Filters, Transmission, and Reception inclusively.

6) Full FIFO / TEF / TXQ configuration, with Serial monitor configuration printout, and memory usage calculation of the MCP2517FD RAM.

7) Ordered initialization. Run all your FIFO/TEF/TXQ configurations, then initiate ::begin(), this will reset the chip and begin an ordered sequence init recommended by the Microchip API documentation.

8) Polling vs Interrupts! Individual callbacks for FIFOs 1-31 and TEF in interrupt mode. The library is capable of polling FIFOs that don't have enabled interrupts using Obj.read(msg); or, specifically, Obj.read(msg,FIFO3);
TEF is available in read(msg, TEF) call if interrupt is disabled, otherwise, you will receive it in the callback.
The TEF contains transmission receipts for all transmissions, including their sequence number (7 bits total). This allows users to assign a sequence to their frame, and confirm that it's been sent later on.
TXQ can be transmitten to using the Obj.write(msg, TXQ) specifically, otherwise all other writes goto FIFO channel available TX channels, or one specified by the user.

9) Selective FIFO queueing but not sending. Allows the user to assign a FIFO to queue mode only, and when the user is ready, to initiate the write and restore FIFO behaviour (automatic sending).

10) Optional SPI access with or without CRC transfers! User can enable CRC transactions via Obj.enableSPICRC();
    This ensures bad SPI traffic does not write garbage to the registers.

11) pinMode and digitalRead/digitalWrite available for the 2 pins on the MCP2517FD. 
12) automatic DLC calculations! Don't worry about not knowing the DLC (15 == 64 bytes, 9 == 12 bytes). Just assign your data length (msg.len) and if your length is smaller than the calculated next DLC it will be padded with 0xAA bytes.

13) dynamic read(msg)/write(msg) assure you write to a properly sized FIFO. If you're data can't fit in a FIFO because you have it set to use a smaller payload size to get more deep queues, it will scan to the next available FIFO for validation.

  Typical usage:

//Header:
 #include "IMCTFD.h"

// Constructor. In my case, I'm using specific pins on SPI1 on both 3.5 and 3.6. 0 is the interrupt pin attached
to pin "INT" of the MCP2517FD.
IMCTFD FD = IMCTFD(SPI1, 5, 21, 32, 31, 0);

Set the baudrate and multiplier:
FD.setBaudRate(500000, 8); // 500kbps arbitration, 4Mbit data transfers

// Here we setup some FIFO channels, 3 RX, 2 TX, plus TEF and TXQ if user wishes.
FD.configureFIFO(FIFO1, FIFO_RX, PLSIZE_5, DEPTH_4, PRIORITY_1, TIMESTAMP_ON);
FD.configureFIFO(FIFO2, FIFO_TX, PLSIZE_7, DEPTH_4, PRIORITY_1, TIMESTAMP_ON);
FD.configureFIFO(FIFO3, FIFO_RX, PLSIZE_7, DEPTH_4, PRIORITY_1, TIMESTAMP_ON);
FD.configureFIFO(FIFO4, FIFO_TX, PLSIZE_7, DEPTH_4, PRIORITY_1, TIMESTAMP_ON);
FD.configureFIFO(FIFO5, FIFO_RX, PLSIZE_7, DEPTH_6, PRIORITY_1, TIMESTAMP_ON);
FD.configureTEF(DEPTH_3, TIMESTAMP_ON);
FD.configureTXQ(PLSIZE_3, DEPTH_3, PRIORITY_1);

FD.setFIFOFilterRange(FILTER_0, FIFO1, 0x1, 0x3 );
FD.setFIFOFilter(FILTER_1, FIFO1, 0x1F5002, 0x1F5803 ); // here we capture 2 extended IDs
FD.setFIFOFilter(FILTER_2, FIFO3, 0x5 );
FD.setFIFOFilter(FILTER_3, FIFO5, 0x4 ); // here we capture a 11bit standard ID.
FD.setFIFOFilter(FILTER_4, FIFO1, 0xFE0, SID12 ); // here we setup a 12 bit standard id to capture in FIFO1 !

FD.enableSPICRC(); // optional, enable CRC SPI transfers.

// Here we enable a few interrupts for reception callbacks
FD.enableFIFOInterrupt(FIFO5);
FD.enableFIFOInterrupt(FIFO2);
FD.enableFIFOInterrupt(TEF);

// assign a global callback for all interrupt FIFOs
FD.onReceive(myCB);

// assign a specific callback for frames comming into FIFO1 interrupt
FD.onReceive(FIFO1, myCB);

// finally... Initiates everything and enables the threading
FD.begin();

Typical usage similar to IFCT:
```
CANFD_message_t msg; <-- CANFD struct
msg.len = 21; <-- we say 21, but automatic DLC conversions will send out a 24 byte frame, with the 3 last bytes padded 0xAA.
msg.fdf = 1; <-- specify that it's an FD frame, >=8 bytes. Not doing so will send a CAN2.0 frame truncated at 8 bytes.
for ( uint8_t i = 0; i < 64; i++ ) msg.buf[i] = i + 1; <-- assign values to the struct data
msg.id = 0xFE3; <-- standard ID (look next line)
msg.flags.sid12 = 1; <-- we specify this as a 12 bit standard ID. If you would like to send it as an Extended ID, change the value to 0. As we all know, arbitrations are faster for standard IDs rather than extended IDs.
msg.fifo == the channel the message came from. This is updated when dynamically reading mailboxes in poll mode, or when they fire in interrupts.
```

Here are some printouts of past configurations during tests:
```
	##########################################
	#	*** TEF Configuration ***	 #
	##########################################
	#	    FIFO Size: 3 		 #
	#	    TimeStamp: Enabled 		 #
	#	    Interrupt: Enabled 		 #
	#	    RAM usage:   36 (Bytes)	 #
	##########################################

	##########################################
	#	*** TXQ Configuration ***	 #
	##########################################
	#	    Payload Size: 20		 #
	#	    FIFO Size: 3 		 #
	#	    Priority: 1 		 #
	#	    Interrupt: Disabled		 #
	#	    RAM usage:   84 (Bytes)	 #
	##########################################

	##########################################
	#	*** FIFO Configuration ***	 #
	##########################################
	#	    FIFO1: Receive FIFO     	 #
	#	    Payload Size: 32		 #
	#	    FIFO Size: 4 		 #
	#	    TimeStamp: Enabled 		 #
	#	    Interrupt: Disabled		 #
	#	    RAM usage:  176 (Bytes)	 #
	##########################################
	#	    FIFO2: Transmit FIFO    	 #
	#	    Payload Size: 64		 #
	#	    FIFO Size: 4 		 #
	#	    Priority: 1 		 #
	#	    Interrupt: Enabled 		 #
	#	    RAM usage:  288 (Bytes)	 #
	##########################################
	#	    FIFO3: Receive FIFO     	 #
	#	    Payload Size: 64		 #
	#	    FIFO Size: 4 		 #
	#	    TimeStamp: Enabled 		 #
	#	    Interrupt: Disabled		 #
	#	    RAM usage:  304 (Bytes)	 #
	##########################################
	#	    FIFO4: Transmit FIFO    	 #
	#	    Payload Size: 64		 #
	#	    FIFO Size: 4 		 #
	#	    Priority: 1 		 #
	#	    Interrupt: Disabled		 #
	#	    RAM usage:  288 (Bytes)	 #
	##########################################
	#	    FIFO5: Receive FIFO     	 #
	#	    Payload Size: 64		 #
	#	    FIFO Size: 6 		 #
	#	    TimeStamp: Enabled 		 #
	#	    Interrupt: Enabled 		 #
	#	    RAM usage:  456 (Bytes)	 #
	##########################################

	##########################################
	#    Total RAM capacity: 2048 (Bytes)	 #
	#    Total RAM usage:    1632 (Bytes)	 #
	#    RAM left available:  416 (Bytes)	 #
	##########################################

	##########################################
	#	*** Filter Configuration ***	 #
	##########################################
	#	    Filter_0 			 #
	#	    Standard ID: 0x1         	 #
	#	    Mask: 0x7FC       		 #
	#	    Assigned to: FIFO1 		 #
	#	    Enhancement: Disabled	 #
	##########################################
	#	    Filter_1 			 #
	#	    Extended ID: 0x1F5002    	 #
	#	    Mask: 0x1FFFF7FE  		 #
	#	    Assigned to: FIFO1 		 #
	#	    Enhancement: Disabled	 #
	##########################################
	#	    Filter_2 			                     #
	#	    Standard ID: 0x5                	 #
	#	    Mask: 0x7FF       	             	 #
	#	    Assigned to: FIFO3 	            	 #
	#	    Enhancement: Disabled           	 #
	##########################################
	#	    Filter_3 			 #
	#	    Standard ID: 0x4         	 #
	#	    Mask: 0x7FF       		 #
	#	    Assigned to: FIFO5 		 #
	#	    Enhancement: Disabled	 #
	##########################################
```

