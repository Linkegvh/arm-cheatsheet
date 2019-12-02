# EE2028 Consolidated notes
## Hardware architecture of a computer
### I/O
### Memory
#### Both program and its data are stored in the main memory
#### Load/store architecture
### Processor
#### Program counter
#### Instruction register
#### General purpose register
#### Control circuits
#### Arithmetic and logic unit (ALU)
## General coding related
### Branching and looping
#### Loop:
#### BNE loop
### Condition Codes
#### Flags
##### N
##### Z
##### V
##### C
### Memory allocation
#### Constant
##### .word
#### Static variable
##### .lcomm
### Addressing
#### Pre-addressing
##### Within [ ] and has a !
#### Post-addressing
#### Do not want to change address
##### LDR/STR Rt, [Rn {, #offset}]
#### PC-relative addressing
##### LDR Rd, ITEM
###### RD <— [PC + offset]
* Offset is calculated by the assembler
#### Offset
##### #imm12
###### 0 - 4095
##### #imm8
###### 0 - 255
##### #imm8m
###### Like above
##### #imm16
###### 0 - 65535
### Operands
#### Suffix
##### {S}
###### Updates the condition code flags
#### operand2
##### Constant
##### Register
###### Only register
###### Register with optional shift
* ADD R0, R1, R2, LSL #0x4
### Other small things
#### Move instructions
##### MOV
###### #imm8
* The mov instruction copies the data item referred to by its second operand (i.e. register contents, memory contents, or a constant value) into the location referred to by its first operand (i.e. a register or memory).
##### MOVW
###### #imm16
#### ADD/SUB
##### #imm12
#### Arithmetic instructions
##### MUL
###### Multiply
* Ro = R1 * R2
##### MLA
###### Multiply with Accumulate
* MLA R0, R4, R5, R6
    * R0 = R4 * R5 + R6
##### SDIV
#### Compare instructions
##### CMP
###### R1 - R2
##### CMN
###### R1 + R2
##### This will only update the condition flag and dump the result
#### Logical instructions
##### AND
###### Bit-wise AND
##### ORR
###### Bit-wise OR
##### EOR
###### Logical XOR
##### BIC
###### Logical AND NOT
* BIC R0, R0, R1
    * R1 is complemented first
    * Then AND with R0
##### ORN
###### Logical OR NOT
##### LSL
##### LSR
##### ASR
###### Retains the sign of this value
##### ROR
##### RRX
###### Move the carry flag into the MSB
## GPIO
### GPIO pin function
### GPIO registers
#### FIOxDIR
#### FIOxSET
##### Or with a value
#### FIOxCLR
#### FIOxPIN
#### FIOxMASK
### GPIO Interrupt Register
#### GPIO Overall Interrupt status register
##### IOIntStatus
#### GPIO Interrupt enable
##### IOxIntEnR
##### IOxIntEnF
#### GPIO interrupt Status
##### IOxIntClr
### Steps to config 
#### Decide whether rising edge or falling edge
#### Enable specific interrupt in GPIO interrupt enable register
##### LPC_GPIOINT -> IOxIntEnR/F |= 1 << x;
#### Enable EINT3 in NVIC
##### NVIC_EnableIRQ(EINT3_IRQn);
#### In EINT3 interrupt handler, Check which pin gives the interrupt
##### Void EINT3_IRQHandler(void)
##### (LPC_GPIOINT -> IOxIntStatR/F >> x) & 0x1
#### Then clear the interrupt and exit the handler
## Interface concept
### Protocol
#### Parallel
#### Serial
#### Synchronous
#### Asynchronous
#### Bus protocol
##### I2C
##### SPI
##### USB
#### Point to point
##### UART
#### Master-Slave
##### I2C
##### SPI
##### USB
#### Peer to peer
##### Eternet
#### Simplex
##### One way communication, one data line
#### Half-duplex
##### Two way communication, one data line
###### I2C
###### USB
#### Full-duplex
##### Two way communication, two data line back and forth
###### UART
###### SPI
#### In-band addressing
##### Same bus for address and data
###### I2C
###### USB
#### Out-of-band addressing
##### Different bus for address and data
###### SPI
## I2C
### Connection
#### SDA
#### SCL
##### In I2C init need to include a clock
### Protocol
#### Master generate start and stop
##### Start
###### SCL high
###### SDA high to low
##### End
###### SCL high
###### SDA Low to high
#### Msg followed by acknowledgement
##### A
##### NA
###### Only generated from the master
* Does not want any data
#### Normal communication
##### Unidirection
###### Start
* Address (7 bits) + R/W (1 bit)
    * A
        * Data (one byte)
            * A
            * NA (receiver mode)
                * Stop
            * A (tramsmitter mode)
                * Stop
* Read
    * 1
* Write
    * 0
##### Change of direction
###### Start
* Address (7 bits) + R/W (1 bit)
    * A
        * Data
            * A/NA
                * Start
                    * Address + R/W
                        * A
#### 12 steps to heaven
##### Step 1
###### Set the I2C operation to default (Clear AA, STA, EN by writing to I2CxCONCLR)
##### Step 2
###### Write 1 to STA bit of I2CxCONSET. Clear SI ( by writing to I2CxCONCLR). Wait for SI to be set (which can be checked by reading the CONSET register)
##### Step 3
###### Write the slave address (follow by WRITE bit) to I2CxDAT. Clear STA bit by writing to I2CxCONCLR. Clear SI. Wait for SI
##### Step 4
###### Write data to I2CxDAT. Clear SI. Wait for SI
* Remember that in TX mode, if the SI is set, it means the data is received and acknowledged by the slave
##### Step 5
###### Write 1 to STA bit of I2CxCONSET for repeating the start condition. Clear SI. Wait for SI
##### Step 6
###### Write the slave address (follow by READ bit) to I2CxDAT. Clear STA bit by writing to I2CxCONCLR. Clear SI. Wait for SI
##### Step 7
###### Set AA. Clear SI. Wait for Si
##### Step 8
###### Store I2CxDAT << 8 to read 16. Clear SI. Wait for SI
##### Step 9
###### Clear AA. Clear SI. Wait for SI
##### Step 10
###### Do reading 16 |= I2CxDAT to append the LSB. Clear SI. Wait for SI
##### Step 11
###### Write STO bit to I2CxCONCLR. Clear SI. Wait for SI
##### Step 12
###### Finish
### Registers
#### I2CONSET
##### I2C Control set register
###### Write 1 to a bit of this register to set the corresponding bit in the I2C control register
##### AA
###### Bit[2]
* Set 1
    * When more bytes are expected from the slave, this causes an acknowledge to be sent to the slave upon receiving a byte
* Set 0
    * When no more bytes are expected from the slave, this castes an NA to be sent to the slave upon receiving a byte
##### SI
###### Bit[3]
* Set 1
    * Hardware is not busy
        * It indicates to the hardware that it can enter the next local step
    * Auto-clear to 0
* 0
    * Hardware is busy
##### STO
###### Bit[4]
##### STA
###### Bit[5]
* 1
    * if not in master mode, enter master mode
    * If in master mode, transmit a repeated start to hold the line to change into RX/TX
    * If the bus is not free, wait for the bus to be free
* 0
    * Nothing
##### I2EN
###### I2C interface enable
* 1 enable
* 0 disable
###### Bit[6]
##### Bit[31:7] reserved
#### I2STAT
##### I2C status register
###### Provides detailed status codes of the I2C operation
#### I2DAT
##### Data register
###### During master or slave transmit mode, data to be transmitted is written to this register
###### During master or slave receive mode, data received are stored first in this register and can be read from it
#### I2CONCLR
##### I2C Control Clear register
###### Write 1 to a bit in this register, the corresponding bit in the I2C control register will be cleared
### Initialisation
#### I2C_init(LPC_I2C2, 100000)
##### Define the clock rate
#### I2C_SetClock(I2C2, 100000)
#### I2C2 -> I2CONCLR = 0000 0000 0000 0000 0000 0000 0011 0100
##### = (1 << 2) | (1 << 4) | (1 << 5)
###### Basically clearing AA, STO, STA
#### I2C_Cmd (LPC_I2C2, ENABLE)
##### I2C2-> I2CONSET = (1 << 6)
## Interrupts and NVIC
### Groups
#### System exception
##### 15 system exceptions
###### 1
* Reset
    * Priority -3
###### 2
* NMI
    * Priority -2
###### 3
* Hard fault
    * Priority -1
###### 4 - 15 have all programmable priority
###### 15
* Systick
    * Priority = programmable
#### External interrupts
##### All interrupts from a particular peripheral are considered as one single interrupt by NVIC
###### UART3 interrupt
###### GPIO interrupt
### Priority levels
#### Exceptions
##### TOP 3 have fixed priority
#### External interrupts
##### Preempt priority
###### Whether to kick another interrupt out of CPU
##### Sub-priority
###### When two are pending, which one to handled first, will not kick the same preempt priority out of CPU
### Normal process
#### First interrupt
#### Then pend the interrupt
##### Pending
###### A state of an exception / interrupt waiting for the CPU to process
#### CPU process
#### Clear the interrupt
##### it is possible to clear the interrupt before the CPU processes it
### Registers
#### Interrupt priority register (IPR)
##### 4 PRI field of 8-bit width
###### Each PRI field is used to assign a priority level (from 0 - 255) to interrupt
* 0 is the highest priority
* 8 bits
    * IPR width
    * Only the most few significant bits are used 
        * Writing to the other bits will be ignored
##### Priority group
###### 0
* Preempt field
    * Bit[7:1]
* Sub-priority field
    * Bit[0]
###### 1
* Preempt field
    * Bit[7:2]
* Sub-priority field
    * Bit[1:0]
###### 2
* Preempt field
    * Bit[7:3]
* Sub-priority field
    * Bit[2:0]
###### 3
* Preempt field
    * Bit[7:4]
* Sub-priority field
    * Bit[3:0]
###### 4
* Preempt field
    * Bit[7:5]
* Sub-priority field
    * Bit[4:0]
###### 5
* Preempt field
    * Bit[7:6]
* Sub-priority field
    * Bit[5:0]
###### 6
* Preempt field
    * Bit[7]
* Sub-priority field
    * Bit[6:0]
###### 7
* Preempt field
    * None
* Sub-priority field
    * Bit[7:0]
###### The whole program can only have one priority group number, meaning that you need to choose carefully on the pre-empt field and the sub-priority field that you want to have for your program
###### Default group is 0
#### ISER
##### Interrupt set enable register
###### RW
* Each bit corresponds to one interrupt
##### Enable interrupts
##### Determine which interrupts are currently enabled 
#### ICER
##### Interrupt clear enable register
###### RW
* Each bit corresponds to one interrupt
##### Disables interrupts
##### Determine which interrupts are currently disabled
###### Don’t really need to use these two registers tho, some other functions will use it 
#### ISPR
##### Interrupt set pending register
###### Bit reference
##### Force interrupts into pending state
##### Determine which interrupts are currently pending
###### RW
#### ICPR
##### Interrupt clear pending register
###### Bit reference
##### Clear pending interrupts
##### Determine which interrupts are currently pending
###### RW
##### Clearing pending status might also be need to be done init he process of recovering from errors as hardfault etc
#### IABR
##### interrupt active bit register
###### Determine which interrupts are active
* R
#### PRIMASK
##### 1 bit
##### Set to allow only NMI and hard fault exceptions, other interrupts and exceptions (having priority >= 0) are masked (disabled)
#### FAULTMASK
##### 1 bit
##### Set to only allow NMI, all others are masked including hard fault
#### BASEPRI
##### 8 bit
##### Used to disable all exceptions / interrupts having priority >= certain priority
### Vector table
#### A part of memory used to store the starting address of the various exception / interrupt handlers
#### Default starts at 0
#### Address is arranged as exception / interrupt number * 4
### Configuring
#### NVIC_SetPriorityGrouping(x);
#### NVIC_SetPriority( some_interrupt, effective priority - NVIC_PRIO_BITS)  
##### Something
###### The interrupt that we want to set
* EINT3_IRQn
* UART2
##### effective priority - NVIC_PRIO_BITS
###### As the function will shift the second argument left by NVIC_PRIO_BITS before writing to IPR
#### NVIC_ClearPendingIRQ( Some_interrupt)
#### NVIC_EnableIRQ ( Some_interrupt)
#### Void some_interrupt_handler (void){
Stuff
}
## UART
### Protocol
#### RX
##### SIPO
###### Check for start bit
###### After getting the start bit, sample at next bit time
* Store the data in shift register to put into FIFO
    * System get data out from FIFO
###### After receiving stop bit, check for start bit again
#### TX
##### PISO
###### Generate a start bit
###### Shifts the required number of data bits out to the line
###### Parity bit
###### Appends the stop bits
#### Stuff needs to know
##### Data line is default high
###### optional parity bit to check for data loss
##### Start bit
###### Logic 0
###### Lasts longer than half of a bit time
* Meaning more than 8 pulses
##### Stop bit
###### Logic 1
##### UART clock is running at a multiple (16) of data rate ==> each bit is 16 clock pulses
###### The receiver is always searching for start bit
###### If a start bit is received
* Wait for one bit and sample and put the content on shift register
##### UART Voltage
###### 3.3V
###### 0V
#### Character Framing
##### Start
###### DATA (LSB send first) (configurable amount of bits but usually 8)
* Optional parity bit
    * Stop
### Registers
#### UnRBR
##### Receive buffer register
###### Processor reads from UnRBR to extract the oldest character from the Rx FIFO
###### Reading UnRBR will remove the character from the Rx FIFO
#### UnTHR
##### Transmit holding register
###### Processor writes to UnTHR to insert the newest character into the Tx FIFO
###### The character is removed from the Tx FIFO when the RSR starts transmitting it
#### UnLCR
##### Line control register
###### Determines the format of data character to be transmitted or received
###### Bit
* 1:0
    * Word length select
* 2
    * Stop bit select
* 3
    * Parity enable
* 5:4
    * Parity select
* 6
    * Break control
* 7
    * Divisor latch access bit (DLAB)
* 32:8
    * Reserved
#### UnLSR
##### Line status register
###### Provides the status information on the UARTn Tx and Rx blocks
###### Bit
* 0
    * Receiver data ready (RDR)
* 1
    * Overrun error
        * When it happens, it means Rx FIFO is full and the data received in shift register will be overwrittened instead of saved
* 2
    * Parity error (PE)
        * When the parity bit of a received character is in the wrong state
* 3
    * Framing error (FE)
        * When the stop bit of a received character is a logic 0, a framing error occurs. 
* 4
    * Break interrupt (BI)
        * When RXDn is held in the spacing state (all zeros) for one full character transmission (start, data, parity, stop), a break interrupt occurs.
        * When this occurs, the receiver goes idle until RXDn goes to marking state (all ones)
        * UnLSR read clears this status bit
* 5
    * Transmitter holding register empty (THRE)
        * When the THR is empty
        * Clear on a UnTHR write
* 6
    * Transmitter empty (TEMT)
        * Set when both UnTHR and UnTSR are empty
        * Clear when either UnTSR or the UnTHR contain valid data
* 7
    * Error in Rx FIFO (RXFE)
        * When a character with a Rx error such as framing error, parity error or break interrupt, is loaded into the UnRBR. 
        * Clear when the UnLSR register is read and there are no subsequent errors in the UARTn FIFO
* 31: 8
    * Reserved
#### UnTER
##### Transmit enable register
###### Bits
* 6:0
    * Reserved
* 7
    * TXEN
        * When 1
            * Data written to the THR is output on the TXD pin as soon as any preceding data has been sent.
        * When 0
            * Means the character is sent and more further sending will happen till it becomes 1
    * Flow control
        * Devices let each other know when they are ready to accept data
            * If want data
                * XON (0X11) character
                    * Set the TXEN
            * If do not want more data
                * XOFF (0x13) character
                    * Clear the TXEN to stop transmission
### Transmission
#### RX
##### UART_ReceiveData
###### Check if the UnRBR has got data inside through checking the line status register for RDR
###### If has
* Arr[i] = LPC_UART0 -> RBR & 0xFF
    * The smart way is to use a pointer instead of a physical passed in array
###### If does not have
* If has time out
    * Wait
    * If time out
        * Break
* If does not have time out
    * Break
#### TX
##### UART_SendData
###### Check if the THR is empty by checking LSR -> THRE
###### If it is empty
* Send in FIFO size amount of data
    * LPC_UART0 -> THR = Arr[i]& 0xFF
        * The smarter way to do this is to use a point instead of a physical passed in array
###### If it is not empty
* If got time out
    * Wait
    * If time out
        * Break
* If does not have time out
    * Break
## Stuff that you need to know
### Word and byte encoding
#### Word
##### 32 Bits
#### Byte
##### 8 bits
##### One ASCII character
##### ARM Controller is byte addressable
###### There is an address to each byte
### Stack and subroutines
#### Push
##### push away registers that are not the input!
#### Pop
##### Pop the pushed away register back
### Registers
#### 15 Registers
##### Low register
###### 0 - 7
##### High register
###### 8 - 12
##### Stack pointer
###### R13
##### Link Register
###### R14
##### Program counter
###### PC
#### Special registers
##### Program status regsisters
###### Application program status register
###### interrupt program status register
###### Execution program status register
##### Interrupt mask registers
###### PRIMASK
###### FAULTMASK
###### BASEPRI
##### Control register
### CMSIS
#### Cortex Microcontroller Software Interface Standard
##### CMSIS + C libraries enable access of RAM core and peripherals functionalities, and interfacing with other devices
###### Enables scaling up to systems
###### Can reuse template codes very easily
##### It defines
###### The register names of the core peripherals
###### The names of the core exception vectors
###### A common way to access peripheral registers
###### A common way to define exception vectors
###### Thus, we just need to specify the ports and pins that we want to access. The CMSIS functions will retrieve the address of that peripheral register by itself
