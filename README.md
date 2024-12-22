

### Downloading dependecies

  - [STM32 Programming Toolset](https://github.com/stlink-org/stlink) + dependencies
    ```sh
    sudo apt-get install git make cmake libusb-1.0-0-dev
    sudo apt-get install gcc build-essential

    wget https://github.com/stlink-org/stlink/releases/download/v1.8.0/stlink_1.8.0-1_amd64.deb -q --show-progress
    sudo dpkg -i stlink_1.8.0-1_amd64.deb
    ```
    Connect the board to an USB port and to test its connectivity you can execute the followind command:
    
    ```sh
    lsusb | grep STM 
    Expected output: STMicroelectronics ST-LINK/V2.1
    ```

    The **COM LED** should also be **green**.
    A problem with USB enumeration is indicated by a blinking red **COM LED**.

    To see more specific information about the board you can execute the following command:
    <pre>
    st-info --probe

    Found 1 stlink programmers
        version:    V2J37S27
        serial:     066BFF373146363143223537
        <span style="background-color: rgba(245, 236, 39, 0.44)">flash:      524288 (pagesize: 2048) <b>(512KB)</b></span>
        <span style="background-color: rgba(245, 236, 39, 0.44)">sram:       147456 <b>(144KB)</b></span>
        chipid:     0x467
        dev-type:   STM32G0Bx_G0Cx
    </pre>

## Startup Script

### ARM Cortex-M0+ Reset Behavior

The Cortex-M0+ has the following reset behavior according to this well documented presentation [here](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://community.arm.com/support-forums/f/soc-design-and-simulation-forum/54006/understanding-reset-sequence-cortex-m0&ved=2ahUKEwjPjMOM3riKAxUfiv0HHXUTGkEQFnoECBcQAQ&usg=AOvVaw0FCZVU6DNUNLWvWUzluuz3):
  - Reads the **Initial SP**, also called the **MSP (Main Stack Pointer)** 
  - Reads the **reset vector**
  - Branches to the starting of the program execution address (**reset handler**)
  - Subsequently executes program instructions (TBD in the `Reset_Handler` function)

### 

### `Reset_Handler`
  A strong reference point was [this forum post](https://allthingsembedded.com/post/2019-01-03-arm-cortex-m-startup-code-for-c-and-c/) and also [this implementation](https://github.com/elzoughby/STM32H7xx-Startup/tree/master) for the **STM32H7xx** series.

  1. Define the interrupt vector table for the NVIC according to the Cortex-M0+ Manual and STM32G0x1xx series reference Manual.

  ```c
  void (* g_pfnVectors[])(void) __attribute__((section (".isr_vector"))) = {...}
  ```
  2. Move the **MSP** to the **SP (Stack Pointer**) using inline assembly and `LDR` instruction _(see page 141 from ARMv6-m Architecture Manual)_. It uses the `_estack` symbol defined in the linker script.

  ```c
  __asm (
        "LDR R0, =_estack       \n\t"
        "MOV SP, R0             \n\t"
    );
  ```
  3. Initialize the `.data` section. In order to use symbols from the ``.data`` section, the startup code needs to copy the data from ``LMA (Flash)`` to ``VMA (SRAM)`` to make sure that all C code can access the initialized data. It uses the `_data_start`, `data_end` symbols for the virtual adresses and `_sidata` symbol for the start address of the LMA of the `.data` section.

  ```c
  uint32_t *dataSrc = &_sidata, *dataDest = &_data_start;
    while (dataDest < &_data_end) {
        *dataDest++ = *dataSrc++;
    }
  ```
  4. Initialize the `.bss` section to zero using inline assembly. It uses the `_bss_start` and `_bss_end` symbols defined in the linker script.

  ```c
    __asm (
        "LDR R0, =_bss_start      \n\t"
        "LDR R1, =_bss_end        \n\t"
        "MOV R2, #0               \n\t"
        "loop_zero:               \n\t"
        "   CMP 	R0, R1          \n\t"
        "   BGE 	end_loop        \n\t"
        "   STR	R2, [R0]        \n\t"
        "   ADD   R0, R0, #4      \n\t"
        "   B 	loop_zero       \n\t"
        "end_loop:                \n\t"
    );
  ```
### `System_Init`
The purpose of this function is to configure the `PLL Block` of the `Clock tree` with a frequency of **64 MHz** and to select `PLLRCLK` as the `SYSCLK` source.

The formula by which the output f<sub>PLLR</sub> is calculated is the following:
<pre>
f<sub>PLLR</sub> = ((f<sub>PLLIN</sub> / M) * N) / R,
where:
  - f<sub>PLLIN</sub> is the frequency of HSI16 clock source of 16 MHz
  - M, N and R are configurable parameters using PLLCFGR register
</pre>


### Documentation
  - [STM32G0x1xx Reference Manual](https://www.st.com/resource/en/reference_manual/rm0444-stm32g0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
    * Memory Organization (Chapter 2.2.2 Memory map and register boundary addresses - **pg. 62**)
    * System Clock Selection (Chapter 5. Reset and clock control (RCC) - **pg. 155**)
  - [STM32 Cortex-M0/M0+ Programming Manual](https://www.st.com/resource/en/programming_manual/pm0223-stm32-cortexm0-mcus-programming-manual-stmicroelectronics.pdf)
  - [ARMv6-M Architecture Reference Manual](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://users.ece.utexas.edu/~valvano/mspm0/Arm_Architecture_v6m_Reference_Manual.pdf&ved=2ahUKEwjn54aZnLmKAxX4nf0HHYpSDTkQFnoECBIQAQ&usg=AOvVaw1wJfJ_0J4djK_M9sc94aE1)



## Linker Script



| **KEYWORD** | **DESCRIPTION**                                                                                                                                                                       |
|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `SECTIONS`  | Defines the mapping between the sections of the input object files to the sections of the linker's output file, by specifying the memory layout of each output section.                |
| `KEEP`      | Tells the linker to not remove the section wrapped by this command. Important when working with ARM microprocessors because the interrupt vector table must be in a predefined memory location and is not referenced directly by code. |
| `LMA`       | Load Memory Address.                                                                                                                                                                  |
| `VMA`       | Virtual Memory Address.                                                                                                                                                               |
| `.`         | Location Counter.                                                                                                                                                                     |
| `ALIGN`     | Introduces the required amount of padding to align the location counter.                                                                                                              |


## Usage

To be able to run the rules inside the provided `Makefile` you are required to also install the [ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads):

```sh
  sudo apt-get install gcc-arm-none-eabi
```

1. Create `.hex` file using the Makefile rule:
```sh
make all
```
2. Flash the device
```sh
st-flash --format ihex write program.hex

2024-12-22T16:36:26 INFO common.c: STM32G0Bx_G0Cx: 144 KiB SRAM, 512 KiB flash in at least 2 KiB pages.
2024-12-22T16:36:26 WARN common_flash.c: Flash base use default L0 address
2024-12-22T16:36:26 INFO common_flash.c: Attempting to write 508 (0x1fc) bytes to stm32 address: 134217728 (0x8000000)
-> Flash page at 0x8000000 erased (size: 0x800)
2024-12-22T16:36:26 INFO flash_loader.c: Starting Flash write for WB/G0/G4/L5/U5/H5/C0

2024-12-22T16:36:26 INFO common_flash.c: Starting verification of write complete
2024-12-22T16:36:26 INFO common_flash.c: Flash written and verified! jolly good!
2024-12-22T16:36:26 INFO common.c: Go to Thumb mode
```

(OPTIONAL) 3. Start a `gdb` connection using `st-util` and connect locally using `gdb-multiarch`

```sh
sudo apt-get install gdb-multiarch

# Inside the first terminal window
st-util

# Inside the second terminal window
gdb-multiarch

(gdb) file program.hex
(gdb) target remote localhost:4242
(gdb) ...
```