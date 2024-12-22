CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld

LDSCRIPT = ./STM32G0B1RE.ld

CPUFLAGS = -mcpu=cortex-m0 -mthumb
CFLAGS = -O -g -Wall
LDFLAGS = -Xlinker -Map=program.map -nostartfiles -T $(LDSCRIPT)

GCCLIB := $(shell $(CC) --print-search-dirs | 	\
		   sed -E -n '/^libraries: =([^:]*)\/.*/s//\1/p')

LIBGCC := $(word 1, $(wildcard 					\
		   $(GCCLIB)/thumb/v6-m/libgcc.a 	  	\
		   $(GCCLIB)/thumb/v6-m/nofp/libgcc.a 	\
		   UNKNOWN))

all: program.hex

%.o: %.c
		$(CC) $(CPUFLAGS) -c $< -o $@

%.elf: startup_stm32g0b1xx.o main.o
		$(CC) $^ $(LIBGCC) ${LDFLAGS} -o $@
		arm-none-eabi-size $@

%.hex: %.elf
		arm-none-eabi-objcopy -O ihex $< $@

clean:
		rm *.hex *.map