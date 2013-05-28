define flash
file main.elf
load
end

define reconnect
target extended-remote localhost:4242
file main.elf
set var {int}0x40021024=0x01000000
end

source -v jtag/armv7m-macros.gdb
reconnect
