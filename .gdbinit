define flash
file main.elf
load
end

define reconnect
target extended-remote localhost:4242
file main.elf
set var {int}(0x40021024)=0x0C000000
end

reconnect
