# OpenMowerGOATO600

## Jailbreak your GOAT

There's a UART port on the bottom, next to the front wheel. Unscrew the two screws and pop the hatch to expose it. Everything's 3.2V. Pin layout is:
```
    __
[. . . . G]
[T R . . .]
```
```
flowcontrol    : none
baudrate is    : 115200
parity is      : none
databits are   : 8
stopbits are   : 1
```

1. Grab your favourite serial interface, connect to UART port, reboot the GOAT, press ctrl+C a few times while it boots until it drops you at a shell 
2. enter 
```
setenv sploit 'androidboot.verifiedbootstate=green ignore="'
```
then 
```
setenv setbootargs 'setenv bootargs console=ttyS0,115200n8 root=PARTLABEL=system${slot_suffix} ro rootwait ${sploit}'
```
then
```
setenv avb_boot 'echo slot=${slot_suffix}; run setbootargs; part size mmc 0 boot${slot_suffix} bootimagesize; part start mmc 0 boot${slot_suffix} bootimageblk; mmc read ${kernel_addr} 0x${bootimageblk} 0x${bootimagesize}; echo booting...; bootm ${kernel_addr}#boardid-0x2000'
```
3. enter `saveenv` to store these values for future boots
4. enter `setenv sploit 'androidboot.verifiedbootstate=green init=/bin/sh ignore="'` to boot into a root init shell this time
5. enter `boot`
6. this will drop you at a root shell. Remount the root drive as rw with `mount -o remount,rw /` then use `passwd` to set a password
7. `mount /dev/mmcblk0p16 /data` then `touch /data/sshd_to_be_run` to set a sshd server to start on boot
8. reboot!
9. disconnect your UART and close the port, we're all done! 

congrats, you can now make whatever modifications you like to the filesystem and kernel and it'll boot without verification. You should be able to ssh into your GOAT and do whatever you like.

Then, when I've finished working out the OM/GOAT shim, you'll just copy it over and run a script and it'll be done.
