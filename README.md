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

## Compile OpenMower

You gotta compile on the GOAT, so use your shiny new SSH login and run
```
mount -o remount,rw /
apt install --reinstall git
mount -o remount,ro /
cd /var
git clone https://github.com/amylizzle/OpenMowerGOATO600.git
cd OpenMowerGOATO600
./chroot_devenv.sh
```
This will take a while. It sets up a chroot dev environment under /var, builds and then runs OpenMower. 

# Development Environment

If you want to do development on your host use
```
sudo UBUNTU_MIRROR="https://archive.ubuntu.com/ubuntu/" HOST_ARCH="amd64" ./chroot_devenv.sh setup
```
to install a local copy of the chroot and install `ser2net` on the GOAT. Edit the config at `/etc/ser2net.conf` to read
```
2000:raw:600:/dev/ttyS3:115200 8DATABITS NONE 1STOPBIT
2001:raw:600:/dev/ttyS4:115200 8DATABITS NONE 1STOPBIT
```
and then on your development machine, run `socat` as
```
sudo socat pty,link=/dev/ttyM1,raw,echo=0 tcp:goat.lan:2000; sudo socat pty,link=/dev/ttyM2,raw,echo=0 tcp:goat.lan:2001
```

You can test it's working by running `mcu_parser.py /dev/ttyM1` from `/hardware_control_RE`

# Make it permanent!
Make sure to remount the root as rewritable: `mount -o remount,rw /`
Disable all the ecovacs stuff (all of this is trivially reversible, and there's a backup partition you can restore if you really mess it up):
```
systemctl disable process_monitor.service
systemctl disable roscore.service
systemctl disable ros.service
systemctl disable eco_main_tools.service
systemctl disable boot_complete.service
systemctl disable memcpu_monitor.service
systemctl disable check_sensor_type.service
systemctl disable audioDeamon.service
systemctl disable auto_ota_check.service
systemctl disable auto_ota_check.timer
systemctl disable log_clean.service
systemctl disable log_clean.timer
systemctl disable cameraProvider.service 
mv /usr/bin/hydra.sh /usr/bin/hydra.sh.bak
```
set openmower to run on startup:
```
bash -c 'cat <<EOF > /etc/systemd/system/openmower.service
[Unit]
Description=OpenMower Service
After=network.target

[Service]
Type=simple
ExecStart=/var/openmower/OpenMowerGOATO600/chroot_devenv.sh run
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF'
```
then 
```
systemctl daemon-reload && sudo systemctl enable --now openmower
```