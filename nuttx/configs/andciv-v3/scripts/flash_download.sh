#! /bin/sh

JLinkExe << EOF
exec EnableFlashDL
exec Device=LPC1758
speed auto
halt
loadbin nuttx.bin 0x00001000
reset
go
exit
EOF

