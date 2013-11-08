#! /bin/sh

JLinkExe << EOF
exec device = lpc1758
speed auto
halt
loadbin nuttx.bin,0x00001000
reset
go
exit
EOF

