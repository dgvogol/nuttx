#! /bin/sh

JLinkExe << EOF
exec device = lpc4357_m4
speed 4000
h
loadbin nuttx.bin,0x1a000000
r
g
q
EOF

