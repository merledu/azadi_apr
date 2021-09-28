li s0, 0x40060000
li a0, 348
li a2, 1
sw a0, 0(s0)
sw a2, 12(s0)
li a4, 1
loop:
lw a3, 20(s0)
beq a4, a3, exit
jal loop
exit:
lw a5, 8(s0)
sw a2, 16(s0)
sw a5, 4(s0)
add a6, a5, a5
sw x0, 24(s0)
rep:
jal rep


