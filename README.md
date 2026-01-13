# C2S-Official
This repository broadly will cover all the files involved in C2S DIRV 2025

(SoC Bitstreams - https://drive.google.com/drive/u/1/folders/11GrKphslTaAf7Hmy3RfOlaYtl2LhL6Qx)

To access PuTty on Ubuntu do - GDK_BACKEND=x11 putty at 115200 after verifying at /dev/ttyUSB0/1


for me :

    in AMD Xilinx i'm NOT using my clg mail
    


download riscv32 toolchain first!!

    sudo apt update
    sudo apt install gcc-riscv32-linux-gnu


    riscv32-linux-gnu-gcc -static -march=rv32ima -mabi=ilp32 arducam_capture.c -o arducam_capture
    
    Target: riscv32-linux-gnu
    C library: glibc
    ABI: ilp32
    ARCH: rv32ima





