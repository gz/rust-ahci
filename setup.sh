#!/bin/bash
cd $HOME
curl https://sh.rustup.rs -sSf | sh -s -- --default-toolchain nightly -y
source $HOME/.cargo/env
modprobe -r ahci
sudo modprobe uio
sudo modprobe uio_pci_generic
#babybel:
echo "0x8086 0x1d02" > /sys/bus/pci/drivers/uio_pci_generic/new_id
lspci -v -d :0x1d02 | grep "Kernel driver in use"

#tilsiter:
echo "0x8086 0xa102" > /sys/bus/pci/drivers/uio_pci_generic/new_id
lspci -v -d :0xa102 | grep "Kernel driver in use"

cd ahci
rustup override add nightly
echo 20 >/proc/sys/vm/nr_hugepages_mempolicy
echo 4 > /sys/kernel/mm/hugepages/hugepages-1048576kB/nr_hugepages_mempolicy
cargo update
cargo build
