#!/bin/bash

sudo mkdir -p /lib/firmware/brcm
cd /lib/firmware/brcm
sudo wget https://raw.githubusercontent.com/qutas/info/master/Files/brcmfmac43455-sdio.txt
sudo wget https://raw.githubusercontent.com/qutas/info/master/Files/brcmfmac43455-sdio.bin
sudo wget https://raw.githubusercontent.com/qutas/info/master/Files/brcmfmac43455-sdio.clm_blob
