#!/bin/bash

cd ./Release
rm -v *.elf *.list *.map

cd ../Extended_debug
rm -v *.elf *.list *.map 

cd ../Debug
rm -v *.elf *.list *.map 