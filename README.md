# Embedded-Systems-NXT-Desposer
Code to make an nxt robot follow a map and dispose of hazardous waste

## How to use this thing for you ding dongs

1. Write the code for your .nxc
2. Type `sh compilenbc.sh [name_of_your_nxc_file_minus_the_nxc_extension]`

Example:

`sh compilenbc.sh move_on_colour`

In case this doesn't work make sure you have the bashrc alias:

`alias nbc='export LD_LIBRARY_PATH=./; ./nbc'`
