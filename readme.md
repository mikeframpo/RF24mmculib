
* In the boards/ directory, copy example/ files into a new directory.
* In the boards/ directory, open the board.mk file and set the BOARD variable to the name of the new directory. Also set the MCU and MMCULIB_DIR.
* Define the board's pin connections in target.h
* In the examples/pipe directory, run `make` and then `make program` to load the program.

