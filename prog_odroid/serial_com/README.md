# Librairies

## serial_lib.cpp/.h
Contains all useful ressources for serial communication

## types_convert.cpp/.h
Contains all useful types convertion

# Executable

## serial_gum_test (serial_gum_test.cpp)
Execute to test communication with Gumstix.\n
To test Gumstix :
1) Plug Gumstix RX/TX on odroid CON10 (pins 6/8).
2) Update and run COM_test_gumstix on target.
3) Run serial_gum_test.


## serial_test_example (serial_test.cpp)
Execute Write/Read on Serial Port /dev/ttySAC0.
1) Plug RX on TX CON10 (pins 6/8).
2) Run serial_test_example.
