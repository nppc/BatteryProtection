cd C:\src\Temp\arduino-1.8.2\hardware\tools\avr\bin\
avrdude.exe -c usbasp -p t13 -P usb -B 4.0 -U flash:w:C:\src\Temp\git\BatteryProtection\BatProtFirmware\BatProtFirmware.hex:a
pause