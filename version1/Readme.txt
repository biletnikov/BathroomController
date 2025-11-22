Фьзы на продакшн + 8 Mhz кристалл, Brown Out detection 4.3 V, разрешаем стирать EEPROM при перепрошивке
avrdude.exe -p m88 -c usbasp -U lfuse:w:0xdc:m -U hfuse:w:0xd4:m -U efuse:w:0xf9:m

Заливаем прошивку
avrdude.exe -p m88 -c usbasp -U eeprom:w:BathroomController.eep
avrdude.exe -p m88 -c usbasp -U flash:w:BathroomController.hex
