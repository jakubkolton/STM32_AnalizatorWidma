 # Analizator widma na STM32 - OPIS PROJEKTU
 
  	Wersja projektu: v 0.1 (ROBOCZA)
    
Sygnał podany na ADC dołączone do DMA poddany jest FFT, po czym wyświetlony na ekranie OLED.

Trwają prace nad rozszerzeniami, m.in. zapisem zrzutów ekranu i informacji o widmie do plików na karcie SD; interfejsem sterowanym joystickiem; wykorzystaniem zjawiska aliasingu do badania sygnałów wąskopasmowych.

## Schemat układu:
![](./AnalizatorWidma.png?raw=true "Schemat układu")

## Zastosowane elementy:

- Nucleo STM32F411RE / STM32L476
- Ekran OLED 128x64 z kontrolerem SSD1306
- Joystick
- Czytnik kart SD

## Wykorzystano zewnętrzne biblioteki: 

- CMSIS DSP: 
    - https://github.com/ARM-software/CMSIS_5;
    - wersja z dnia: 21.03.2021
- Adafruit_SSD1306:
    - Jest to biblioteka pisana pod Arduino. Wykonano jej częściowy port.
    - https://github.com/adafruit/Adafruit_SSD1306;
    - wersja z dnia: 21.03.2021
 - Adafruit_GFX:
    - Jest to biblioteka pisana pod Arduino. Wykonano jej częściowy port.
    - https://github.com/adafruit/Adafruit_SSD1306;
    - wersja z dnia: 21.03.2021   



## HISTORIA ZMIAN: 

#### v 0.1:
- Wersja robocza programu.
- FFT sygnału próbkowanego przez ADC i podawanego do DMA.
- Wyświetlanie widma amplitudowego na ekranie.
- Planowane rozszerzenie o:
  - Poprawa dziłania FFT.
  - Zapis zrzutów ekranu i informacji o widmie na kartę SD.
  - Interfejs i jego obsługa z użyciem joysticka.
  - Log wysyłany przez UART.
  - Sterowanie parametrami FFT (rozdzielczość, pasmo).
  - Eliminacja aliasingu i wykorzystanie go do analizy sygnałów wąskopasmowych.
