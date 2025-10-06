# 5DOF-Robotic-Arm
Sterowanie Manipulatorem Robotycznym z Wykorzystaniem Operacji Macierzowych i Programowania Sekwencyjnego Ruchów - Robotic Manipulator Control Using Matrix Operations and Sequential Motion Programming

Komunikacja I2C: 
Początkowym problemem okazał się fakt, że enkodery magnetyczne AS5600 mają ten sam adres, który jest niemożliwy do zmiany (0x36). Biorąc pod uwagę, że ESP-32 posiada tylko dwa porty I2C niezbędny okazał się zakup multipleksera, który umożliwi komunikację z wieloma urządzeniami I2C, także posiadającymi ten sam adres. Do każdego wejścia oraz wyjścia komunikacji I2C dodałem rezystory pull-up 4,7kOhm w celu stabilizacji połączenia.
Po podpięciu urządzeń przystąpiłem do programowania. Korzystałem z biblioteki Wire.h umożliwiającej poprawną komunikację I2C. 

Komunikacja UART:

Operacje związane z pozycją silników: