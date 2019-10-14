##Kontroler 6DoF
### Opis projektu
Projekt składa się z dwóch części: kontrolera bezprzewodowego wraz z odbiornikiem
oraz aplikacji komunikującej się z kontrolerem.
###1.1 Kontroler
Kontroler będzie umożliwiał wykrywanie swojej orientacji przestrzennej i pozycji. Ze
względu na dryf, wykrywanie pozycji będzie krótkotrwałe, a pozycja kontrolera po zakończeniu ruchu będzie sprowadzana do ustalonej pozycji początkowej. Kontroler będzie
urządzeniem bezprzewodowym i komunikującym się z odbiornikiem podłączonym do komputera.
Kontroler będzie wyposażony w:
• Mikrokontroler STM32
• Akcelerometr z żyroskopem
• Moduł radiowy
• Przyciski
• Baterię
Odbiornik będzie składał się z:
• Mikrokontroler STM32 komunikujący się z PC
• Moduł radiowy
###1.2 Aplikacja
Aplikacja będzie odbierać dane wysyłane przez kontrole i pozwalać będzie na wizualizację tego ruchu. Aplikacja zostanie napisana w języku C++ z wykorzystaniem bibliotek
wxWidgets i OpenGL.
  
  
## Technologies
Project is created with:
* Lorem version: 12.3
* Ipsum version: 2.33
* Ament library version: 999
	
## Setup
To run this project, install it locally using npm:

```
$ cd ../lorem
$ npm install
$ npm start
```
