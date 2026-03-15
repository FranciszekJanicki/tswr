# Projekt na przedmiot TSwR

**Samobalansujący robot dwukołowy**

## Opis

Celem projektu jest stabilizacja i sterowanie ruchem mobilnego robota typu odwrócone wahadło na kołach. Zadaniem robota jest utrzymanie pionowej przy jednoczesnym zachowaniu zadanej pozycji.

Konstrukcja takiego robota została już zbudowana i uruchomiona wcześniej (projekt przejściowy). Dotychczasowe działanie oparte na klasycznych regulatorach PID nie było w pełni satysfakcjonujące. Robot co prawda utrzymywał pion, ale jakość regulacji pozostawiała sporo do życzenia i występował dryft pozycji. Jeśli uda się w ramach tego przedmiotu zaprojektować lepsze sterownie (i lekkie obliczeniowo) to możemy uruchomić sterownik na prawdzimym robocie.

Zdjecie:
![Nasz Robot](doc/segway.jpg)

## Symulacja

Python + PyBullet

## Sterownik

Myśleliśmy o dwóch opcjach:
- *MPC* - może okazać się zbyt wymagający obliczeniowo dla prawdziwego robota, ale w symulacji zadziała
- *LQI* (Linear Quadratic Integral) - aby robot utrzymywał pion i nie dryfował. Zwykły LQR nie poradziłby sobie w przypadku gdy czujnik będzie krzywo umieszczony (co w praktyce jest nieuniknione).