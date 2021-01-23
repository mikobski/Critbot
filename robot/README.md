# Tutorial
English at the bottom of the page.
## Wykorzystane komponenty
1. Arduino - sterowanie kołami, enkodery z kół, kontroler RC
2. Ublox M8N - GPS
3. D435, Intel Real Sense - obraz przesyłany do aplikacji
4. RP lidar S1 - wykrywanie przeszkód
5. Intel NUC i5 - jednostka centralna
## Instalacja
Aby zainstalować wymagane pakiety wystarczy użyć skryptu: 
```
./install_enviroment_robot.sh
```
Jeśli nie wystąpiły błędy, oprogramowanie jest gotowe do uruchomienia.
## Uruchamianie
Aby uruchomić oprogramowanie:
```
cd launch
roslaunch full_critbot_init.launch
```
## Basic specification
1. Arduino - steering wheels, wheels encoders, RC controller
2. Ublox M8N - GPS
3. D435, Intel Real Sense - image sent to webapp
4. RP lidar S1 - detecting obstacles
5. Intel NUC i5 - central unit
## Installation
To install required software, please run script
```
./install_enviroment_robot.sh
```
If everything went fine, you should be able to start robot software
## Running
To run robot software:
```
cd launch
roslaunch critbot_init.launch
```
