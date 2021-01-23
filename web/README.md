# Aplikacja operatora
English at the bottom
## Instalacja Node.js i NPM
### 
#### Linux (Ubuntu 18.04)
Uruchom `./install_environment_web.sh`.
#### Windows 10
Ręczna instalacja Node.js (zalecana wersja 14.15 lub nowsza).

## Budowanie i uruchomienie
#### Wersja deweloperska
Wykonaj w folderze *web/*
* `npm install` - dociąganie zależności
* `npm start` - budowanie (ze śledzeniem zmian na żywo) i uruchomienie serwera pod adresem `http://localhost:3000/`

#### Budowanie zoptymalizowanej wersji
Wykonaj w folderze *web/*
* `npm install` - dociąganie zależności
* `npm build` - budowanie

## Installation (Node.js and NPM)
### 
#### Linux (Ubuntu 18.04)
Run `./install_environment_web.sh` script.
#### Windows 10
Install Node.js from offical website. Required version is 14.15 or higher.

## Building app and running dev server
#### Dev version
Run in *web/* directory
* `npm install` - installing dependecies
* `npm start` - building app and running HTTP server at `http://localhost:3000/`. This command also watch files and automatically build app after changes.

#### Building optimized app
Run in *web/* directory
* `npm install` - installing dependecies
* `npm build` - building app

