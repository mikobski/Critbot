# Interfejs graficzny strony internetowej

## Instalacja Node.js i NPM
#### Linux
Uruchom `./install_environment_web.sh`.
#### Windows
Zainstaluj z linku dla [x64](https://nodejs.org/dist/latest-v10.x/node-v10.22.0-x64.msi) lub [x86](https://nodejs.org/dist/latest-v10.x/node-v10.22.0-x86.msi).

Może być konieczne ponowne odpalenie konsoli lub reset, aby zaciągnęło nowe zmienne środowiskowe. Dla sprawdzenia można uruchomić komendy:
* `node -v` powinna wypluć `v10.22.0`
*  `npm -v` powinna wypluć `6.14.6`

## Budowanie Webviz
Uruchom w folderze *web/*:
* `npm run bootstrap` - dociąganie zależności
* `npm run build` - budowanie pakietów
* `npm run build-static-webviz` - budowanie plików strony internetowej
* `npm run serve-static-webviz` - uruchomienie testowego serwera

Wszystkie powyższe komendy są konieczne do wykonania w celu uruchomienia Webviz. Oczywiście pierwsze 3 komendy trzeba uruchomić tylko w przypadku zmian w kodzie strony.
* `npm run clean` - usuwanie zbudowanych plików

## Używanie Webviz
Domyślnie Webviz jest dostępny pod adresem `http://localhost:8080/`, a źródło danych to `ws://localhost:9090`, więc ma taki sam port jak domyślny dla uruchomionego *ros_bridge*.