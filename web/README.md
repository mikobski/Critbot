# Interfejs graficzny strony internetowej

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