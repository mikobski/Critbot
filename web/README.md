# Interfejs graficzny strony internetowej

## Budowanie Webviz
Uruchom w folderze *web/*
`npm run bootstrap` Dociąganie zależności
`npm run build` Budowanie pakietów
`npm run build-static-webviz` Budowanie plików strony internetowej
`npm run serve-static-webviz` Uruchomienie testowego serwera
Wszystkie powyższe komendy są konieczne do wykonania w celu uruchomienia Webviz. Oczywiście pierwsze 3 komendy trzeba uruchomić tylko w przypadku zmian w kodzie strony.
`npm run clean` Usuwanie zbudowanych plików

## Używanie Webviz
Domyślnie Webviz jest dostępny pod adresem *http://localhost:8080/*, a źródło danych to *ws://localhost:9090*, więc takie samo jak domyślny port dla uruchomionego *ros_bridge*.