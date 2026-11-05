#!/usr/bin/env bash
# run_tests.sh – pokreće ./card nad svim slikama u folderu i računa tačnost.
#
# Imena slika određuju očekivani rezultat:  <rank>_<suit>[_bilo_sta].jpg
#   npr.  ace_spades_1.jpg   ->  "ace spades"
#         ten_hearts_kos.jpeg ->  "ten hearts"
# rank: two three four five six seven eight nine ten jack queen king ace
# suit: hearts diamonds clubs spades
#
# Upotreba:  ./run_tests.sh [folder]      (podrazumijevano: tests)

DIR="${1:-tests}"
BIN="./card"

if [ ! -x "$BIN" ]; then echo "Nema $BIN – prvo prevedi: g++ -std=c++17 -O2 main.cpp -o card"; exit 2; fi
if [ ! -d "$DIR" ]; then echo "Nema foldera $DIR"; exit 2; fi

total=0; ok=0; fail=0
for f in "$DIR"/*.jpg "$DIR"/*.jpeg "$DIR"/*.png; do
    [ -e "$f" ] || continue
    base=$(basename "$f"); name="${base%.*}"
    rank=$(echo "$name" | cut -d_ -f1)
    suit=$(echo "$name" | cut -d_ -f2)
    expected="$rank $suit"

    got=$("$BIN" "$f" 2>/dev/null)
    total=$((total+1))
    if [ "$got" == "$expected" ]; then
        ok=$((ok+1))
    else
        fail=$((fail+1))
        echo "GRESKA  $base   ocekivano: '$expected'   dobijeno: '${got:-<nista>}'"
    fi
done

if [ "$total" -eq 0 ]; then echo "Nema slika u $DIR"; exit 2; fi
echo "-----------------------------------------"
echo "Tacno: $ok / $total   ($((100*ok/total))%)"
[ "$fail" -eq 0 ]
