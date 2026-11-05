# Prepoznavanje Poker Karata – SystemC/TLM Virtuelna Platforma

Ovaj projekat predstavlja SystemC/TLM 2.0 virtuelnu platformu za prepoznavanje poker karata sa fotografija, zasnovanu na particionisanju hardvera i softvera za Zynq-7000 SoC (ARM procesor + FPGA akcelerator).

## 📁 Pregled fajlova i direktorijuma u projektu

### Izvorni kod i zaglavlja (`.cpp`, `.hpp`, `.h`)
* **`card_algo.hpp`** – Referentni (zlatni) model algoritma implementiran u C++-u (`recognizeCard`). Služi kao osnovni kriterijum tačnosti.
* **`bfs_model.hpp`** – Bitski tačan model BFS akceleratora koji simulira hardverska ograničenja (10-bitne koordinate, 20-bitne adrese, FIFO red dubine 1024).
* **`bfs_core.hpp`** – Funkcionalno jezgro BFS akceleratora na virtuelnoj platformi sa proračunom ciklusa izvršavanja i vremenskim modelom.
* **`vp_platform.hpp`** – Definicija modula virtuelne platforme (BRAM memorija, magistrala i BFS akcelerator registri/interfejs).
* **`vp_main.cpp`** – Glavni fajl virtuelne platforme koji sadrži testbench (`Cpu` modul) i `sc_main`.
* **`main.cpp`** – Glavni ulazni fajl za samostalno pokretanje softverskog referentnog modela.
* **`template_loader.hpp`** – Modul za učitavanje i pripremu šablona karata (13 za rank i 4 za suit).
* **`downscale.hpp`** – Modul za skaliranje ulaznih slika (smanjivanje blokovskim prosekom na maksimalnu dimenziju).
* **`stb_image.h` / `stb_image_write.h`** – Biblioteke za učitavanje i čuvanje slika u različitim formatima (JPG, PNG).

### Podaci, skripte i izvršni fajlovi
* **`Card_Imgs/`** – Folder sa šablonima karata za matching (rankovi i suitovi).
* **`tests/` / `tests1/`** – Folderi sa test slikama karata različitih rezolucija, osvetljenja i rotacija.
* **`ace_diamonds.jpg`** – Primer pojedinačne test slike karte.
* **`run_tests.sh`** – Skripta za automatsko pokretanje regresionih testova nad test skupom slika.
* **`Makefile`** – Skripta za prevođenje i kompajliranje SystemC virtuelne platforme.
* **`vp`** – Kompajlirani izvršni fajl virtuelne platforme.
* **`stb-master/`** – Eksterni repozitorijum sa pomoćnim `stb` bibliotekama.
* **`Prepoznavanje poker karata – specifikacija sistema.docx`** – Kompletan dokument sa specifikacijom sistema.
* **`readme_md.md`** – Lokalna dokumentacija projekta.

---

## 🛠️ Kompajliranje i pokretanje

### 1. Kompajliranje projekta

Virtuelna platforma se kompajlira pomoću `make` alata uz obavezno navođenje putanje do instaliranog SystemC okruženja (`SYSTEMC_HOME`):

```bash
make SYSTEMC_HOME=/putanja/do/systemc
```

### 2. Pokretanje virtuelne platforme

Zbog hardverskih ograničenja akceleratora (10-bitne koordinate, maksimalno $1024 \times 1024$ piksela i BRAM kapacitet od 270 KB / 240 KB iskoristivo u 1-bitnom radu), ulazne slike je potrebno skalirati pre slanja akceleratoru. To se postiže postavljanjem promenljive okruženja `CARD_MAX_SIDE=1024`.

#### **A) Pokretanje za pojedinačnu sliku**

Ukoliko želite da pokrenete obradu samo jedne određene slike (npr. `ace_diamonds.jpg` ili slika iz test foldera):

```bash
CARD_MAX_SIDE=1024 ./vp ace_diamonds.jpg
```

Možete kombinovati i sa flagovima za prenos/rad:
```bash
CARD_MAX_SIDE=1024 ./vp tests/ace_spades.jpg --burst
```

#### **B) Pokretanje nad skupom test slika**

* **Osnovni režim (slika se prenosi reč-po-reč, prekidni rad):**
  ```bash
  CARD_MAX_SIDE=1024 ./vp tests/*
  ```

* **Paketni režim (`--burst`):**
  Koristi burst/DMA prenos binarne slike u BRAM u paketima (podrazumevano 1 KB paketi). Ovo znatno ubrzava prenos u odnosu na rad reč-po-reč (sa ~4.87 ms na ~0.26 ms):
  ```bash
  CARD_MAX_SIDE=1024 ./vp tests/* --burst
  ```
  *(Opciono: `--burst=512` za definisanje veličine paketa u bajtovima).*

* **Režim prozivanja (`--poll`):**
  Softver periodično proziva statusni registar akceleratora (`STATUS.BUSY` / `STATUS.DONE`) umesto korišćenja prekida:
  ```bash
  CARD_MAX_SIDE=1024 ./vp tests/* --poll
  ```

---

## 💡 Zašto je obavezno koristiti `CARD_MAX_SIDE=1024`?

1. **Ograničenje registara akceleratora:** Koordinate u akceleratoru su 10-bitne ($0 - 1023$), a adresa u memoriji je 20-bitna, pa slika ne sme prelaziti $1024 \times 1024$ piksela.
2. **Kapacitet BRAM-a (270 KB):** Zynq XC7Z010 čip raspolaže sa 60 BRAM blokova (270 KB ukupno). U 1-bitnom načinu rada iskoristivo je 240 KB. Binarna slika dimenzija $768 \times 1024$ zauzima oko 96 KB, dok bi slika visoke rezolucije (npr. 12 MP) zauzela preko 1.5 MB i ne bi stala u BRAM.
3. **Tačnost obrade:** Skaliranje održava visoku tačnost algoritma ($12/15$ test slika) uz značajno smanjenje potrebnog vremena obrade i memorije.