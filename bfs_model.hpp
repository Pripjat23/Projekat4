// bfs_model.hpp
//
// BITSKI TAČAN MODEL hardverskog BFS akceleratora (opcija 1).
//
// Ovo je rezultat bitske analize: isti algoritam kao findLargestComponent u
// card_algo.hpp, ali napisan onako kako bi ga radio hardver:
//   * ulaz je binarna slika, 1 bit po pikselu, u BRAM-u (upisuje je PS)
//   * nema posebnog "visited" niza: čim se piksel stavi u red, bit u slici se
//     briše (slika se uništava; PS ima svoju sivu sliku za warp)
//   * red je kružni FIFO fiksne dubine, zapis = {y, x}, svaki 10 bita
//   * koordinate su 10-bitne, brojač piksela 21-bitni
//   * nema dijeljenja ni pokretnog zareza
// Rezultat mora biti IDENTIČAN referentnom modelu (check_bfs_model.cpp to provjerava).
#pragma once

#include <cstdint>
#include <vector>
#include "card_algo.hpp"

// ─── Parametri dizajna (iz bitske analize) ───────────────────────────────────
constexpr int BFS_COORD_BITS = 10;                     // x, y: 0..1023
constexpr int BFS_MAX_DIM    = 1 << BFS_COORD_BITS;    // najveća širina/visina
constexpr int BFS_MAX_PIXELS = 1 << 20;                // 20-bitna adresa bita
constexpr int BFS_COUNT_BITS = 21;                     // broj piksela komponente (do 2^20)
constexpr int BFS_SUM_BITS   = 11;                     // x+y (0..2046), x-y (-1023..1023, sa predznakom)
constexpr int BFS_FIFO_DEPTH = 1024;                   // 10-bitni pokazivači
constexpr int BFS_FIFO_WIDTH = 2 * BFS_COORD_BITS;     // 20 bita po zapisu

struct BfsPoint { uint16_t x = 0, y = 0; };

struct BfsResult {
    bool ok = false;          // pronađena barem jedna komponenta
    bool overflow = false;    // FIFO se prepunio: rezultat NIJE važeći
    uint32_t count = 0;       // broj piksela najveće komponente
    uint16_t minX = 0, maxX = 0, minY = 0, maxY = 0;
    BfsPoint tl, tr, br, bl;  // uglovi
    uint32_t maxFrontier = 0; // najveća zauzetost FIFO-a (dubina koja je zaista trebala)
    uint32_t numComponents = 0;
};

class BfsAccelerator {
public:
    // PS strana: upis binarne slike u BRAM (piksel == 255 -> bit 1). false ako ne staje.
    bool writeImage(const uint8_t* binary255, int w, int h) {
        if (w <= 0 || h <= 0 || w > BFS_MAX_DIM || h > BFS_MAX_DIM || w * h > BFS_MAX_PIXELS)
            return false;
        w_ = w; h_ = h;
        words_.assign((size_t)(w * h + 31) / 32, 0u);
        for (int i = 0; i < w * h; ++i)
            if (binary255[i] == 255) words_[i >> 5] |= 1u << (i & 31);
        return true;
    }

    // Veličina BRAM-a koju slika zauzima (bajtova), bez reda.
    size_t imageBytes() const { return words_.size() * 4; }

    // Pokretanje: skenira slike red po red; za svaku nađenu komponentu radi BFS i
    // pamti najveću (strogo veću, kao referentni model: pri jednakosti ostaje prva).
    BfsResult run() {
        BfsResult best;
        if (w_ == 0) return best;
        uint32_t fifo[BFS_FIFO_DEPTH];               // zapis: y[19:10], x[9:0]
        uint32_t numComp = 0, maxFront = 0;
        bool overflow = false;

        for (int y0 = 0; y0 < h_; ++y0) {
            for (int x0 = 0; x0 < w_; ++x0) {
                if (!bit(y0 * w_ + x0)) continue;
                ++numComp;

                uint32_t head = 0, tail = 0, size = 0;   // 10-bitni pokazivači, zauzetost
                auto push = [&](int x, int y) {
                    if (size == (uint32_t)BFS_FIFO_DEPTH) { overflow = true; return; }
                    clearBit(y * w_ + x);                // označi kao "obiđen" čim uđe u red
                    fifo[tail] = ((uint32_t)y << BFS_COORD_BITS) | (uint32_t)x;
                    tail = (tail + 1) & (BFS_FIFO_DEPTH - 1);
                    ++size;
                    if (size > maxFront) maxFront = size;
                };

                BfsResult cur;
                cur.minX = 0xFFFF; cur.minY = 0xFFFF; cur.maxX = 0; cur.maxY = 0;
                int minS = INT_MAX, maxS = INT_MIN, minD = INT_MAX, maxD = INT_MIN;

                push(x0, y0);
                while (size > 0) {
                    const uint32_t e = fifo[head];
                    head = (head + 1) & (BFS_FIFO_DEPTH - 1);
                    --size;
                    const int x = (int)(e & (BFS_MAX_DIM - 1));
                    const int y = (int)(e >> BFS_COORD_BITS);

                    ++cur.count;
                    if (x < cur.minX) cur.minX = (uint16_t)x;
                    if (x > cur.maxX) cur.maxX = (uint16_t)x;
                    if (y < cur.minY) cur.minY = (uint16_t)y;
                    if (y > cur.maxY) cur.maxY = (uint16_t)y;
                    const int s = x + y, d = x - y;
                    if (s < minS) { minS = s; cur.tl = {(uint16_t)x, (uint16_t)y}; }
                    if (s > maxS) { maxS = s; cur.br = {(uint16_t)x, (uint16_t)y}; }
                    if (d > maxD) { maxD = d; cur.tr = {(uint16_t)x, (uint16_t)y}; }
                    if (d < minD) { minD = d; cur.bl = {(uint16_t)x, (uint16_t)y}; }

                    // susjedi istim redom kao u referentnom modelu: desno, lijevo, dolje, gore
                    if (x + 1 < w_ && bit(y * w_ + x + 1)) push(x + 1, y);
                    if (x > 0      && bit(y * w_ + x - 1)) push(x - 1, y);
                    if (y + 1 < h_ && bit((y + 1) * w_ + x)) push(x, y + 1);
                    if (y > 0      && bit((y - 1) * w_ + x)) push(x, y - 1);
                }

                if (cur.count > best.count) {
                    best.count = cur.count;
                    best.minX = cur.minX; best.maxX = cur.maxX;
                    best.minY = cur.minY; best.maxY = cur.maxY;
                    best.tl = cur.tl; best.tr = cur.tr; best.br = cur.br; best.bl = cur.bl;
                }
            }
        }
        best.ok = best.count > 0;
        best.overflow = overflow;
        best.maxFrontier = maxFront;
        best.numComponents = numComp;
        return best;
    }

private:
    bool bit(int idx) const { return (words_[idx >> 5] >> (idx & 31)) & 1u; }
    void clearBit(int idx) { words_[idx >> 5] &= ~(1u << (idx & 31)); }

    std::vector<uint32_t> words_;   // BRAM: 32 bita po riječi, piksel idx = bit (idx & 31) riječi idx >> 5
    int w_ = 0, h_ = 0;
};
