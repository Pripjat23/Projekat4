// bfs_core.hpp
//
// Jezgro akceleratora za virtuelnu platformu: ISTI algoritam kao bfs_model.hpp,
// ali piše se kroz "port B" BRAM-a (čitanje i upis 32-bitnih riječi) i broji
// cikluse. Čist C++ (bez SystemC-a), pa se testira odvojeno od platforme.
//
// Model ciklusa (PRETPOSTAVKE, mijenjaju se u BfsTiming; vidi poglavlje 8):
//   * skeniranje: 1 ciklus po 32-bitnoj riječi slike (prazne riječi se preskaču)
//   * po nađenoj komponenti ("sjeme"): nekoliko ciklusa za početne koordinate
//   * po obiđenom pikselu: cyclesPerPixel ciklusa (uzimanje iz reda, čitanje
//     4 susjeda, brisanje bita, ažuriranje ekstrema)
#pragma once

#include <cstdint>
#include "bfs_model.hpp"   // BfsResult, BfsPoint, parametri dizajna

struct BfsTiming {
    int cyclesPerScanWord = 1;
    int cyclesPerSeed = 2;
    int cyclesPerPixel = 8;
};

struct BfsRun {
    BfsResult res;
    uint64_t scanCycles = 0;
    uint64_t seedCycles = 0;
    uint64_t pixelCycles = 0;
    uint64_t visitedPixels = 0;   // svi obiđeni svijetli pikseli (sve komponente)
    uint64_t cycles() const { return scanCycles + seedCycles + pixelCycles; }
};

// Mem mora imati: uint32_t readWord(int wordIndex); void writeWord(int wordIndex, uint32_t value);
template <class Mem>
BfsRun runBfs(Mem& mem, int w, int h, const BfsTiming& t) {
    BfsRun out;
    BfsResult best;
    const int N = w * h;
    const int nWords = (N + 31) / 32;

    uint32_t fifo[BFS_FIFO_DEPTH];
    uint32_t numComp = 0, maxFront = 0;
    bool overflow = false;

    auto getBit = [&](int idx) { return (mem.readWord(idx >> 5) >> (idx & 31)) & 1u; };
    auto clearBit = [&](int idx) {
        uint32_t v = mem.readWord(idx >> 5);
        mem.writeWord(idx >> 5, v & ~(1u << (idx & 31)));
    };

    for (int wi = 0; wi < nWords; ++wi) {
        out.scanCycles += t.cyclesPerScanWord;
        for (;;) {
            const uint32_t word = mem.readWord(wi);
            if (word == 0) break;
            int b = 0;
            while (!((word >> b) & 1u)) ++b;           // najniži postavljeni bit
            const int idx = wi * 32 + b;
            if (idx >= N) break;                       // bitovi iza kraja slike su uvijek 0

            ++numComp;
            out.seedCycles += t.cyclesPerSeed;
            const int x0 = idx % w, y0 = idx / w;      // u hardveru: brojači kolone i reda

            uint32_t head = 0, tail = 0, size = 0;
            auto push = [&](int x, int y) {
                if (size == (uint32_t)BFS_FIFO_DEPTH) { overflow = true; return; }
                clearBit(y * w + x);
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
                out.pixelCycles += t.cyclesPerPixel;
                ++out.visitedPixels;
                if (x < cur.minX) cur.minX = (uint16_t)x;
                if (x > cur.maxX) cur.maxX = (uint16_t)x;
                if (y < cur.minY) cur.minY = (uint16_t)y;
                if (y > cur.maxY) cur.maxY = (uint16_t)y;
                const int s = x + y, d = x - y;
                if (s < minS) { minS = s; cur.tl = {(uint16_t)x, (uint16_t)y}; }
                if (s > maxS) { maxS = s; cur.br = {(uint16_t)x, (uint16_t)y}; }
                if (d > maxD) { maxD = d; cur.tr = {(uint16_t)x, (uint16_t)y}; }
                if (d < minD) { minD = d; cur.bl = {(uint16_t)x, (uint16_t)y}; }

                if (x + 1 < w && getBit(y * w + x + 1))   push(x + 1, y);
                if (x > 0     && getBit(y * w + x - 1))   push(x - 1, y);
                if (y + 1 < h && getBit((y + 1) * w + x)) push(x, y + 1);
                if (y > 0     && getBit((y - 1) * w + x)) push(x, y - 1);
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
    out.res = best;
    return out;
}
