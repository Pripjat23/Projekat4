// downscale.hpp
//
// Umanjenje slike (box filter: prosjek f x f piksela) za eksperimente sa manjom
// rezolucijom, tj. sa slikom koja staje u BRAM. Strana testbench-a, nije dio
// algoritma. Faktor se zadaje promjenljivom okruženja CARD_DOWNSCALE ili
// CARD_MAX_SIDE (vidi downscaleFactorFor).
#pragma once

#include <cstdint>
#include <cstdlib>
#include <vector>

// CARD_DOWNSCALE=4 -> 4; bez promjenljive -> 1 (bez umanjenja)
inline int downscaleFactorFromEnv() {
    const char* e = std::getenv("CARD_DOWNSCALE");
    int f = e ? std::atoi(e) : 1;
    return f < 1 ? 1 : f;
}

// Faktor umanjenja za konkretnu sliku:
//   CARD_MAX_SIDE=512 -> najmanji cio faktor za koji je duža strana <= 512
//                        (slike različitih rezolucija završe na sličnoj veličini)
//   CARD_DOWNSCALE=4  -> isti faktor za sve slike
//   bez promjenljivih -> 1
inline int downscaleFactorFor(int w, int h) {
    const char* m = std::getenv("CARD_MAX_SIDE");
    if (m) {
        int side = std::atoi(m);
        if (side >= 16) {
            int longest = w > h ? w : h;
            return (longest + side - 1) / side;   // zaokruživanje naviše
        }
    }
    return downscaleFactorFromEnv();
}

// RGB (3 bajta/piksel) -> RGB umanjeno f puta po svakoj osi. Vraća false ako bi
// slika postala premala (manje od 16 piksela u nekoj dimenziji).
inline bool downscaleBox(const uint8_t* src, int w, int h, int f,
                         std::vector<uint8_t>& dst, int& ow, int& oh) {
    ow = w / f;
    oh = h / f;
    if (f < 1 || ow < 16 || oh < 16) return false;
    dst.resize((size_t)ow * oh * 3);
    const int area = f * f;
    for (int y = 0; y < oh; ++y)
        for (int x = 0; x < ow; ++x)
            for (int c = 0; c < 3; ++c) {
                int sum = 0;
                for (int j = 0; j < f; ++j) {
                    const uint8_t* row = src + ((size_t)(y * f + j) * w + (size_t)x * f) * 3 + c;
                    for (int i = 0; i < f; ++i) sum += row[i * 3];
                }
                dst[((size_t)y * ow + x) * 3 + c] = (uint8_t)((sum + area / 2) / area);
            }
    return true;
}
