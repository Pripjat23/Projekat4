// main.cpp
//
// "Testbench" strana: čita sliku i šablone sa diska i ispisuje rezultat.
// Sav algoritam je u card_algo.hpp.
//
// Prevođenje:   g++ -std=c++17 -O2 main.cpp -o card
// (potreban je samo stb_image.h u istom folderu)
//
// Pokretanje:   ./card                    -> obrađuje karta.jpeg
//               CARD_VERBOSE=1 ./card x.jpg  -> dodatno ispisuje score-ove (manje = bolje)
//               ./card a.jpg b.jpg ...    -> obrađuje sve navedene slike
//
// Šabloni se očekuju u: Card_Imgs/Ranks/*.jpg i Card_Imgs/Suits/*.jpg

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "card_algo.hpp"

#include <cstdio>
#include <cstdlib>

// ─── Učitavanje šablona (jednom, na početku) ─────────────────────────────────

static bool loadTemplate(const char* path, int id, Template& t) {
    int w, h, ch;
    uint8_t* raw = stbi_load(path, &w, &h, &ch, 0);
    if (!raw) {
        std::fprintf(stderr, "Ne mogu da ucitam sablon: %s\n", path);
        return false;
    }
    if (w * h > TPL_MAX) {
        std::fprintf(stderr, "Sablon je prevelik (%dx%d): %s\n", w, h, path);
        stbi_image_free(raw);
        return false;
    }
    t.id = id;
    t.w = w;
    t.h = h;
    binarizeRaw(raw, t.bin, w, h, ch);
    stbi_image_free(raw);
    return true;
}

static bool loadTemplates(const char* dir, TemplateSet& set) {
    // redoslijed odgovara id-evima 1..13 (two..ace); "0" je desetka (vidi matchSymbol)
    static const char* rankFiles[NUM_RANKS] = {
        "2", "3", "4", "5", "6", "7", "8", "9", "0", "jack", "queen", "king", "ace"};
    static const char* suitFiles[NUM_SUITS] = {"hearts", "diamonds", "clubs", "spades"};

    char path[512];
    for (int i = 0; i < NUM_RANKS; ++i) {
        std::snprintf(path, sizeof(path), "%s/Ranks/%s.jpg", dir, rankFiles[i]);
        if (!loadTemplate(path, i + 1, set.ranks[i])) return false;
    }
    for (int i = 0; i < NUM_SUITS; ++i) {
        std::snprintf(path, sizeof(path), "%s/Suits/%s.jpg", dir, suitFiles[i]);
        if (!loadTemplate(path, i, set.suits[i])) return false;
    }
    return true;
}

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    static TemplateSet templates;
    if (!loadTemplates("Card_Imgs", templates)) return 2;

    const int nImages = (argc > 1) ? argc - 1 : 1;
    int failures = 0;

    for (int i = 0; i < nImages; ++i) {
        const char* path = (argc > 1) ? argv[i + 1] : "karta.jpeg";

        int w, h, ch;
        uint8_t* image = stbi_load(path, &w, &h, &ch, 3); // uvijek RGB
        if (!image) {
            std::fprintf(stderr, "%s: ne mogu da ucitam sliku\n", path);
            ++failures;
            continue;
        }

        CardResult res;
        Status st = recognizeCard(image, w, h, templates, res);
        stbi_image_free(image);

        if (st != OK) {
            std::fprintf(stderr, "%s: %s\n", path, statusText(st));
            ++failures;
            continue;
        }

        if (nImages > 1) std::printf("%s: ", path);
        std::printf("%s %s", rankName(res.rank), suitName(res.suit));
        if (std::getenv("CARD_VERBOSE"))
            std::printf("   (rank score %.3f, suit score %.3f)", res.rankScore, res.suitScore);
        std::printf("\n");
    }
    return failures ? 1 : 0;
}
