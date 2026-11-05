// template_loader.hpp
//
// Učitavanje šablona sa diska. Mora se uključiti POSLIJE stb_image.h
// (zajedničko za main.cpp i profile.cpp). Algoritam (card_algo.hpp) ne zna
// ništa o disku.
#pragma once

#include <cstdio>
#include "card_algo.hpp"

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
