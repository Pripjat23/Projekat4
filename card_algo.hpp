// card_algo.hpp
//
// Algoritam za prepoznavanje poker karte (rank + suit) – FUNKCIONALNI MODEL.
//
// Pravila ovog fajla (važna za kasnije profinjavanje u SystemC-u):
//   * nema čitanja sa diska i nema stb biblioteke (to radi samo main.cpp)
//   * nema new/malloc, nema STL kontejnera u algoritmu (samo statički baferi)
//   * jedina ulazna tačka je recognizeCard()
//
// Poređenje sa originalnom verzijom – šta je ispravljeno:
//   1. findLargestComponent: ranije je vraćao tačke ZADNJE obrađene regije
//      (regionBuf se prepisivao). Sada se tokom BFS-a prati samo ono što je
//      potrebno (broj piksela, bounding box, 4 ugla) i čuva se NAJVEĆA regija.
//      Usput nestaju tri bafera od po ~134 MB.
//   2. Uglovi: tr i bl su bili zamijenjeni, pa je warp bio ogledalski okrenut
//      i morao se "popravljati" horizontalnim flipom. Sada su uglovi tačni,
//      a flip je izbačen.
//   3. Šabloni se ne čitaju iz matchera (ranije: stbi_load pri svakom pozivu);
//      dobijaju se kroz TemplateSet. Nedostajući šablon više ne "nestaje"
//      tiho, nego se to javlja pri učitavanju (main.cpp).
//   4. Bounding box simbola: širina/visina sada uključuju i zadnji piksel (+1).
//   5. rankMatcher i matchSuit imali su isti kod – sada je jedna funkcija
//      matchSymbol().
//   6. binarizeRaw: za slike sa 2 kanala (siva + alfa) više se alfa ne računa
//      kao zelena komponenta.
#pragma once

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>

// ─── Konstante ───────────────────────────────────────────────────────────────

constexpr int MAX_W = 4096;
constexpr int MAX_H = 4096;
constexpr int MAX_PIXELS = MAX_W * MAX_H;   // najveća ulazna slika

constexpr int CARD_W = 200, CARD_H = 300;   // veličina izravnate karte
constexpr int CORNER_W = 33, CORNER_H = 90; // gornji lijevi ugao karte
constexpr int SYM_MAX = CORNER_W * CORNER_H; // najveći isječak simbola (uvijek iz ugla karte)
constexpr int TPL_MAX = 128 * 128;          // najveći šablon (tvoji su 70x125 i 70x100)

constexpr int NUM_RANKS = 13;
constexpr int NUM_SUITS = 4;

constexpr int THRESH_CARD   = 120; // binarizacija cijele slike
constexpr int THRESH_CORNER = 100; // binarizacija ugla karte
constexpr int THRESH_SYMBOL = 120; // binarizacija rank/suit isječaka
constexpr int MIN_CARD_PIXELS = 100;

// Varijanta B (strujna lokalizacija karte, bez BFS-a) – uključuje se prevodom:
//   g++ ... -DCARD_LOCALIZE_STREAMING
// Piksel se računa tek kad je u svom redu već minRun uzastopnih svijetlih
// piksela, gdje je minRun = max(STREAM_MIN_RUN, širina / STREAM_RUN_DIV).
// Tako se ignorišu mrlje i odsjaji kraći od ~1% širine slike, nezavisno od
// rezolucije (u hardveru: jedan brojač i jedan komparator).
#ifndef STREAM_MIN_RUN
#define STREAM_MIN_RUN 4          // najmanja dužina niza, u pikselima
#endif
#ifndef STREAM_RUN_DIV
#define STREAM_RUN_DIV 100        // ... ili širina slike / STREAM_RUN_DIV (šta je veće)
#endif

// Margina (u pikselima) oko simbola prije poređenja sa šablonom. Ako su tvoji
// šabloni "tijesno" izrezani oko simbola, treba SYMBOL_PAD = 0 (to je podrazumijevano;
// na test skupu daje 12/15, a sa 2 bitno manje). Mijenja se prevodom: -DSYMBOL_PAD=2
#ifndef SYMBOL_PAD
#define SYMBOL_PAD 0
#endif

// ─── Tipovi ──────────────────────────────────────────────────────────────────

struct Point2f { float x, y; };

// Rezultat BFS-a: sve što treba za uglove karte i za bounding box simbola.
struct Component {
    int count = 0;
    int minX = 0, maxX = 0, minY = 0, maxY = 0;
    Point2f tl{}, tr{}, br{}, bl{};
};

struct Template {
    int id = -1;
    int w = 0, h = 0;
    uint8_t bin[TPL_MAX];   // binarizovan šablon: simbol = 255
};

struct TemplateSet {
    Template ranks[NUM_RANKS]; // id 1..13 (two..ace)
    Template suits[NUM_SUITS]; // id 0..3 (hearts, diamonds, clubs, spades)
};

enum Status {
    OK = 0,
    ERR_IMAGE_SIZE,
    ERR_NO_CARD,
    ERR_NO_SYMBOL,
    ERR_NO_MATCH
};

struct CardResult {
    int rank = -1;
    int suit = -1;
    float rankScore = 1.0f; // udio različitih piksela (manje = bolje)
    float suitScore = 1.0f;
};

// Dijagnostika (opciono): šta je matcher zaista poredio sa šablonima.
struct GlyphDebug {
    uint8_t crop[SYM_MAX];   // isječak koji je dao najbolji rezultat (simbol = 255)
    int w = 0, h = 0;
    int candidate = -1;      // 0 = najveći komad, 1 = krajnji desni, 2 = unija komada
};

struct DebugInfo {
    int imgW = 0, imgH = 0;
    const uint8_t* binary = nullptr;   // binarizovana cijela slika (karta = 255)
    Component card;
    const uint8_t* warped = nullptr;   // CARD_W x CARD_H x 3
    const uint8_t* corner = nullptr;   // CORNER_W x CORNER_H, binarizovan ugao (simbol = 0)
    GlyphDebug rank, suit;
};

// ─── Pomoćne funkcije ────────────────────────────────────────────────────────

inline float ptDist(Point2f a, Point2f b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// RGB (3 bajta/piksel) -> 1 kanal. dst mora imati w*h bajtova.
inline void toGrayscale(const uint8_t* src, uint8_t* dst, int w, int h) {
    int n = w * h;
    for (int i = 0; i < n; ++i) {
        dst[i] = static_cast<uint8_t>(
            0.299f * src[i * 3] + 0.587f * src[i * 3 + 1] + 0.114f * src[i * 3 + 2]);
    }
}

// In-place: piksel > threshold -> 255, inače 0.
inline void binarize(uint8_t* data, int n, int threshold) {
    for (int i = 0; i < n; ++i) data[i] = (data[i] > threshold) ? 255 : 0;
}

// Isto, ali upisuje u poseban bafer.
inline void binarizeTo(const uint8_t* src, uint8_t* dst, int n, int threshold) {
    for (int i = 0; i < n; ++i) dst[i] = (src[i] > threshold) ? 255 : 0;
}

// Slika sa ch kanala (1, 2, 3 ili 4) -> binarni 1-kanalni bafer.
inline void binarizeRaw(const uint8_t* src, uint8_t* dst, int w, int h, int ch,
                        int threshold = 120) {
    int n = w * h;
    for (int i = 0; i < n; ++i) {
        float g;
        if (ch < 3) g = src[i * ch];   // siva (ili siva + alfa)
        else        g = src[i * ch + 0] * 0.30f + src[i * ch + 1] * 0.59f + src[i * ch + 2] * 0.11f;
        dst[i] = (g > threshold) ? 255 : 0;
    }
}

// ─── BFS: bijele komponente ─────────────────────────────────────────────────
//
// Za svaku komponentu prati se samo brojač, bounding box i 4 ekstremne tačke
// (min/max od x+y i x-y), pa nije potrebno pamtiti listu tačaka.
// findComponents čuva najviše maxOut NAJVEĆIH komponenti (redoslijed nije
// sortiran). Vraća broj sačuvanih komponenti u nOut.
// Nije re-entrant (statički baferi).

// Dijagnostika: najveća zauzetost BFS reda (tail-head) pri obradi CIJELE slike.
// To je dubina FIFO reda koju bi trebao hardverski BFS (red ne mora čuvati sve
// piksele komponente, nego samo front). Resetuje ih pozivalac.
inline int& bfsMaxFrontier() {
    static int v = 0;
    return v;
}

// CAP = najveći broj piksela slike koju funkcija smije da obradi. Statički baferi
// su srazmjerni CAP-u: cijela slika koristi MAX_PIXELS, a simboli iz ugla samo
// SYM_MAX (par KB). Instancira se samo ono što se zaista poziva.
template <int CAP>
inline void findComponentsT(const uint8_t* binary, int w, int h,
                            Component* out, int maxOut, int& nOut) {
    nOut = 0;
    const int n = w * h;
    if (w <= 0 || h <= 0 || n > CAP || maxOut <= 0) return;

    static uint8_t visited[CAP];
    static int queue[CAP];
    std::memset(visited, 0, n);

    for (int start = 0; start < n; ++start) {
        if (binary[start] != 255 || visited[start]) continue;

        Component cur;
        cur.minX = INT_MAX; cur.minY = INT_MAX;
        cur.maxX = INT_MIN; cur.maxY = INT_MIN;
        int minS = INT_MAX, maxS = INT_MIN, minD = INT_MAX, maxD = INT_MIN;

        int head = 0, tail = 0;
        queue[tail++] = start;
        visited[start] = 1;

        while (head < tail) {
            int idx = queue[head++];
            int x = idx % w, y = idx / w;
            ++cur.count;
            cur.minX = std::min(cur.minX, x); cur.maxX = std::max(cur.maxX, x);
            cur.minY = std::min(cur.minY, y); cur.maxY = std::max(cur.maxY, y);

            int s = x + y, d = x - y;
            if (s < minS) { minS = s; cur.tl = {(float)x, (float)y}; } // gore-lijevo
            if (s > maxS) { maxS = s; cur.br = {(float)x, (float)y}; } // dole-desno
            if (d > maxD) { maxD = d; cur.tr = {(float)x, (float)y}; } // gore-desno
            if (d < minD) { minD = d; cur.bl = {(float)x, (float)y}; } // dole-lijevo

            if (x + 1 < w) { int ni = idx + 1; if (binary[ni] == 255 && !visited[ni]) { visited[ni] = 1; queue[tail++] = ni; } }
            if (x > 0)     { int ni = idx - 1; if (binary[ni] == 255 && !visited[ni]) { visited[ni] = 1; queue[tail++] = ni; } }
            if (y + 1 < h) { int ni = idx + w; if (binary[ni] == 255 && !visited[ni]) { visited[ni] = 1; queue[tail++] = ni; } }
            if (y > 0)     { int ni = idx - w; if (binary[ni] == 255 && !visited[ni]) { visited[ni] = 1; queue[tail++] = ni; } }
            if (CAP == MAX_PIXELS && tail - head > bfsMaxFrontier()) bfsMaxFrontier() = tail - head;
        }

        if (nOut < maxOut) {
            out[nOut++] = cur;
        } else {
            // zamijeni najmanju sačuvanu ako je nova veća
            int smallest = 0;
            for (int i = 1; i < nOut; ++i)
                if (out[i].count < out[smallest].count) smallest = i;
            if (cur.count > out[smallest].count) out[smallest] = cur;
        }
    }
}

// Cijela slika (do MAX_PIXELS).
inline void findComponents(const uint8_t* binary, int w, int h,
                           Component* out, int maxOut, int& nOut) {
    findComponentsT<MAX_PIXELS>(binary, w, h, out, maxOut, nOut);
}

// Najveća komponenta (karta u cijeloj slici). false ako je nema.
inline bool findLargestComponent(const uint8_t* binary, int w, int h, Component& best) {
    int n = 0;
    findComponents(binary, w, h, &best, 1, n);
    if (n == 0) best = Component{};
    return n > 0 && best.count > 0;
}

// ─── Varijanta B: strujna lokalizacija karte ─────────────────────────────────
//
// Jedan prolaz kroz sliku, piksel po piksel, red po red (kako dolazi iz kamere
// ili iz memorije). Za svaki piksel: grayscale (ista formula kao toGrayscale),
// poređenje sa pragom i ažuriranje samo nekoliko brojača:
// broj piksela, bounding box i ekstremi od x+y i x-y (= 4 ugla karte).
// Nema međubafera (gray/binarna/visited/BFS red), pa nema ni BFS-a.
//
// Razlika prema originalu: umjesto NAJVEĆE svijetle komponente uzima se SVE
// što je svijetlo. Ako je na slici još jedna velika svijetla površina, uglovi
// će biti pogrešni; tačnost se provjerava na test skupu.

inline bool locateCardStreaming(const uint8_t* rgb, int w, int h, Component& card) {
    card = Component{};
    if (!rgb || w <= 0 || h <= 0 || w * h > MAX_PIXELS) return false;

    int count = 0;
    int minX = INT_MAX, maxX = INT_MIN, minY = INT_MAX, maxY = INT_MIN;
    int minS = INT_MAX, maxS = INT_MIN, minD = INT_MAX, maxD = INT_MIN;
    Point2f tl{}, tr{}, br{}, bl{};
    const int minRun = std::max(STREAM_MIN_RUN, w / STREAM_RUN_DIV);

    // ažuriranje okvira i ekstrema jednim pikselom
    auto upd = [&](int x, int y) {
        minX = std::min(minX, x); maxX = std::max(maxX, x);
        minY = std::min(minY, y); maxY = std::max(maxY, y);
        const int sum = x + y, diff = x - y;
        if (sum < minS)  { minS = sum;  tl = {(float)x, (float)y}; }
        if (sum > maxS)  { maxS = sum;  br = {(float)x, (float)y}; }
        if (diff > maxD) { maxD = diff; tr = {(float)x, (float)y}; }
        if (diff < minD) { minD = diff; bl = {(float)x, (float)y}; }
    };

    for (int y = 0; y < h; ++y) {
        int run = 0;   // koliko uzastopnih svijetlih piksela je do sada u ovom redu
        const uint8_t* row = rgb + (size_t)y * w * 3;
        for (int x = 0; x < w; ++x) {
            const uint8_t* p = row + x * 3;
            uint8_t g = static_cast<uint8_t>(0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2]);
            if (g > THRESH_CARD) ++run; else run = 0;
            if (run < minRun) continue;

            if (run == minRun) {
                // niz je upravo postao dovoljno dug: obračunaj i njegov POČETAK.
                // U nizu x+y i x-y rastu s x, pa su minimumi uvijek na početku,
                // a maksimumi na kraju (njih obračunavaju naredni pikseli).
                upd(x - minRun + 1, y);
                count += minRun - 1;
            }
            upd(x, y);
            ++count;
        }
    }
    if (count == 0) return false;
    card.count = count;
    card.minX = minX; card.maxX = maxX; card.minY = minY; card.maxY = maxY;
    card.tl = tl; card.tr = tr; card.br = br; card.bl = bl;
    return true;
}

// ─── Uglovi karte u redoslijedu za warp ──────────────────────────────────────
//
// Povratni redoslijed je uvijek u smjeru kazaljke na satu: [0]->[1] je gornja
// ivica izravnate (uspravne) karte. Ako je karta na slici položena (šira nego
// viša), redoslijed se rotira za 90°. Kako karta ima indeks i gore-lijevo i
// dole-desno (rotiran za 180°), svejedno u koju stranu je rotirana.

inline std::array<Point2f, 4> orderCorners(const Component& c) {
    float avgW = (ptDist(c.tl, c.tr) + ptDist(c.bl, c.br)) / 2.0f;
    float avgH = (ptDist(c.tl, c.bl) + ptDist(c.tr, c.br)) / 2.0f;
    if (avgW > avgH) return {c.bl, c.tl, c.tr, c.br};
    return {c.tl, c.tr, c.br, c.bl};
}

// ─── Izravnavanje karte (bilinearno preslikavanje četverougla) ───────────────
//
// dst mora imati dstW*dstH*3 bajtova. Napomena: ovo NIJE potpuna perspektivna
// transformacija (homografija), ali je dovoljno za blago nagnute karte.

inline void warpImage(const uint8_t* src, int srcW, int srcH,
                      const std::array<Point2f, 4>& c,
                      uint8_t* dst, int dstW, int dstH) {
    for (int y = 0; y < dstH; ++y) {
        float v = y / (float)(dstH - 1);
        for (int x = 0; x < dstW; ++x) {
            float u = x / (float)(dstW - 1);
            Point2f top    = {(1 - u) * c[0].x + u * c[1].x, (1 - u) * c[0].y + u * c[1].y};
            Point2f bottom = {(1 - u) * c[3].x + u * c[2].x, (1 - u) * c[3].y + u * c[2].y};
            Point2f p      = {(1 - v) * top.x + v * bottom.x, (1 - v) * top.y + v * bottom.y};
            int px = std::clamp((int)p.x, 0, srcW - 1);
            int py = std::clamp((int)p.y, 0, srcH - 1);
            for (int ch = 0; ch < 3; ++ch)
                dst[(y * dstW + x) * 3 + ch] = src[(py * srcW + px) * 3 + ch];
        }
    }
}

// ─── Isječci ─────────────────────────────────────────────────────────────────

inline void cropTopLeft(const uint8_t* src, int srcW, uint8_t* dst, int cW, int cH) {
    for (int y = 0; y < cH; ++y)
        for (int x = 0; x < cW; ++x)
            for (int c = 0; c < 3; ++c)
                dst[(y * cW + x) * 3 + c] = src[(y * srcW + x) * 3 + c];
}

// Gornjih 60% -> rank, donjih 40% -> suit. rankBuf i suitBuf: w*h*3 (worst-case).
inline void splitSymbol(const uint8_t* src, int w, int h,
                        uint8_t* rankBuf, int& rankH,
                        uint8_t* suitBuf, int& suitH) {
    int mid = static_cast<int>(h * 0.60f);
    rankH = mid;
    suitH = h - mid;
    for (int y = 0; y < rankH; ++y)
        for (int x = 0; x < w; ++x)
            for (int c = 0; c < 3; ++c)
                rankBuf[(y * w + x) * 3 + c] = src[(y * w + x) * 3 + c];
    for (int y = 0; y < suitH; ++y)
        for (int x = 0; x < w; ++x)
            for (int c = 0; c < 3; ++c)
                suitBuf[(y * w + x) * 3 + c] = src[((y + mid) * w + x) * 3 + c];
}

// ─── Poređenje simbola sa šablonima ──────────────────────────────────────────
//
// bin: binarizovan 1-kanalni isječak (simbol je CRN = 0).
// Vraća id najboljeg šablona ili -1; score = udio različitih piksela [0,1].
//
// Problem: simbol se na niskoj rezoluciji može raspasti na više komada (npr.
// desetka = cifre "1" i "0"; šablon za desetku je samo "0"). Zato se ne bira
// unaprijed jedan komad, nego se isprobaju kandidati i uzima se onaj koji ima
// NAJBOLJI score sa nekim šablonom:
//   0 = najveći komad
//   1 = krajnji desni značajni komad  (samo ako tryRightmost)
//   2 = unija svih značajnih komada   (samo ako ih je više)
// "Značajan" = barem 30% piksela najvećeg komada (sitne mrlje se ignorišu).

inline void scoreCrop(const uint8_t* cropped, int cW, int cH,
                      const Template* tpls, int nTpl,
                      int& bestId, float& bestScore) {
    static uint8_t resized[TPL_MAX];
    bestId = -1;
    bestScore = 2.0f;
    for (int t = 0; t < nTpl; ++t) {
        const Template& tp = tpls[t];
        if (tp.w <= 0 || tp.h <= 0) continue;

        // nearest-neighbor: isječak -> dimenzije šablona
        const float xR = (float)cW / tp.w, yR = (float)cH / tp.h;
        for (int y = 0; y < tp.h; ++y)
            for (int x = 0; x < tp.w; ++x)
                resized[y * tp.w + x] = cropped[(int)(y * yR) * cW + (int)(x * xR)];

        int diff = 0;
        const int area = tp.w * tp.h;
        for (int i = 0; i < area; ++i)
            if (resized[i] != tp.bin[i]) ++diff;

        float sc = (float)diff / (float)area;
        if (sc < bestScore) { bestScore = sc; bestId = tp.id; }
    }
}

inline int matchSymbol(const uint8_t* bin, int w, int h,
                       const Template* tpls, int nTpl, float& score,
                       bool tryRightmost = false, GlyphDebug* gd = nullptr) {
    score = 1.0f;
    if (w <= 0 || h <= 0 || w * h > SYM_MAX) return -1;

    static uint8_t inverted[SYM_MAX];
    static uint8_t cropped[SYM_MAX];

    // crni simbol -> bijeli
    const int n = w * h;
    for (int i = 0; i < n; ++i) inverted[i] = 255 - bin[i];

    Component comps[8];
    int nComps = 0;
    findComponentsT<SYM_MAX>(inverted, w, h, comps, 8, nComps);
    if (nComps == 0) return -1;

    int largest = 0;
    for (int i = 1; i < nComps; ++i)
        if (comps[i].count > comps[largest].count) largest = i;
    const int minCount = comps[largest].count * 3 / 10;

    int rightmost = largest, nSignificant = 0;
    int uMinX = INT_MAX, uMaxX = INT_MIN, uMinY = INT_MAX, uMaxY = INT_MIN;
    for (int i = 0; i < nComps; ++i) {
        if (comps[i].count < minCount) continue;
        ++nSignificant;
        uMinX = std::min(uMinX, comps[i].minX); uMaxX = std::max(uMaxX, comps[i].maxX);
        uMinY = std::min(uMinY, comps[i].minY); uMaxY = std::max(uMaxY, comps[i].maxY);
        if (comps[i].minX + comps[i].maxX > comps[rightmost].minX + comps[rightmost].maxX)
            rightmost = i;
    }

    // kandidati: okviri (minX,maxX,minY,maxY) + oznaka
    int box[3][4]; int tag[3]; int nCand = 0;
    auto addBox = [&](int x0, int x1, int y0, int y1, int t) {
        box[nCand][0] = x0; box[nCand][1] = x1; box[nCand][2] = y0; box[nCand][3] = y1;
        tag[nCand++] = t;
    };
    addBox(comps[largest].minX, comps[largest].maxX, comps[largest].minY, comps[largest].maxY, 0);
    if (tryRightmost && rightmost != largest)
        addBox(comps[rightmost].minX, comps[rightmost].maxX, comps[rightmost].minY, comps[rightmost].maxY, 1);
    if (nSignificant > 1)
        addBox(uMinX, uMaxX, uMinY, uMaxY, 2);

    int bestId = -1;
    float bestScore = 2.0f;
    for (int c = 0; c < nCand; ++c) {
        const int minX = std::max(0, box[c][0] - SYMBOL_PAD);
        const int maxX = std::min(w - 1, box[c][1] + SYMBOL_PAD);
        const int minY = std::max(0, box[c][2] - SYMBOL_PAD);
        const int maxY = std::min(h - 1, box[c][3] + SYMBOL_PAD);
        const int cW = maxX - minX + 1, cH = maxY - minY + 1;

        for (int y = 0; y < cH; ++y)
            for (int x = 0; x < cW; ++x)
                cropped[y * cW + x] = inverted[(minY + y) * w + (minX + x)];

        int id; float sc;
        scoreCrop(cropped, cW, cH, tpls, nTpl, id, sc);
        if (id != -1 && sc < bestScore) {
            bestScore = sc; bestId = id;
            if (gd) {
                gd->w = cW; gd->h = cH; gd->candidate = tag[c];
                std::memcpy(gd->crop, cropped, (size_t)cW * cH);
            }
        }
    }
    if (bestId != -1) score = bestScore;
    return bestId;
}

// ─── Imena ───────────────────────────────────────────────────────────────────

inline const char* rankName(int r) {
    static const char* names[] = {"unknown", "two", "three", "four", "five", "six", "seven",
                                  "eight", "nine", "ten", "jack", "queen", "king", "ace"};
    return (r >= 0 && r <= 13) ? names[r] : "unknown";
}

inline const char* suitName(int s) {
    static const char* names[] = {"hearts", "diamonds", "clubs", "spades"};
    return (s >= 0 && s < 4) ? names[s] : "unknown";
}

inline const char* statusText(Status s) {
    switch (s) {
        case OK:             return "ok";
        case ERR_IMAGE_SIZE: return "neispravna ili prevelika slika";
        case ERR_NO_CARD:    return "karta nije pronadjena na slici";
        case ERR_NO_SYMBOL:  return "simbol u uglu karte nije pronadjen";
        case ERR_NO_MATCH:   return "nijedan sablon ne odgovara";
    }
    return "?";
}

// ─── Jedina ulazna tačka algoritma ───────────────────────────────────────────
//
//   rgb – RGB slika, 3 bajta/piksel, širine w i visine h
//   tpl – šabloni (učitani unaprijed)
//   out – rezultat (rank 1..13, suit 0..3)

// Drugi dio algoritma: od pronađene karte (uglova) do ranka i suita. U podijeljenom
// sistemu ovo radi softver (PS), a uglove daje akcelerator.
inline Status recognizeFromCard(const uint8_t* rgb, int w, int h, const Component& card,
                                const TemplateSet& tpl, CardResult& out,
                                DebugInfo* dbg = nullptr);

inline Status recognizeCard(const uint8_t* rgb, int w, int h,
                            const TemplateSet& tpl, CardResult& out,
                            DebugInfo* dbg = nullptr) {
    out = CardResult{};
    if (dbg) { dbg->imgW = w; dbg->imgH = h; }
    if (!rgb || w <= 0 || h <= 0 || w * h > MAX_PIXELS) return ERR_IMAGE_SIZE;

    Component card;
#ifdef CARD_LOCALIZE_STREAMING
    // [1+2] varijanta B: lokalizacija karte u jednom prolazu, bez međubafera
    if (!locateCardStreaming(rgb, w, h, card) || card.count < MIN_CARD_PIXELS)
        return ERR_NO_CARD;
    if (dbg) dbg->card = card;   // dbg->binary ostaje nullptr (ne postoji binarna slika)
#else
    // [1] grayscale + binarizacija cijele slike
    static uint8_t grayBuf[MAX_PIXELS];
    static uint8_t binaryBuf[MAX_PIXELS];
    toGrayscale(rgb, grayBuf, w, h);
    binarizeTo(grayBuf, binaryBuf, w * h, THRESH_CARD);

    // [2] najveća bijela komponenta = karta; uglovi
    if (!findLargestComponent(binaryBuf, w, h, card) || card.count < MIN_CARD_PIXELS)
        return ERR_NO_CARD;
    if (dbg) { dbg->binary = binaryBuf; dbg->card = card; }
#endif
    return recognizeFromCard(rgb, w, h, card, tpl, out, dbg);
}

inline Status recognizeFromCard(const uint8_t* rgb, int w, int h, const Component& card,
                                const TemplateSet& tpl, CardResult& out,
                                DebugInfo* dbg) {
    out = CardResult{};
    if (!rgb || w <= 0 || h <= 0 || w * h > MAX_PIXELS) return ERR_IMAGE_SIZE;
    auto corners = orderCorners(card);

    // [3] izravnavanje karte na 200x300
    static uint8_t warped[CARD_W * CARD_H * 3];
    warpImage(rgb, w, h, corners, warped, CARD_W, CARD_H);
    if (dbg) dbg->warped = warped;

    // [4] gornji lijevi ugao 33x90
    static uint8_t cornerBuf[CORNER_W * CORNER_H * 3];
    cropTopLeft(warped, CARD_W, cornerBuf, CORNER_W, CORNER_H);

    // [5] grayscale + binarizacija ugla
    static uint8_t grayCorner[CORNER_W * CORNER_H];
    static uint8_t binCorner[CORNER_W * CORNER_H];
    toGrayscale(cornerBuf, grayCorner, CORNER_W, CORNER_H);
    binarizeTo(grayCorner, binCorner, CORNER_W * CORNER_H, THRESH_CORNER);
    if (dbg) dbg->corner = binCorner;

    // [6] okvir oko crnih piksela (rank + suit zajedno)
    int minX = CORNER_W, maxX = -1, minY = CORNER_H, maxY = -1;
    for (int y = 0; y < CORNER_H; ++y)
        for (int x = 0; x < CORNER_W; ++x)
            if (binCorner[y * CORNER_W + x] == 0) {
                minX = std::min(minX, x); maxX = std::max(maxX, x);
                minY = std::min(minY, y); maxY = std::max(maxY, y);
            }
    if (maxX < 0) return ERR_NO_SYMBOL;
    const int symW = maxX - minX + 1, symH = maxY - minY + 1;
    if (symH < 2) return ERR_NO_SYMBOL;

    static uint8_t symBuf[CORNER_W * CORNER_H * 3];
    for (int y = 0; y < symH; ++y)
        for (int x = 0; x < symW; ++x)
            for (int c = 0; c < 3; ++c)
                symBuf[(y * symW + x) * 3 + c] =
                    cornerBuf[((minY + y) * CORNER_W + (minX + x)) * 3 + c];

    // [7] podjela na rank (gore) i suit (dolje)
    static uint8_t rankRGB[CORNER_W * CORNER_H * 3];
    static uint8_t suitRGB[CORNER_W * CORNER_H * 3];
    int rankH = 0, suitH = 0;
    splitSymbol(symBuf, symW, symH, rankRGB, rankH, suitRGB, suitH);

    // [8] grayscale + binarizacija isječaka
    static uint8_t rankBin[CORNER_W * CORNER_H];
    static uint8_t suitBin[CORNER_W * CORNER_H];
    toGrayscale(rankRGB, rankBin, symW, rankH);
    toGrayscale(suitRGB, suitBin, symW, suitH);
    binarize(rankBin, symW * rankH, THRESH_SYMBOL);
    binarize(suitBin, symW * suitH, THRESH_SYMBOL);

    // [9] poređenje sa šablonima
    int rank = matchSymbol(rankBin, symW, rankH, tpl.ranks, NUM_RANKS, out.rankScore,
                           true, dbg ? &dbg->rank : nullptr);
    int suit = matchSymbol(suitBin, symW, suitH, tpl.suits, NUM_SUITS, out.suitScore,
                           false, dbg ? &dbg->suit : nullptr);
    if (rank == -1 || suit == -1) return ERR_NO_MATCH;

    out.rank = rank;
    out.suit = suit;
    return OK;
}
