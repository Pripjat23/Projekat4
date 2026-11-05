// vp_main.cpp
//
// Testbench virtuelne platforme: "procesor" (softver) + sc_main.
//
// Softver za svaku sliku radi isto što bi radio PS na ploči:
//   1. učita sliku, umanji je (CARD_MAX_SIDE / CARD_DOWNSCALE), pretvori u sivu, binarizuje
//   2. upiše bitove u BRAM preko magistrale (riječ po riječ ili paketno)
//   3. upiše IMG_W, IMG_H, CTRL.START (sa prekidom ili proziva STATUS)
//   4. čeka završetak, pročita registre (uglovi karte)
//   5. uporedi sa referentnim modelom (findLargestComponent)
//   6. softverski dio: iz uglova izravna kartu, prepozna rank i suit i ISPIŠE KARTU
// i ispisuje SIMULIRANO vrijeme po fazama. Šabloni se čitaju iz Card_Imgs/.
// Vrijeme softverskog dijela (6) se ne simulira: na ARM-u ga treba izmjeriti.
//
// Prevođenje: vidi Makefile (treba SystemC sa TLM-om).
// Pokretanje: CARD_MAX_SIDE=1024 ./vp tests/*.jpeg [--burst[=bajtova]] [--poll]

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "vp_platform.hpp"
#include "downscale.hpp"
#include "template_loader.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

struct Options {
    bool burst = false;          // paketni upis slike (DMA) umjesto riječ po riječ
    unsigned burstBytes = 1024;  // veličina jednog paketa
    bool poll = false;           // proziva STATUS umjesto prekida
};

// "tests/ace_spades_1.jpeg" -> "ace spades"; "" ako ime nije u tom obliku
static std::string expectedFromName(const std::string& path) {
    size_t slash = path.find_last_of("/\\");
    std::string b = (slash == std::string::npos) ? path : path.substr(slash + 1);
    size_t dot = b.find_last_of('.');
    if (dot != std::string::npos) b = b.substr(0, dot);
    size_t u1 = b.find('_');
    if (u1 == std::string::npos) return "";
    std::string r = b.substr(0, u1), rest = b.substr(u1 + 1);
    size_t u2 = rest.find('_');
    std::string su = (u2 == std::string::npos) ? rest : rest.substr(0, u2);
    bool okR = false, okS = false;
    for (int i = 1; i <= NUM_RANKS; ++i) if (r == rankName(i)) okR = true;
    for (int i = 0; i < NUM_SUITS; ++i) if (su == suitName(i)) okS = true;
    return (okR && okS) ? r + " " + su : "";
}

class Cpu : public sc_core::sc_module {
public:
    tlm_utils::simple_initiator_socket<Cpu> socket;
    sc_core::sc_in<bool> irq;

    SC_HAS_PROCESS(Cpu);

    Cpu(sc_core::sc_module_name name, const Options& opt,
        const std::vector<std::string>& images, const vp::Timing& t)
        : sc_core::sc_module(name), socket("socket"), irq("irq"), opt_(opt), images_(images), t_(t) {
        SC_THREAD(software);
    }

    int failures() const { return failures_; }

private:
    // ─── pristup magistrali (jedan 32-bitni ili paketni prenos) ──────────────
    void transfer(tlm::tlm_command cmd, uint64_t addr, unsigned char* data, unsigned len) {
        tlm::tlm_generic_payload trans;
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        trans.set_command(cmd);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(len);
        trans.set_streaming_width(len);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
        socket->b_transport(trans, delay);
        if (trans.is_response_error()) {
            std::fprintf(stderr, "GRESKA na magistrali, adresa 0x%llx\n", (unsigned long long)addr);
            ++failures_;
        }
        sc_core::wait(delay);   // loosely-timed: procesor "potroši" vrijeme transakcije
    }
    void write32(uint64_t addr, uint32_t v) {
        transfer(tlm::TLM_WRITE_COMMAND, addr, reinterpret_cast<unsigned char*>(&v), 4);
    }
    uint32_t read32(uint64_t addr) {
        uint32_t v = 0;
        transfer(tlm::TLM_READ_COMMAND, addr, reinterpret_cast<unsigned char*>(&v), 4);
        return v;
    }

    static double ms(sc_core::sc_time t) { return t.to_seconds() * 1000.0; }
    static int bx(uint32_t v) { return (int)(v & 0x3FFu); }
    static int by(uint32_t v) { return (int)((v >> 16) & 0x3FFu); }

    // ─── softver ─────────────────────────────────────────────────────────────
    void software() {
        double sumWrite = 0, sumAccel = 0, sumRead = 0;
        int nOk = 0, nImg = 0, nExpected = 0, nCorrect = 0, nSameAsSw = 0, nRecognized = 0;

        static TemplateSet templates;
        const bool haveTemplates = loadTemplates("Card_Imgs", templates);
        if (!haveTemplates)
            std::fprintf(stderr, "Sabloni nisu ucitani (Card_Imgs/): ispisuje se samo lokalizacija karte.\n");

        for (const std::string& path : images_) {
            int w, h, ch;
            uint8_t* img = stbi_load(path.c_str(), &w, &h, &ch, 3);
            if (!img) {
                std::fprintf(stderr, "%s: ne mogu da ucitam\n", path.c_str());
                ++failures_;
                continue;
            }

            // 1) priprema (u pravom sistemu radi PS u softveru)
            std::vector<uint8_t> small;
            const uint8_t* pix = img;
            const int f = downscaleFactorFor(w, h);
            if (f > 1 && downscaleBox(img, w, h, f, small, w, h)) pix = small.data();

            std::vector<uint8_t> gray((size_t)w * h), bin((size_t)w * h);
            toGrayscale(pix, gray.data(), w, h);
            binarizeTo(gray.data(), bin.data(), w * h, THRESH_CARD);

            Component ref;
            const bool refOk = findLargestComponent(bin.data(), w, h, ref);

            std::vector<uint32_t> words((size_t)(w * h + 31) / 32, 0u);
            for (int i = 0; i < w * h; ++i)
                if (bin[(size_t)i] == 255) words[(size_t)i >> 5] |= 1u << (i & 31);

            if (w > BFS_MAX_DIM || h > BFS_MAX_DIM || w * h > BFS_MAX_PIXELS) {
                std::printf("%-30s %4dx%-4d NE STAJE u akcelerator\n", path.c_str(), w, h);
                ++failures_;
                stbi_image_free(img);
                continue;
            }

            // 2) upis slike u BRAM
            const sc_core::sc_time t0 = sc_core::sc_time_stamp();
            if (opt_.burst) {
                unsigned char* base = reinterpret_cast<unsigned char*>(words.data());
                const unsigned total = (unsigned)words.size() * 4;
                for (unsigned off = 0; off < total; off += opt_.burstBytes) {
                    unsigned len = total - off < opt_.burstBytes ? total - off : opt_.burstBytes;
                    transfer(tlm::TLM_WRITE_COMMAND, vp::BRAM_BASE + off, base + off, len);
                }
            } else {
                for (size_t i = 0; i < words.size(); ++i) write32(vp::BRAM_BASE + 4 * i, words[i]);
            }
            const sc_core::sc_time t1 = sc_core::sc_time_stamp();

            // 3) pokretanje i čekanje
            write32(vp::ACC_BASE + vp::REG_IMG_W, (uint32_t)w);
            write32(vp::ACC_BASE + vp::REG_IMG_H, (uint32_t)h);
            write32(vp::ACC_BASE + vp::REG_CTRL, vp::CTRL_START | (opt_.poll ? 0u : vp::CTRL_IRQ_EN));
            if (opt_.poll) {
                while (!(read32(vp::ACC_BASE + vp::REG_STATUS) & vp::ST_DONE)) {}
            } else {
                while (!irq.read()) sc_core::wait(irq.value_changed_event());
            }
            const sc_core::sc_time t2 = sc_core::sc_time_stamp();

            // 4) rezultati
            const uint32_t st    = read32(vp::ACC_BASE + vp::REG_STATUS);
            const uint32_t count = read32(vp::ACC_BASE + vp::REG_CARD_COUNT);
            const uint32_t bbx   = read32(vp::ACC_BASE + vp::REG_BBOX_X);
            const uint32_t bby   = read32(vp::ACC_BASE + vp::REG_BBOX_Y);
            const uint32_t tl    = read32(vp::ACC_BASE + vp::REG_CORNER_TL);
            const uint32_t tr    = read32(vp::ACC_BASE + vp::REG_CORNER_TR);
            const uint32_t br    = read32(vp::ACC_BASE + vp::REG_CORNER_BR);
            const uint32_t bl    = read32(vp::ACC_BASE + vp::REG_CORNER_BL);
            const uint32_t front = read32(vp::ACC_BASE + vp::REG_MAX_FRONT);
            const sc_core::sc_time t3 = sc_core::sc_time_stamp();

            // 5) poređenje sa referentnim modelom
            const bool flags = !(st & (vp::ST_NO_CARD | vp::ST_OVERFLOW | vp::ST_ERR_SIZE));
            auto eq = [](Point2f a, uint32_t v) { return (int)a.x == bx(v) && (int)a.y == by(v); };
            const bool match = refOk && flags && (int)count == ref.count &&
                               bx(bbx) == ref.minX && by(bbx) == ref.maxX &&
                               bx(bby) == ref.minY && by(bby) == ref.maxY &&
                               eq(ref.tl, tl) && eq(ref.tr, tr) && eq(ref.br, br) && eq(ref.bl, bl);

            std::printf("%-30s %4dx%-4d %-13s karta %6u px, front %4u | upis %7.3f ms, akcelerator %7.3f ms, citanje %6.3f ms\n",
                        path.c_str(), w, h, match ? "IDENTICNO" : "NEPODUDARANJE", count, front,
                        ms(t1 - t0), ms(t2 - t1), ms(t3 - t2));
            if (match) ++nOk; else ++failures_;
            ++nImg;
            sumWrite += ms(t1 - t0); sumAccel += ms(t2 - t1); sumRead += ms(t3 - t2);

            // 6) softverski dio (PS): uglovi iz registara -> izravnavanje, izrezivanje ugla,
            //    podjela simbola, poređenje sa šablonima -> rank i suit
            if (haveTemplates && match) {
                Component card;
                card.count = (int)count;
                card.minX = bx(bbx); card.maxX = by(bbx);
                card.minY = bx(bby); card.maxY = by(bby);
                card.tl = {(float)bx(tl), (float)by(tl)};
                card.tr = {(float)bx(tr), (float)by(tr)};
                card.br = {(float)bx(br), (float)by(br)};
                card.bl = {(float)bx(bl), (float)by(bl)};

                CardResult res, sw;
                const Status st = recognizeFromCard(pix, w, h, card, templates, res);
                const Status stSw = recognizeCard(pix, w, h, templates, sw);   // sam softver, radi poređenja

                std::string got = "-";
                if (st == OK) got = std::string(rankName(res.rank)) + " " + suitName(res.suit);
                const bool sameAsSw = (st == stSw) && res.rank == sw.rank && res.suit == sw.suit;
                const std::string expected = expectedFromName(path);

                std::printf("    KARTA: %-16s", got.c_str());
                if (st != OK) std::printf(" (%s)", statusText(st));
                std::printf("  [softver bez akceleratora: %s]", sameAsSw ? "isto" : "RAZLICITO");
                if (!expected.empty()) {
                    ++nExpected;
                    const bool correct = (got == expected);
                    if (correct) ++nCorrect;
                    std::printf("  [ocekivano po imenu fajla: %s -> %s]", expected.c_str(), correct ? "TACNO" : "GRESKA");
                }
                std::printf("\n");
                if (st == OK) ++nRecognized;
                if (sameAsSw) ++nSameAsSw;
            }
            stbi_image_free(img);
        }

        if (nImg > 0) {
            std::printf("\n=== SAZETAK (simulirano vrijeme, prosjek po slici) ===\n");
            std::printf("slika: %d, identicno referentnom modelu: %d\n", nImg, nOk);
            std::printf("upis slike u BRAM (%s): %.3f ms\n", opt_.burst ? "paketno" : "rijec po rijec", sumWrite / nImg);
            std::printf("akcelerator (takt %.0f MHz, %d ciklusa po pikselu BFS-a): %.3f ms\n",
                        1000.0 / t_.clkNs, t_.bfs.cyclesPerPixel, sumAccel / nImg);
            std::printf("citanje rezultata: %.3f ms\n", sumRead / nImg);
            std::printf("ukupno hardverski dio: %.3f ms\n", (sumWrite + sumAccel + sumRead) / nImg);
            if (haveTemplates) {
                std::printf("prepoznato (rank i suit nadjeni): %d od %d\n", nRecognized, nImg);
                std::printf("isto kao samo softver (bez akceleratora): %d od %d\n", nSameAsSw, nImg);
                if (nExpected > 0)
                    std::printf("tacno prema imenu fajla: %d od %d\n", nCorrect, nExpected);
            }
        }
        sc_core::sc_stop();
    }

    Options opt_;
    std::vector<std::string> images_;
    const vp::Timing& t_;
    int failures_ = 0;
};

int sc_main(int argc, char* argv[]) {
    Options opt;
    std::vector<std::string> images;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--poll") opt.poll = true;
        else if (a == "--burst") opt.burst = true;
        else if (a.rfind("--burst=", 0) == 0) { opt.burst = true; opt.burstBytes = (unsigned)std::atoi(a.c_str() + 8); }
        else images.push_back(a);
    }
    if (opt.burstBytes < 4) opt.burstBytes = 4;
    opt.burstBytes &= ~3u;
    if (images.empty()) {
        std::fprintf(stderr, "Upotreba: %s slika... [--burst[=bajtova]] [--poll]\n", argv[0]);
        return 2;
    }

    vp::Timing timing;
    vp::Bram bram("bram");
    vp::Bus bus("bus", timing);
    vp::Accel accel("accel", bram, timing);
    Cpu cpu("cpu", opt, images, timing);
    sc_core::sc_signal<bool> irq_sig("irq_sig");

    cpu.socket.bind(bus.tsock);
    bus.bram_sock.bind(bram.socket);
    bus.acc_sock.bind(accel.socket);
    accel.irq.bind(irq_sig);
    cpu.irq.bind(irq_sig);

    sc_core::sc_start();
    return cpu.failures() ? 1 : 0;
}
