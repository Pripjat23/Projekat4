// vp_platform.hpp
//
// Virtuelna platforma (SystemC + TLM 2.0, loosely-timed): BRAM, magistrala i
// akcelerator. Procesor (softver/testbench) je u vp_main.cpp.
//
//   Cpu (initiator) --> Bus --+--> Bram    (0x4000_0000, 128 KB, port A)
//                             +--> Accel   (0x43C0_0000, registri)
//   Accel --(port B, direktno)--> Bram     (lokalni pristup, bez magistrale)
//   Accel.irq --> Cpu.irq
//
// Funkcionalno jezgro akceleratora je runBfs() iz bfs_core.hpp (testirano odvojeno).
// Ovaj fajl dodaje: adresnu mapu, registre, protokol i vrijeme.
#pragma once

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "bfs_core.hpp"

namespace vp {

// ─── Adresna mapa i registri (poglavlje 5.3 specifikacije) ───────────────────
constexpr uint64_t BRAM_BASE = 0x40000000ull;
constexpr uint64_t BRAM_SIZE = 128 * 1024;        // 1024 x 1024 bita
constexpr uint64_t ACC_BASE  = 0x43C00000ull;
constexpr uint64_t ACC_SIZE  = 0x1000;

constexpr uint32_t REG_CTRL = 0x00, REG_STATUS = 0x04, REG_IMG_W = 0x08, REG_IMG_H = 0x0C,
                   REG_CARD_COUNT = 0x10, REG_BBOX_X = 0x14, REG_BBOX_Y = 0x18,
                   REG_CORNER_TL = 0x1C, REG_CORNER_TR = 0x20, REG_CORNER_BR = 0x24,
                   REG_CORNER_BL = 0x28, REG_MAX_FRONT = 0x2C, REG_NUM_COMP = 0x30;

constexpr uint32_t CTRL_START = 1u << 0, CTRL_IRQ_EN = 1u << 1, CTRL_SOFT_RESET = 1u << 2;
constexpr uint32_t ST_BUSY = 1u << 0, ST_DONE = 1u << 1, ST_NO_CARD = 1u << 2,
                   ST_OVERFLOW = 1u << 3, ST_ERR_SIZE = 1u << 4;

// ─── Parametri vremenskog modela (PRETPOSTAVKE; provjeriti u koraku 6) ───────
struct Timing {
    double clkNs = 10.0;          // takt akceleratora: 100 MHz
    int busSingleCycles = 20;     // jedan 32-bitni pristup PS -> PL (AXI GP)
    int busBurstSetupCycles = 20; // početak paketnog prenosa
    int busBytesPerCycle = 4;     // propusnost paketnog prenosa (32-bitna GP magistrala)
    BfsTiming bfs;                // ciklusi samog akceleratora
};

// ─── BRAM: port A (TLM, sa magistrale) i port B (direktan, za akcelerator) ───
class Bram : public sc_core::sc_module {
public:
    tlm_utils::simple_target_socket<Bram> socket;
    std::vector<uint32_t> words;

    Bram(sc_core::sc_module_name name)
        : sc_core::sc_module(name), socket("socket"), words(BRAM_SIZE / 4, 0u) {
        socket.register_b_transport(this, &Bram::b_transport);
    }

    // port B: čitanje/upis riječi (jedan ciklus; ciklusi se broje u akceleratoru)
    uint32_t readWord(int i) const { return words[(size_t)i]; }
    void writeWord(int i, uint32_t v) { words[(size_t)i] = v; }

private:
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& /*delay*/) {
        const uint64_t addr = trans.get_address();
        const unsigned len = trans.get_data_length();
        unsigned char* ptr = trans.get_data_ptr();
        if ((addr & 3) || (len & 3) || addr + len > BRAM_SIZE || ptr == nullptr) {
            trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }
        if (trans.is_write()) std::memcpy(&words[addr / 4], ptr, len);
        else                  std::memcpy(ptr, &words[addr / 4], len);
        trans.set_response_status(tlm::TLM_OK_RESPONSE);
    }
};

// ─── Magistrala: dekodiranje adresa + kašnjenje ──────────────────────────────
class Bus : public sc_core::sc_module {
public:
    tlm_utils::simple_target_socket<Bus> tsock;
    tlm_utils::simple_initiator_socket<Bus> bram_sock;
    tlm_utils::simple_initiator_socket<Bus> acc_sock;

    Bus(sc_core::sc_module_name name, const Timing& t)
        : sc_core::sc_module(name), tsock("tsock"), bram_sock("bram_sock"), acc_sock("acc_sock"), t_(t) {
        tsock.register_b_transport(this, &Bus::b_transport);
    }

private:
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay) {
        const uint64_t addr = trans.get_address();
        const unsigned len = trans.get_data_length();

        // kašnjenje: jedan pristup ili paketni prenos
        int cycles;
        if (len <= 4) cycles = t_.busSingleCycles;
        else cycles = t_.busBurstSetupCycles + (int)((len + t_.busBytesPerCycle - 1) / t_.busBytesPerCycle);
        delay += sc_core::sc_time(cycles * t_.clkNs, sc_core::SC_NS);

        if (addr >= BRAM_BASE && addr + len <= BRAM_BASE + BRAM_SIZE) {
            trans.set_address(addr - BRAM_BASE);
            bram_sock->b_transport(trans, delay);
        } else if (addr >= ACC_BASE && addr + len <= ACC_BASE + ACC_SIZE) {
            trans.set_address(addr - ACC_BASE);
            acc_sock->b_transport(trans, delay);
        } else {
            trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }
        trans.set_address(addr);   // vrati originalnu adresu pozivaocu
    }

    const Timing& t_;
};

// ─── Akcelerator: registri (AXI-Lite) + BFS nad BRAM-om (port B) ─────────────
class Accel : public sc_core::sc_module {
public:
    tlm_utils::simple_target_socket<Accel> socket;
    sc_core::sc_out<bool> irq;

    SC_HAS_PROCESS(Accel);

    Accel(sc_core::sc_module_name name, Bram& bram, const Timing& t)
        : sc_core::sc_module(name), socket("socket"), irq("irq"), bram_(bram), t_(t) {
        socket.register_b_transport(this, &Accel::b_transport);
        SC_THREAD(run_thread);
        // JEDINI proces koji piše u irq (sc_signal smije imati samo jednog "vozača"):
        SC_METHOD(drive_irq);
        sensitive << irq_ev_;
        dont_initialize();
    }

    // statistika za izvještaj testbench-a
    uint64_t lastCycles() const { return lastCycles_; }
    uint64_t lastVisited() const { return lastVisited_; }

private:
    // adapter: port B BRAM-a kao "Mem" za runBfs
    struct PortB {
        Bram& b;
        uint32_t readWord(int i) { return b.readWord(i); }
        void writeWord(int i, uint32_t v) { b.writeWord(i, v); }
    };

    // Svi ostali dijelovi samo postave željeni nivo prekida; upis u signal radi drive_irq.
    void setIrq(bool level) {
        irqLevel_ = level;
        irq_ev_.notify(sc_core::SC_ZERO_TIME);
    }
    void drive_irq() { irq.write(irqLevel_); }

    static uint32_t pack(uint32_t lo, uint32_t hi) { return (lo & 0x3FFu) | ((hi & 0x3FFu) << 16); }

    uint32_t regRead(uint32_t a) const {
        switch (a) {
            case REG_CTRL:       return irqEn_ ? CTRL_IRQ_EN : 0u;
            case REG_STATUS:     return status_;
            case REG_IMG_W:      return w_;
            case REG_IMG_H:      return h_;
            case REG_CARD_COUNT: return res_.count;
            case REG_BBOX_X:     return pack(res_.minX, res_.maxX);
            case REG_BBOX_Y:     return pack(res_.minY, res_.maxY);
            case REG_CORNER_TL:  return pack(res_.tl.x, res_.tl.y);
            case REG_CORNER_TR:  return pack(res_.tr.x, res_.tr.y);
            case REG_CORNER_BR:  return pack(res_.br.x, res_.br.y);
            case REG_CORNER_BL:  return pack(res_.bl.x, res_.bl.y);
            case REG_MAX_FRONT:  return res_.maxFrontier;
            case REG_NUM_COMP:   return res_.numComponents;
            default:             return 0u;
        }
    }

    void regWrite(uint32_t a, uint32_t v) {
        switch (a) {
            case REG_CTRL:
                irqEn_ = (v & CTRL_IRQ_EN) != 0;
                if (v & CTRL_SOFT_RESET) { status_ = 0; setIrq(false); }
                if ((v & CTRL_START) && !(status_ & ST_BUSY)) start();
                break;
            case REG_IMG_W: w_ = v; break;
            case REG_IMG_H: h_ = v; break;
            default: break;   // ostali registri su samo za čitanje
        }
    }

    void start() {
        setIrq(false);
        if (w_ < 1 || h_ < 1 || w_ > (uint32_t)BFS_MAX_DIM || h_ > (uint32_t)BFS_MAX_DIM ||
            w_ * h_ > (uint32_t)BFS_MAX_PIXELS) {
            status_ = ST_DONE | ST_ERR_SIZE;
            if (irqEn_) setIrq(true);
            return;
        }
        status_ = ST_BUSY;                 // briše DONE i greške prethodnog pokretanja
        start_ev_.notify(sc_core::SC_ZERO_TIME);
    }

    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& /*delay*/) {
        const uint64_t addr = trans.get_address();
        if (trans.get_data_length() != 4 || (addr & 3) || addr >= ACC_SIZE ||
            trans.get_data_ptr() == nullptr) {
            trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }
        uint32_t value;
        if (trans.is_write()) {
            std::memcpy(&value, trans.get_data_ptr(), 4);
            regWrite((uint32_t)addr, value);
        } else {
            value = regRead((uint32_t)addr);
            std::memcpy(trans.get_data_ptr(), &value, 4);
        }
        trans.set_response_status(tlm::TLM_OK_RESPONSE);
    }

    // Obrada: funkcionalni rezultat se računa odmah (runBfs), a zatim se
    // simulirano vrijeme troši koliko je ciklusa jezgro izbrojalo.
    void run_thread() {
        for (;;) {
            sc_core::wait(start_ev_);
            PortB mem{bram_};
            BfsRun run = runBfs(mem, (int)w_, (int)h_, t_.bfs);
            sc_core::wait(sc_core::sc_time((double)run.cycles() * t_.clkNs, sc_core::SC_NS));

            res_ = run.res;
            lastCycles_ = run.cycles();
            lastVisited_ = run.visitedPixels;
            status_ = ST_DONE;
            if (!res_.ok)       status_ |= ST_NO_CARD;
            if (res_.overflow)  status_ |= ST_OVERFLOW;
            if (irqEn_) setIrq(true);
        }
    }

    Bram& bram_;
    const Timing& t_;
    sc_core::sc_event start_ev_;
    sc_core::sc_event irq_ev_;
    bool irqLevel_ = false;
    uint32_t status_ = 0, w_ = 0, h_ = 0;
    bool irqEn_ = false;
    BfsResult res_;
    uint64_t lastCycles_ = 0, lastVisited_ = 0;
};

}  // namespace vp
