#pragma once
#include <stdint.h>
#include <string.h>

namespace table_now {
constexpr uint32_t MAGIC = 0x31544252UL;
constexpr uint8_t VERSION = 1;
constexpr uint32_t DWELL_MS = 2000;
constexpr uint32_t SETTLE_MS = 200;
constexpr uint32_t LINK_TIMEOUT_MS = 600;
constexpr uint32_t HOLD_TIMEOUT_MS = 60000;
enum Type : uint8_t { STATUS=1, START=2, HOLD=3, PICK_DONE=4, ABORT=5, HEARTBEAT=6 };
enum State : uint8_t { IDLE=0, MOVING=1, SETTLING=2, READY=3, HELD=4, DONE=5, FAULT=6 };
enum Result : uint8_t { OK=0, BAD_SESSION=1, BAD_STATE=2, STALE_WINDOW=3, REPLAY=4, BAD_ARGUMENT=5 };
#pragma pack(push,1)
struct Packet {
    uint32_t magic;
    uint8_t version, type, state, picked;
    uint32_t tableBoot, carBoot, run, sequence, window, ack;
    uint8_t result, reserved[3];
};
#pragma pack(pop)
static_assert(sizeof(Packet)==36, "Unexpected ESP-NOW packet layout");
inline Packet makePacket(Type type) {
    Packet p={}; p.magic=MAGIC; p.version=VERSION; p.type=type; return p;
}
inline bool validPacket(const Packet &p) {
    return p.magic==MAGIC && p.version==VERSION && p.type>=STATUS &&
           p.type<=HEARTBEAT && p.state<=FAULT && p.picked<=3 &&
           p.result<=BAD_ARGUMENT && !p.reserved[0] && !p.reserved[1] && !p.reserved[2];
}
inline bool newer32(uint32_t value, uint32_t previous) { return (int32_t)(value-previous)>0; }

// No Arduino dependencies: the same rules are exercised by the host tests.
class TableEngine {
public:
    explicit TableEngine(uint32_t boot) : boot_(boot) {}
    void confirmPosition() { armed_=true; }
    State state() const { return state_; }
    uint32_t window() const { return window_; }
    uint8_t picked() const { return picked_; }
    void fault() { state_=FAULT; }
    void moveComplete(uint32_t now) {
        if (state_==MOVING) { state_=SETTLING; entered_=now; }
    }
    void tick(uint32_t now) {
        if (state_==IDLE || state_==DONE || state_==FAULT) return;
        if ((uint32_t)(now-heartbeat_)>=LINK_TIMEOUT_MS) { fault(); return; }
        if (state_==HELD && (uint32_t)(now-entered_)>=HOLD_TIMEOUT_MS) { fault(); return; }
        if (state_==SETTLING && (uint32_t)(now-entered_)>=SETTLE_MS) {
            if (++window_==0) { fault(); return; }
            state_=READY; entered_=now;
        } else if (state_==READY && (uint32_t)(now-entered_)>=DWELL_MS) {
            state_=MOVING; entered_=now;
        }
    }
    Packet status() {
        Packet p=makePacket(STATUS);
        p.tableBoot=boot_; p.carBoot=car_; p.run=run_; p.sequence=++statusSequence_;
        p.window=window_; p.state=state_; p.picked=picked_;
        p.ack=lastSequence_; p.result=lastResult_; return p;
    }
    Packet handle(const Packet &p,uint32_t now) {
        // Apply local deadlines before considering a command at a boundary.
        tick(now);
        if (!validPacket(p) || p.type==STATUS || !p.tableBoot || !p.carBoot || !p.run)
            return reply(p.sequence,BAD_ARGUMENT);
        if (p.tableBoot!=boot_) return reply(p.sequence,BAD_SESSION);
        if (p.type==START && (state_==IDLE || state_==DONE) &&
            (p.carBoot!=car_ || p.run!=run_)) {
            if (!armed_) return reply(p.sequence,BAD_STATE);
            if (!p.sequence || p.window) return reply(p.sequence,BAD_ARGUMENT);
            car_=p.carBoot; run_=p.run; picked_=0; window_=0;
            state_=SETTLING; entered_=heartbeat_=now;
            cache(p,OK); return status();
        }
        if (p.carBoot!=car_ || p.run!=run_) return reply(p.sequence,BAD_SESSION);
        if (p.type==HEARTBEAT) { heartbeat_=now; return status(); }
        if (p.sequence==lastSequence_) {
            if (p.type==lastCommand_.type && p.window==lastCommand_.window)
                return status(); // Re-ACK without executing a second time.
            return reply(p.sequence,BAD_ARGUMENT);
        }
        if (!p.sequence || !newer32(p.sequence,lastSequence_)) return reply(p.sequence,REPLAY);
        Result result=OK;
        if (p.type==ABORT) {
            fault();
        } else if (p.type==HOLD) {
            if (p.window!=window_ || state_==MOVING || state_==SETTLING)
                result=STALE_WINDOW;
            else if (state_!=READY) result=BAD_STATE;
            else { state_=HELD; entered_=now; }
        } else if (p.type==PICK_DONE) {
            if (p.window!=window_) result=STALE_WINDOW;
            else if (state_!=HELD) result=BAD_STATE;
            else { ++picked_; state_=picked_==3 ? DONE : MOVING; entered_=now; }
        } else result=BAD_STATE;
        if (result==OK) heartbeat_=now;
        cache(p,result); return status();
    }
private:
    uint32_t boot_,car_=0,run_=0,window_=0,lastSequence_=0,statusSequence_=0;
    uint32_t entered_=0,heartbeat_=0;
    State state_=IDLE;
    uint8_t picked_=0;
    Result lastResult_=OK;
    bool armed_=false;
    Packet lastCommand_={};
    void cache(const Packet &p,Result r) { lastSequence_=p.sequence; lastResult_=r; lastCommand_=p; }
    Packet reply(uint32_t seq,Result r) { Packet p=status(); p.ack=seq; p.result=r; return p; }
};

// Rounded absolute sector targets avoid 533-pulse-per-sector accumulation drift.
class SectorSteps {
public:
    explicit SectorSteps(uint32_t stepsPerRev) : steps_(stepsPerRev) {}
    uint32_t next() {
        uint32_t before=(uint64_t(sector_)*steps_+3)/6;
        uint32_t after=(uint64_t(sector_+1)*steps_+3)/6;
        sector_=(sector_+1)%6; return after-before;
    }
    void reset() { sector_=0; }
private:
    uint32_t steps_; uint8_t sector_=0;
};
} // namespace table_now
