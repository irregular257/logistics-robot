#include "turntable_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
using namespace table_now;
static Packet request(Type type,uint32_t seq,uint32_t window=0) {
    Packet p=makePacket(type); p.tableBoot=11; p.carBoot=22; p.run=33;
    p.sequence=seq; p.window=window; return p;
}
static void heartbeat(TableEngine &e,uint32_t now) { e.handle(request(HEARTBEAT,0),now); }
static void waitTo(TableEngine &e,uint32_t begin,uint32_t end) {
    for (uint32_t t=begin;t<end;t+=100) { heartbeat(e,t); e.tick(t); }
    e.tick(end);
}
int main() {
    TableEngine unarmed(11);
    assert(unarmed.handle(request(START,1),0).result==BAD_STATE);
    TableEngine e(11); e.confirmPosition();
    assert(e.handle(request(START,1),0).state==SETTLING);
    e.tick(200); assert(e.state()==READY && e.window()==1);
    Packet hold=request(HOLD,2,1);
    assert(e.handle(hold,300).state==HELD);
    assert(e.handle(hold,350).state==HELD);
    Packet done=request(PICK_DONE,3,1);
    assert(e.handle(done,400).picked==1);
    assert(e.handle(done,410).picked==1); // Lost ACK retry must not count twice.
    assert(e.state()==MOVING);
    e.moveComplete(500); heartbeat(e,600); e.tick(700); assert(e.window()==2);
    assert(e.handle(request(HOLD,4,1),710).result==STALE_WINDOW);
    assert(e.state()==READY);
    e.handle(request(HOLD,5,2),720); e.handle(request(PICK_DONE,6,2),730);
    e.moveComplete(800); heartbeat(e,900); e.tick(1000);
    e.handle(request(HOLD,7,3),1010); e.handle(request(PICK_DONE,8,3),1020);
    assert(e.state()==DONE && e.picked()==3);
    e.handle(request(PICK_DONE,8,3),1030); assert(e.picked()==3);
    // Old task/session cannot claim a new run or modify a restarted board.
    TableEngine rebooted(12); rebooted.confirmPosition();
    assert(rebooted.handle(hold,100).result==BAD_SESSION);
    // A command at exactly the dwell deadline cannot reopen the old window.
    TableEngine boundary(11); boundary.confirmPosition(); boundary.handle(request(START,1),0);
    boundary.tick(200); waitTo(boundary,300,2100);
    auto late=boundary.handle(request(HOLD,2,1),2200);
    assert(late.result==STALE_WINDOW && boundary.state()==MOVING);
    // Heartbeat loss during motion stops in fault and does not auto-resume.
    boundary.tick(2800); assert(boundary.state()==FAULT);
    heartbeat(boundary,2801); assert(boundary.state()==FAULT);
    assert(boundary.handle(request(START,3),2802).result==BAD_STATE);
    // Old sequence or a different operation with a reused sequence is rejected.
    assert(e.handle(request(HOLD,7,3),1040).result==REPLAY);
    assert(e.handle(request(HOLD,8,3),1050).result==BAD_ARGUMENT);
    // Local elapsed arithmetic remains correct over millis() wrap.
    TableEngine wrap(11); wrap.confirmPosition();
    wrap.handle(request(START,1),0xFFFFFF80U); wrap.tick(0x48U);
    assert(wrap.state()==READY);
    wrap.handle(request(HOLD,2,1),0x49U); assert(wrap.state()==HELD);
    wrap.tick(0x49U+LINK_TIMEOUT_MS); assert(wrap.state()==FAULT);
    // Re-ACKing a duplicate must report the CURRENT fault after a deadline,
    // rather than replaying the earlier HELD snapshot that allowed a grasp.
    TableEngine expiredLink(11); expiredLink.confirmPosition();
    expiredLink.handle(request(START,1),0); expiredLink.tick(200);
    expiredLink.handle(request(HOLD,2,1),300);
    auto expiredReply=expiredLink.handle(request(HOLD,2,1),900);
    assert(expiredReply.state==FAULT && expiredLink.state()==FAULT);
    // Even healthy heartbeats cannot extend a HOLD forever. A timeout stops
    // the task and later heartbeat/duplicate HOLD traffic cannot resume it.
    TableEngine expiredHold(11); expiredHold.confirmPosition();
    expiredHold.handle(request(START,1),0); expiredHold.tick(200);
    expiredHold.handle(request(HOLD,2,1),300);
    waitTo(expiredHold,400,300+HOLD_TIMEOUT_MS-1);
    assert(expiredHold.state()==HELD);
    auto holdTimeoutReply=expiredHold.handle(request(HOLD,2,1),300+HOLD_TIMEOUT_MS);
    assert(holdTimeoutReply.state==FAULT && expiredHold.picked()==0);
    heartbeat(expiredHold,301+HOLD_TIMEOUT_MS); assert(expiredHold.state()==FAULT);
    // The last instant INSIDE the window is valid; a repeated START never
    // restarts the two-second clock or changes the active window.
    TableEngine beforeBoundary(11); beforeBoundary.confirmPosition();
    beforeBoundary.handle(request(START,1),0); beforeBoundary.tick(200);
    waitTo(beforeBoundary,300,2100);
    auto startRetry=beforeBoundary.handle(request(START,1),2150);
    assert(startRetry.state==READY && startRetry.window==1);
    assert(beforeBoundary.handle(request(HOLD,2,1),2199).state==HELD);
    Packet otherCar=request(PICK_DONE,3,1); otherCar.carBoot=23;
    assert(beforeBoundary.handle(otherCar,2200).result==BAD_SESSION);
    assert(beforeBoundary.state()==HELD && beforeBoundary.picked()==0);
    // Six 60-degree targets must total one revolution for nondivisible counts.
    for (uint32_t pulses : {200U,3200U,6400U}) {
        SectorSteps s(pulses); uint32_t sum=0;
        for (unsigned i=0;i<6;++i) sum+=s.next(); assert(sum==pulses);
        assert(s.next()==(pulses+3)/6);
        s.reset(); sum=0;
        for (unsigned i=0;i<600;++i) sum+=s.next(); assert(sum==100*pulses);
    }
    puts("turntable state machine: all checks passed");
}
