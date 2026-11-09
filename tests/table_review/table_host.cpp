/* Exercise the production STM32 logistics state machine with the external
 * table enabled, using the existing motor/arm simulator. */
#define LOG_EXTERNAL_TABLE_ENABLED 1
#define main legacy_logistics_main
#include "../logistics_host.cpp"
#undef main

static uint32_t sim_task,sim_window,sim_boot,sim_move_until;
static uint8_t sim_state,sim_picked,sim_fault;
static bool sim_silent,sim_hold_ack,sim_pick_ack;
static unsigned sim_pick_commits;

static void table_reset(void)
{
    sim_task=0U; sim_window=1U; sim_boot=1234U; sim_move_until=0U;
    sim_state=WEB_TABLE_READY; sim_picked=sim_fault=0U;
    sim_silent=false; sim_hold_ack=sim_pick_ack=true; sim_pick_commits=0U;
    reset();
}
static void table_service(void)
{
    LogisticsTableRequest request;
    if (Logistics_TableReadRequest(&request)) {
        if (request.operation==WEB_TABLE_OP_START) {
            if (!sim_task) sim_task=request.task_id;
        } else if (request.operation==WEB_TABLE_OP_HOLD && sim_hold_ack &&
                   sim_state==WEB_TABLE_READY && request.window==sim_window) {
            sim_state=WEB_TABLE_HELD;
        } else if (request.operation==WEB_TABLE_OP_PICK_DONE && sim_pick_ack &&
                   sim_state==WEB_TABLE_HELD && request.window==sim_window) {
            ++sim_picked; ++sim_pick_commits;
            assert(!arm_out && !holding && stores==sim_picked && !lift_active);
            sim_state=sim_picked==3U ? WEB_TABLE_DONE : WEB_TABLE_MOVING;
            sim_move_until=clock_ms+200U;
        } else if (request.operation==WEB_TABLE_OP_ABORT) sim_state=WEB_TABLE_IDLE;
    }
    if (sim_state==WEB_TABLE_MOVING && clock_ms>=sim_move_until) {
        sim_state=WEB_TABLE_READY; ++sim_window;
    }
    if (!sim_silent) {
        LogisticsTableStatus status={};
        status.state=sim_state; status.picked=sim_picked; status.fault=sim_fault;
        status.task_id=sim_task; status.window=sim_window; status.boot_id=sim_boot;
        status.received_at=clock_ms; status.seen=1U;
        Logistics_TableReceive(&status);
    }
}
static void table_step(void)
{
    advance(10U); table_service(); Logistics_Task();
}
static void table_run(uint32_t duration)
{
    uint32_t deadline=clock_ms+duration;
    while (clock_ms<deadline) table_step();
}
static void until_state(uint8_t state,uint32_t limit=15000U)
{
    uint32_t deadline=clock_ms+limit;
    while (clock_ms<deadline && logistics_state!=state && Logistics_IsRunning()) table_step();
    assert(logistics_state==state);
}
static void assert_abort(void)
{
    LogisticsTableRequest request;
    assert(Logistics_TableReadRequest(&request) && request.operation==WEB_TABLE_OP_ABORT);
    assert(!lift_active && !observing);
    for (unsigned i=0;i<4U;++i) assert(rpm[i]==0);
    table_run(500U);
    assert(Logistics_TableReadRequest(&request) && request.operation==WEB_TABLE_OP_ABORT);
}
int main(void)
{
    /* A detection can only request HOLD. It never authorizes lowering. */
    table_reset(); sim_hold_ack=false; assert(Logistics_Start());
    until_state(LOG_TABLE_WAIT_HELD);
    table_run(250U);
    assert(logistics_state==LOG_TABLE_WAIT_HELD && !lift_active && !holding);
    sim_hold_ack=true; table_run(200000U);
    assert(logistics_state==LOG_DONE && logistics_error==0U && drops==3U);
    assert(sim_pick_commits==3U && sim_picked==3U && started_observing==3U);

    /* Matching both attributes is required to request HOLD, even though
     * the external table has already announced a stationary READY window. */
    table_reset(); wrong_color=2U; assert(Logistics_Start());
    until_state(LOG_TABLE_SCAN); table_run(300U);
    LogisticsTableRequest pending;
    assert(Logistics_TableReadRequest(&pending) && pending.operation==WEB_TABLE_OP_START);
    assert(!lift_active && !holding);
    wrong_color=0U; wrong_shape=3U; table_run(300U);
    assert(logistics_state==LOG_TABLE_SCAN && !lift_active && !holding);
    wrong_shape=0U; table_run(200000U);
    assert(logistics_state==LOG_DONE && sim_pick_commits==3U);

    /* No telemetry at startup: no detector and no grasp. */
    table_reset(); sim_silent=true; assert(Logistics_Start()); table_run(3500U);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_TABLE_LINK);
    assert(started_observing==0U); assert_abort();

    /* Fresh READY with the wrong task cannot refresh the startup timeout. */
    table_reset(); assert(Logistics_Start()); sim_task=0xdeadbeefU; table_run(3500U);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_TABLE_LINK);
    assert_abort();

    table_reset(); sim_hold_ack=false; assert(Logistics_Start());
    until_state(LOG_TABLE_WAIT_HELD); table_run(LOG_TABLE_HOLD_TIMEOUT_MS+200U);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_TABLE_TIMEOUT);
    assert(stores==0U && !holding); assert_abort();

    table_reset(); sim_hold_ack=false; assert(Logistics_Start());
    until_state(LOG_TABLE_WAIT_HELD); sim_state=WEB_TABLE_HELD; ++sim_window;
    table_run(300U);
    assert(logistics_state==LOG_TABLE_WAIT_HELD && !lift_active && !holding);
    table_run(LOG_TABLE_HOLD_TIMEOUT_MS);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_TABLE_TIMEOUT);
    assert_abort();

    /* A late HOLD is abandoned without a motor pulse toward the material. */
    table_reset(); sim_hold_ack=false; assert(Logistics_Start());
    until_state(LOG_TABLE_WAIT_HELD);
    sim_state=WEB_TABLE_MOVING; sim_move_until=clock_ms+300U;
    table_run(160U);
    assert(logistics_state==LOG_TABLE_WAIT_READY && !lift_active && !holding && !observing);
    /* The simulator expects one detector start per planned item. Reset its
     * counter for this deliberately repeated window of the first item. */
    started_observing=0U;
    sim_hold_ack=true; table_run(200000U);
    assert(logistics_state==LOG_DONE && drops==3U && sim_pick_commits==3U);

    /* Wireless loss, old HELD status, a reboot and a wrong window each stop
     * an already moving lift. The last case has otherwise fresh telemetry. */
    for (unsigned scenario=0U;scenario<4U;++scenario) {
        table_reset(); assert(Logistics_Start()); until_state(LOG_PICK_LOWER);
        assert(lift_active);
        if (scenario==0U) sim_fault=1U;
        if (scenario==1U) sim_silent=true;
        if (scenario==2U) ++sim_boot;
        if (scenario==3U) ++sim_window;
        table_run(450U);
        assert(logistics_state==LOG_FAULT && stores==0U && drops==0U);
        assert(logistics_error==(scenario==3U ? LOG_ERROR_TABLE_STATE : LOG_ERROR_TABLE_LINK));
        assert_abort();
    }

    table_reset(); assert(Logistics_Start()); until_state(LOG_GRIP);
    assert(holding); sim_state=WEB_TABLE_MOVING; sim_move_until=clock_ms+1000U;
    table_run(100U);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_TABLE_STATE && holding);
    assert_abort();

    /* Completion must be acknowledged before the next target or route. */
    table_reset(); sim_pick_ack=false; assert(Logistics_Start());
    until_state(LOG_TABLE_WAIT_PICK); table_run(500U);
    assert(logistics_state==LOG_TABLE_WAIT_PICK && started_observing==1U && stores==1U);
    assert(sim_pick_commits==0U && !holding);
    sim_pick_ack=true; table_run(200000U);
    assert(logistics_state==LOG_DONE && sim_pick_commits==3U);

    table_reset(); sim_pick_ack=false; assert(Logistics_Start());
    until_state(LOG_TABLE_WAIT_PICK); table_run(LOG_TABLE_PICK_TIMEOUT_MS+200U);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_TABLE_TIMEOUT);
    assert(stores==1U && started_observing==1U); assert_abort();

    table_reset(); assert(Logistics_Start()); until_state(LOG_TABLE_WAIT_HELD);
    Logistics_Cancel(); assert(logistics_state==LOG_ABORTED); assert_abort();
    puts("PASS: external table HOLD interlock, three pickups after clearance, late-window recovery, stale/task/boot/window/link rejection, completion ACK gating, repeated ABORT");
    return 0;
}
