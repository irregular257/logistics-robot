/* Verify table frames pass through the real session/CRC/sequence parser. */
#define LOG_EXTERNAL_TABLE_ENABLED 1
#define main legacy_web_main
#include "../web_review/stm32_sequences.cpp"
#undef main

static LogisticsTableRequest simulated_request;
static LogisticsTableStatus received_table_status;
static unsigned received_table_count;
uint8_t Logistics_TableReadRequest(LogisticsTableRequest *out)
{
    *out=simulated_request; return out->operation!=0U;
}
void Logistics_TableReceive(const LogisticsTableStatus *status)
{
    received_table_status=*status; ++received_table_count;
}
static size_t count_reply(uint8_t command)
{
    size_t count=0U;
    for (size_t i=0;i<replies.size();++i) if (replies[i].command==command) ++count;
    return count;
}
int main(void)
{
    simulated_request={};
    if (legacy_web_main()!=0) return 1;
    reset(); received_table_count=0U;
    request(WEB_CMD_HELLO,123U,1U);
    simulated_request.operation=WEB_TABLE_OP_HOLD;
    simulated_request.task_id=0x12345678U; simulated_request.window=0x87654321U;
    clock_ms+=WEB_TABLE_PERIOD_MS; WebControl_Task();
    CHECK(count_reply(WEB_CMD_TABLE_REQUEST)==1U);
    const WebFrame *table_frame=nullptr;
    for (size_t i=0;i<replies.size();++i)
        if (replies[i].command==WEB_CMD_TABLE_REQUEST) table_frame=&replies[i];
    CHECK(table_frame && table_frame->session==123U && table_frame->length==9U);
    if (table_frame) {
        CHECK(table_frame->payload[0]==WEB_TABLE_OP_HOLD);
        CHECK(Web_Get32(table_frame->payload+1)==simulated_request.task_id);
        CHECK(Web_Get32(table_frame->payload+5)==simulated_request.window);
    }
    clock_ms+=WEB_TABLE_PERIOD_MS; WebControl_Task();
    CHECK(count_reply(WEB_CMD_TABLE_REQUEST)==2U);

    uint8_t payload[WEB_TABLE_STATUS_SIZE]={WEB_TABLE_HELD,1U,0U};
    Web_Put32(payload+3,simulated_request.task_id); Web_Put32(payload+7,simulated_request.window);
    Web_Put32(payload+11,0x11223344U);
    size_t acknowledgments=count_reply(WEB_CMD_ACK);
    request(WEB_CMD_TABLE_STATUS,123U,2U,(const int8_t *)payload,sizeof(payload));
    CHECK(received_table_count==1U && count_reply(WEB_CMD_ACK)==acknowledgments);
    CHECK(received_table_status.state==WEB_TABLE_HELD && received_table_status.picked==1U);
    CHECK(received_table_status.task_id==simulated_request.task_id);
    CHECK(received_table_status.window==simulated_request.window);
    CHECK(received_table_status.boot_id==0x11223344U);
    CHECK(received_table_status.received_at<=clock_ms && received_table_status.seen);
    request(WEB_CMD_TABLE_STATUS,123U,2U,(const int8_t *)payload,sizeof(payload));
    CHECK(received_table_count==1U); /* duplicate cannot refresh the hold lease */
    request(WEB_CMD_TABLE_STATUS,456U,3U,(const int8_t *)payload,sizeof(payload));
    CHECK(received_table_count==1U);
    request(WEB_CMD_TABLE_STATUS,123U,3U,(const int8_t *)payload,14U);
    CHECK(received_table_count==1U);
    payload[0]=7U; request(WEB_CMD_TABLE_STATUS,123U,4U,(const int8_t *)payload,sizeof(payload));
    CHECK(received_table_count==1U);
    payload[0]=WEB_TABLE_HELD; payload[2]=1U;
    request(WEB_CMD_TABLE_STATUS,123U,5U,(const int8_t *)payload,sizeof(payload));
    CHECK(received_table_count==2U && received_table_status.fault==1U);
    printf("External table UART session/sequence/serialization review: %u failure(s)\n",failures);
    return failures ? 1 : 0;
}
