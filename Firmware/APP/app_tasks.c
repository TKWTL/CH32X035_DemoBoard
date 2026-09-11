#include "app_tasks.h"
#include "coroOS.h"
#include "debug.h"
#include "pd.h"
#include "board.h"
#include "ina226.h"
#include "ssd1306_u8g2.h"
#include "ws2812.h"
#include "ws2812_effect.h"
#include "i2c_api.h"
#include "time_api.h"
#include "usart_async.h"

static coro_scheduler_t s_scheduler;
static uint8_t s_ws_ready;
static uint8_t s_ws_effect_active;
static uint8_t s_ssd1306_online;
static uint8_t s_bus_measure_valid;
static uint16_t s_bus_mv;
static int32_t s_bus_current_mA_x100;

static void APP_PrintResetCause(void)
{
    uint8_t any = 0u;

    printf("[RESET] cause:");

    if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
    {
        printf(" POR/PDR");
        any = 1u;
    }
    if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
    {
        printf(" PIN");
        any = 1u;
    }
    if(RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET)
    {
        printf(" SW");
        any = 1u;
    }
    if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        printf(" IWDG");
        any = 1u;
    }
    if(RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)
    {
        printf(" WWDG");
        any = 1u;
    }
    if(RCC_GetFlagStatus(RCC_FLAG_LPWRRST) != RESET)
    {
        printf(" LPWR");
        any = 1u;
    }

    if(!any)
        printf(" unknown/cleared");

    printf("\r\n");
    RCC_ClearFlag();
}

static void APP_PrintINA226Raw(uint16_t bus_raw, uint16_t shunt_raw)
{
    INA226_Measurement m;
    int32_t cur;
    int32_t shu;
    char cur_sign = '+';
    char shu_sign = '+';
    uint32_t cur_abs;
    uint32_t shu_abs;

    INA226_ConvertMeasurement(bus_raw, shunt_raw, &m);
    cur = m.current_mA_x100;
    shu = m.shunt_uV_x10;

    if(cur < 0) { cur_sign = '-'; cur = -cur; }
    if(shu < 0) { shu_sign = '-'; shu = -shu; }
    cur_abs = (uint32_t)cur;
    shu_abs = (uint32_t)shu;

    printf("[INA226] VBUS=%lu.%03lu mV, VSHUNT=%c%lu.%01lu uV, I=%c%lu.%02lu mA\r\n",
           (unsigned long)(m.bus_uV / 1000UL),
           (unsigned long)(m.bus_uV % 1000UL),
           shu_sign, (unsigned long)(shu_abs / 10UL), (unsigned long)(shu_abs % 10UL),
           cur_sign, (unsigned long)(cur_abs / 100UL), (unsigned long)(cur_abs % 100UL));
}

static void APP_FormatBusVoltage(char line[11], uint16_t mv)
{
    uint16_t cv = (uint16_t)((mv + 5u) / 10u); /* 0.01 V */
    uint16_t volts = (uint16_t)(cv / 100u);
    uint16_t frac = (uint16_t)(cv % 100u);

    if(volts > 99u)
        volts = 99u;

    line[0] = 'B'; line[1] = 'U'; line[2] = 'S'; line[3] = ' ';
    line[4] = (char)('0' + (volts / 10u));
    line[5] = (char)('0' + (volts % 10u));
    line[6] = '.';
    line[7] = (char)('0' + (frac / 10u));
    line[8] = (char)('0' + (frac % 10u));
    line[9] = 'V';
    line[10] = '\0';
}

static void APP_FormatBusCurrent(char line[11], int32_t current_mA_x100)
{
    uint32_t abs_i;
    uint16_t ca; /* 0.01 A */

    if(current_mA_x100 < 0)
    {
        line[4] = '-';
        abs_i = (uint32_t)(-current_mA_x100);
    }
    else
    {
        line[4] = '+';
        abs_i = (uint32_t)current_mA_x100;
    }

    ca = (uint16_t)((abs_i + 500u) / 1000u);
    if(ca > 999u)
        ca = 999u;

    line[0] = 'B'; line[1] = 'U'; line[2] = 'S'; line[3] = ' ';
    line[5] = (char)('0' + (ca / 100u));
    line[6] = '.';
    line[7] = (char)('0' + ((ca / 10u) % 10u));
    line[8] = (char)('0' + (ca % 10u));
    line[9] = 'A';
    line[10] = '\0';
}

static void APP_FormatPDO(char line[11], const PD_DisplayPDO *pdo)
{
    uint16_t volts = (uint16_t)((pdo->max_mv + 500u) / 1000u);

    if(volts > 99u)
        volts = 99u;

    if(pdo->type == PD_DISPLAY_PDO_AVS)
    {
        uint16_t w = pdo->power_w;
        if(w > 999u) w = 999u;
        line[0] = 'A'; line[1] = 'V'; line[2] = 'S';
        line[3] = (volts >= 10u) ? (char)('0' + (volts / 10u)) : ' ';
        line[4] = (char)('0' + (volts % 10u));
        line[5] = ' ';
        line[6] = (w >= 100u) ? (char)('0' + (w / 100u)) : ' ';
        line[7] = (w >= 10u) ? (char)('0' + ((w / 10u) % 10u)) : ' ';
        line[8] = (char)('0' + (w % 10u));
        line[9] = 'W';
        line[10] = '\0';
        return;
    }

    if(pdo->type == PD_DISPLAY_PDO_EPR)
    {
        line[0] = 'E'; line[1] = 'P'; line[2] = 'R';
    }
    else if(pdo->type == PD_DISPLAY_PDO_PPS)
    {
        line[0] = 'P'; line[1] = 'P'; line[2] = 'S';
    }
    else
    {
        line[0] = 'S'; line[1] = 'P'; line[2] = 'R';
    }

    {
        uint16_t da = (uint16_t)((pdo->current_ma + 50u) / 100u); /* 0.1 A */
        if(da > 99u) da = 99u;
        line[3] = (volts >= 10u) ? (char)('0' + (volts / 10u)) : ' ';
        line[4] = (char)('0' + (volts % 10u));
        line[5] = ' ';
        line[6] = (char)('0' + (da / 10u));
        line[7] = '.';
        line[8] = (char)('0' + (da % 10u));
        line[9] = 'A';
        line[10] = '\0';
    }
}

static void APP_DrawSSD1306Dashboard(void)
{
    static const uint8_t baseline[5] = { 11u, 24u, 37u, 50u, 63u };
    PD_DisplayPDO pdo[5];
    uint8_t pdo_count;
    uint8_t i;
    char line[11];
    u8g2_t *u8g2 = SSD1306_U8G2_Get();

    u8g2_ClearBuffer(u8g2);
    u8g2_SetFont(u8g2, SSD1306_U8G2_DEFAULT_FONT);

    /* 128x64 -> two 64-pixel columns. Five 12-pixel text rows plus four
     * one-pixel separators exactly consume the 64-pixel height. */
    //for(i = 0u; i < 4u; ++i)
    //    u8g2_DrawHLine(u8g2, 0u, (u8g2_uint_t)(12u + 13u * i), 128u);
    //u8g2_DrawVLine(u8g2, 64u, 0u, 64u);

    /* Left slot 0: compact inverted welcome banner. */
    u8g2_DrawBox(u8g2, 8u, 2u, 48u, 8u);
    u8g2_SetDrawColor(u8g2, 0u);
    u8g2_DrawStr(u8g2, 8u, baseline[0], "CH32X035");
    u8g2_SetDrawColor(u8g2, 1u);
    u8g2_DrawStr(u8g2, 5u, baseline[1], "DemoBoard");

    if(s_bus_measure_valid)
    {
        APP_FormatBusVoltage(line, s_bus_mv);
        u8g2_DrawStr(u8g2, 1u, baseline[2], line);
        APP_FormatBusCurrent(line, s_bus_current_mA_x100);
        u8g2_DrawStr(u8g2, 1u, baseline[3], line);
    }
    else
    {
        u8g2_DrawStr(u8g2, 1u, baseline[2], "BUS --.--V");
        u8g2_DrawStr(u8g2, 1u, baseline[3], "BUS --.--A");
    }

    if(!PD_IsConnected())
    {
        u8g2_DrawStr(u8g2, 7u, baseline[4], "PD WAIT");
        /* No attached Source means no PDOs. Keep the right half completely
         * blank instead of exposing stale capability data from a prior attach. */
        return;
    }
    else if(!PD_IsPowerReady())
        u8g2_DrawStr(u8g2, 4u, baseline[4], "NEGOTIATE");
    else if(PD_IsEPRContractActive())
        u8g2_DrawStr(u8g2, 4u, baseline[4], "EPR READY");
    else
        u8g2_DrawStr(u8g2, 4u, baseline[4], "SPR READY");

    pdo_count = PD_GetDisplayPDOs(pdo, 5u);
    if(pdo_count == 0u)
    {
        u8g2_DrawStr(u8g2, 70u, baseline[0], "PDO WAIT");
        return;
    }

    for(i = 0u; i < pdo_count; ++i)
    {
        APP_FormatPDO(line, &pdo[i]);
        u8g2_DrawStr(u8g2, 67u, baseline[i], line);
    }
}

/* PD always runs first. Keep all I2C/display work outside pd.c/pd_port.c. */
THRD_DECLARE(thread_pd)
{
    THRD_BEGIN;
    while(1)
    {
        PD_Task(TIME_Millis());
        THRD_YIELD;
    }
    THRD_END;
}

/* I2C state-machine pump: one O(1) step per scheduler pass. Payload movement
 * is handled by DMA1 CH6/CH7, so this thread never waits on a hardware flag. */
THRD_DECLARE(thread_i2c_service)
{
    THRD_BEGIN;
    while(1)
    {
        I2C_API_Service();
        THRD_YIELD;
    }
    THRD_END;
}

/* Independent bus watchdog. Normal device NACK is not treated as a stuck bus;
 * timeout/BERR/ARLO/OVR request a peripheral reset + optional 9-clock recovery. */
THRD_DECLARE(thread_i2c_watchdog)
{
    static uint32_t last_recovery_count;
    static I2C_API_Diagnostic d;

    THRD_BEGIN;
    last_recovery_count = I2C_API_GetRecoveryCount();

    while(1)
    {
        THRD_DELAY(10u);
        I2C_API_WatchdogService(TIME_Millis());

        if(I2C_API_GetRecoveryCount() != last_recovery_count)
        {
            last_recovery_count = I2C_API_GetRecoveryCount();
            I2C_API_GetDiagnostic(&d);
            printf("[I2C] recovered bus #%lu: last=%s SCL=%u SDA=%u STAR1=0x%04x STAR2=0x%04x\r\n",
                   (unsigned long)last_recovery_count,
                   I2C_API_GetLastErrorName(),
                   d.scl_high, d.sda_high, d.star1, d.star2);
        }
    }
    THRD_END;
}

/* INA226 owns the bus only while a request is active. Every wait below returns
 * control to coroOS, so PD and other threads continue to run while I2C/DMA runs. */
THRD_DECLARE(thread_ina226)
{
    static uint16_t mfr;
    static uint16_t die;
    static uint16_t bus_raw;
    static uint16_t shunt_raw;
    static uint16_t vbus_mv;
    static uint8_t current_div;
    static uint8_t log_div;
    static uint8_t online;
    static I2C_API_Result result;
    static INA226_Measurement m;

    THRD_BEGIN;
    online = 0u;
    current_div = 0u;
    log_div = 0u;
    s_bus_measure_valid = 0u;

    while(1)
    {
        if(!online)
        {
            THRD_UNTIL(INA226_BeginPing());
            THRD_UNTIL(INA226_IsTransferComplete());
            result = INA226_TakePingResult();
            if(result != I2C_API_RESULT_OK)
            {
                s_bus_measure_valid = 0u;
                THRD_DELAY(1000u);
                continue;
            }

            THRD_UNTIL(INA226_BeginReadReg16(INA226_REG_MFR_ID));
            THRD_UNTIL(INA226_IsTransferComplete());
            result = INA226_TakeReadReg16(&mfr);
            if(result != I2C_API_RESULT_OK)
            {
                s_bus_measure_valid = 0u;
                THRD_DELAY(1000u);
                continue;
            }

            THRD_UNTIL(INA226_BeginReadReg16(INA226_REG_DIE_ID));
            THRD_UNTIL(INA226_IsTransferComplete());
            result = INA226_TakeReadReg16(&die);
            if(result != I2C_API_RESULT_OK)
            {
                s_bus_measure_valid = 0u;
                THRD_DELAY(1000u);
                continue;
            }

            online = 1u;
            printf("[INA226] detected async: MFR=0x%04x DIE=0x%04x, I2C=400kHz DMA RX/TX\r\n",
                   mfr, die);
        }

        /* Keep the PD VBUS feed at 50 ms. Current is refreshed every 200 ms,
         * which is fast enough for the front panel without creating needless
         * I2C traffic. */
        THRD_DELAY(50u);
        THRD_UNTIL(INA226_BeginReadReg16(INA226_REG_BUS));
        THRD_UNTIL(INA226_IsTransferComplete());
        result = INA226_TakeReadReg16(&bus_raw);
        if(result != I2C_API_RESULT_OK)
        {
            online = 0u;
            s_bus_measure_valid = 0u;
            continue;
        }

        vbus_mv = INA226_BusRawToMv(bus_raw);
        s_bus_mv = vbus_mv;
        PD_SetVbusMillivolts(vbus_mv);

        current_div++;
        log_div++;
        if(current_div < 4u)
            continue;
        current_div = 0u;

        THRD_UNTIL(INA226_BeginReadReg16(INA226_REG_SHUNT));
        THRD_UNTIL(INA226_IsTransferComplete());
        result = INA226_TakeReadReg16(&shunt_raw);
        if(result != I2C_API_RESULT_OK)
        {
            online = 0u;
            s_bus_measure_valid = 0u;
            continue;
        }

        INA226_ConvertMeasurement(bus_raw, shunt_raw, &m);
        s_bus_current_mA_x100 = m.current_mA_x100;
        s_bus_measure_valid = 1u;

        if(log_div >= 20u)
        {
            log_div = 0u;
            /* Keep a low-rate measurement heartbeat even while detached.
             * This makes an externally powered/no-charger board visibly alive
             * instead of appearing to freeze after the INA226 probe log. */
            APP_PrintINA226Raw(bus_raw, shunt_raw);
        }
    }
    THRD_END;
}

/* SSD1306 hot-plug monitor + dashboard. It runs independently of PD state so
 * an externally powered board overwrites any stale SSD1306 GDDRAM immediately
 * after reset/detach. Two successful probes are required before initialization;
 * once online, the periodic framebuffer write doubles as hot-unplug detection. */
THRD_DECLARE(thread_ssd1306)
{
    static uint8_t address;
    static uint8_t consecutive_ok;
    static uint32_t known_recovery_count;
    static I2C_API_Result result;

    THRD_BEGIN;
    address = SSD1306_I2C_ADDR_PRIMARY;
    consecutive_ok = 0u;
    s_ssd1306_online = 0u;
    known_recovery_count = I2C_API_GetRecoveryCount();

    while(1)
    {
        if(s_ssd1306_online)
        {
            /* A real framebuffer write also acts as the online liveness check,
             * so a separate ping is unnecessary once the panel is present. */
            THRD_DELAY(200u);

            if(I2C_API_GetRecoveryCount() != known_recovery_count)
            {
                known_recovery_count = I2C_API_GetRecoveryCount();
                printf("[SSD1306] I2C recovery occurred; re-probing display\r\n");
                s_ssd1306_online = 0u;
                consecutive_ok = 0u;
                address = SSD1306_I2C_ADDR_PRIMARY;
                continue;
            }

            APP_DrawSSD1306Dashboard();
            if(!SSD1306_U8G2_BeginFlush())
                continue;
            THRD_UNTIL((result = SSD1306_U8G2_PollFlush()) != I2C_API_RESULT_ACTIVE);
            if(result != I2C_API_RESULT_OK)
            {
                printf("[SSD1306] display write failed/NACK; display offline\r\n");
                s_ssd1306_online = 0u;
                consecutive_ok = 0u;
                address = SSD1306_I2C_ADDR_PRIMARY;
            }
            continue;
        }

        if(I2C_API_GetRecoveryCount() != known_recovery_count)
        {
            known_recovery_count = I2C_API_GetRecoveryCount();
            consecutive_ok = 0u;
        }

        THRD_UNTIL(SSD1306_U8G2_BeginProbe(address));
        THRD_UNTIL(SSD1306_U8G2_ProbeComplete());
        result = SSD1306_U8G2_TakeProbeResult();

        if(result != I2C_API_RESULT_OK && address == SSD1306_I2C_ADDR_PRIMARY)
        {
            address = SSD1306_I2C_ADDR_ALTERNATE;
            THRD_UNTIL(SSD1306_U8G2_BeginProbe(address));
            THRD_UNTIL(SSD1306_U8G2_ProbeComplete());
            result = SSD1306_U8G2_TakeProbeResult();
        }

        if(result != I2C_API_RESULT_OK)
        {
            consecutive_ok = 0u;
            address = SSD1306_I2C_ADDR_PRIMARY;
            /* No panel present: retain the low-rate hot-plug probe. */
            THRD_DELAY(500u);
            continue;
        }

        consecutive_ok++;
        if(consecutive_ok < 2u)
        {
            /* At boot, confirm a present panel quickly so old GDDRAM is
             * replaced without waiting another full hot-plug interval. */
            THRD_DELAY(20u);
            continue;
        }

        SSD1306_U8G2_SetAddress(address);
        THRD_UNTIL(SSD1306_U8G2_BeginPanelInit());
        THRD_UNTIL(SSD1306_U8G2_PanelInitComplete());
        result = SSD1306_U8G2_TakePanelInitResult();
        if(result != I2C_API_RESULT_OK)
        {
            consecutive_ok = 0u;
            address = SSD1306_I2C_ADDR_PRIMARY;
            continue;
        }

        APP_DrawSSD1306Dashboard();
        if(!SSD1306_U8G2_BeginFlush())
        {
            consecutive_ok = 0u;
            continue;
        }
        THRD_UNTIL((result = SSD1306_U8G2_PollFlush()) != I2C_API_RESULT_ACTIVE);
        if(result != I2C_API_RESULT_OK)
        {
            consecutive_ok = 0u;
            address = SSD1306_I2C_ADDR_PRIMARY;
            continue;
        }

        s_ssd1306_online = 1u;
        consecutive_ok = 0u;
        printf("[SSD1306] online at 0x%02x; 5+5 slot PDO dashboard, MiaoUI h12w6 font\r\n",
               address);
    }
    THRD_END;
}

THRD_DECLARE(thread_ws2812)
{
    THRD_BEGIN;
    while(1)
    {
        THRD_DELAY(WS2812_EFFECT_FRAME_MS);

        if(!s_ws_ready)
            continue;

        if(!PD_IsPowerReady())
        {
            if(s_ws_effect_active)
            {
                (void)WS2812_Fill(0u, 0u, 0u);
                WS2812_EffectInit();
                s_ws_effect_active = 0u;
                printf("[WS2812] PD power not stable; LEDs off\r\n");
            }
            continue;
        }

        if(!s_ws_effect_active)
        {
            WS2812_EffectInit();
            s_ws_effect_active = 1u;
            printf("[WS2812] PD power stable; chase/fade enabled\r\n");
        }

        if(WS2812_EffectStep() != 0u)
        {
            printf("[WS2812] runtime PIOC error=0x%02x; effect disabled\r\n",
                   WS2812_GetLastStatus());
            s_ws_ready = 0u;
            s_ws_effect_active = 0u;
        }
    }
    THRD_END;
}

static const coro_thread_fn_t s_threads[] =
{
    thread_pd,
    thread_i2c_service,
    thread_i2c_watchdog,
    thread_ina226,
    thread_ssd1306,
    thread_ws2812
};

static coro_pt_t s_thread_states[sizeof(s_threads) / sizeof(s_threads[0])];

void APP_Tasks_Init(void)
{
    uint8_t ws_status;

    printf("\r\n=== CH32X035 custom dev-board demo / coroOS ===\r\n");
    APP_PrintResetCause();
    printf("SystemClk:%lu Hz\r\n", (unsigned long)SystemCoreClock);
    printf("ChipID:%08lx\r\n", (unsigned long)DBGMCU_GetCHIPID());
    printf("UART1: PB10 TX / PB11 RX @ 921600, DMA async, TX/RX 256B\r\n");

    Board_Init();
    printf("[BOARD] PD Sink front-end enabled\r\n");

    I2C_API_Init(INA226_I2C_CLOCK_HZ);
    INA226_Init();
    SSD1306_U8G2_Init();
    printf("[I2C] I2C1 PA10/PA11 @ 400kHz, DMA1 CH6 TX / CH7 RX; async clients scheduled\r\n");

    ws_status = WS2812_Init();
    s_ws_ready = (ws_status == 0u) ? 1u : 0u;
    s_ws_effect_active = 0u;
    if(s_ws_ready)
    {
        WS2812_EffectInit();
        printf("[WS2812] ready: PIOC IO0 -> PC7, %u LEDs; held off until PD power stable\r\n",
               (unsigned)WS2812_LED_COUNT);
    }
    else
        printf("[WS2812] init failed: status=0x%02x; effect disabled\r\n", ws_status);

    PD_Init();
    printf("[PD] Sink started; SPR <= %u mV, EPR target %u mV / max %u mA\r\n",
           (unsigned)PD_REQUEST_MAX_FIXED_MV,
           (unsigned)PD_EPR_TARGET_MV,
           (unsigned)PD_EPR_REQUEST_MAX_MA);
    printf("[PD] EPR Sink Operational PDP=%u W; status=PD WAIT, waiting for CC attach\r\n\r\n",
           (unsigned)PD_EPR_SINK_PDP_W);

    CoroOS_Init(&s_scheduler,
                s_threads,
                s_thread_states,
                (uint8_t)(sizeof(s_threads) / sizeof(s_threads[0])));
}

void APP_Tasks_RunOnce(void)
{
    USART1_Async_Service();
    CoroOS_RunOnce(&s_scheduler);
}
