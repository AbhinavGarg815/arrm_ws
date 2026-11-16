/* st3215_test - bring-up firmware: safe per-joint rotation test for the three
 * ST3215 wrist servos on the Waveshare ESP32 Servo Driver Expansion Board.
 * st3215_arm.cpp is the production counterpart and is what the board normally
 * runs; this one is for finding and checking servos by hand.
 *
 * Safe at boot: broadcast torque-off, so a reset - which opening the USB port
 * can cause - ends with the wrist limp.
 *
 * Safety layers, all ending in broadcast torque-off:
 *   dead-man   no byte from the host for HOST_TIMEOUT_MS while a servo holds.
 *   load       |present load| above LOAD_LIMIT - a collision or a stall. The
 *              ST3215 measures its own load, so this is real contact sensing.
 *   bus        the held servo stops answering.
 *   fence      moves are relative, clamped to FENCE_DEG around the hold point
 *              and to the servo's own angle limits.
 *   one servo  only one servo holds torque at a time.
 *   duplicate  an ID that answered inconsistently during scan (two servos
 *              sharing it) can never be enabled - one command would move both.
 *   x / s      explicit stop.
 * The physical e-stop is still the only thing that removes power.
 *
 * Protocol: Feetech STS (ST3215), 1 Mbaud on GPIO18 (RX) / GPIO19 (TX).
 * Packet  FF FF id len inst params... chk,  chk = ~(id+len+inst+params)
 * Reply   FF FF id len err  params... chk,  multi-byte values little-endian.
 *
 * Host commands (115200 baud, one per line):
 *   scan         ping ids 1..20, report position/load/volt/temp/limits
 *   hold <id>    goal := present position, torque on - holds, no motion
 *   m <deg>      move the held servo by <deg>, relative, fenced
 *   x | s        STOP: broadcast torque off
 *   h            heartbeat (no reply)
 *   c            clear a latched fault
 *   t1 | t0 | t  telemetry on / off / toggle   T,ms,id,goal,pos,load
 * Replies: "## ..." ack, "!! ..." refusal or fault, "S,..." scan row.
 * ========================================================================== */

#include <Arduino.h>

/* ---------------------------------------------------------------- limits - */
static const float    STEPS_PER_DEG   = 4096.0f / 360.0f;
static const uint16_t TEST_SPEED      = 114;   /* steps/s  = 10 deg/s          */
static const uint8_t  TEST_ACC        = 5;     /* x100 steps/s^2 ~ 44 deg/s^2  */
/* Hard cap on how far a test may take a servo from its hold point. 45 deg is
 * enough to see the motion clearly (a 30 deg wrist-roll test was asked for)
 * while still keeping any single test a fraction of the servo's travel. */
static const float    FENCE_DEG       = 45.0f;
static const uint16_t LOAD_LIMIT      = 500;   /* 0..1000 = 0..100% of stall   */
static const uint16_t HOST_TIMEOUT_MS = 400;
static const uint8_t  SCAN_MAX_ID     = 20;

/* ------------------------------------------------------------------- bus - */
static const int      BUS_RX = 18, BUS_TX = 19;
static const uint32_t BUS_BAUD = 1000000;
static const uint8_t  BROADCAST = 0xFE;

enum : uint8_t { INST_PING = 0x01, INST_READ = 0x02, INST_WRITE = 0x03 };
enum : uint8_t {
    REG_MIN_ANGLE = 9, REG_MAX_ANGLE = 11, REG_TORQUE = 40, REG_ACC = 41,
    REG_GOAL_POS = 42, REG_POS = 56, REG_LOAD = 60, REG_VOLT = 62, REG_TEMP = 63,
};

/* Result: >= 0 number of parameter bytes, < 0 failure. */
enum { BUS_TIMEOUT = -1, BUS_CHECKSUM = -2 };

static int transact(uint8_t id, uint8_t inst, const uint8_t *prm, uint8_t n,
                    uint8_t *out, uint8_t outMax, uint8_t *errOut = nullptr) {
    uint8_t pkt[32];
    const uint8_t len = n + 2;
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = id; pkt[3] = len; pkt[4] = inst;
    uint8_t sum = id + len + inst;
    for (uint8_t i = 0; i < n; ++i) { pkt[5 + i] = prm[i]; sum += prm[i]; }
    pkt[5 + n] = (uint8_t)~sum;
    const uint8_t pktLen = 6 + n;

    while (Serial1.available()) Serial1.read();
    Serial1.write(pkt, pktLen);
    Serial1.flush();
    if (id == BROADCAST) return 0;                 /* broadcast gets no reply */

    /* Collect until a valid reply that is not an echo of what we sent. The
     * Waveshare board's half-duplex circuit should not echo, but tolerating
     * one costs nothing and removes a class of silent misreads. */
    uint8_t buf[64];
    uint8_t got = 0;
    bool badSum = false;
    const uint32_t t0 = micros();
    while (micros() - t0 < 4000) {
        while (Serial1.available() && got < sizeof(buf)) buf[got++] = Serial1.read();
        for (uint8_t i = 0; i + 5 < got; ++i) {
            if (buf[i] != 0xFF || buf[i + 1] != 0xFF) continue;
            const uint8_t plen = buf[i + 3];
            if (i + 4 + plen > got) break;             /* not all here yet   */
            if (plen == len && i + pktLen <= got && memcmp(buf + i, pkt, pktLen) == 0) {
                i += pktLen - 1;                        /* our own echo       */
                continue;
            }
            uint8_t s = buf[i + 2] + plen;
            for (uint8_t k = 0; k < plen - 1; ++k) s += buf[i + 4 + k];
            if ((uint8_t)~s != buf[i + 3 + plen]) { badSum = true; continue; }
            if (buf[i + 2] != id) continue;
            if (errOut) *errOut = buf[i + 4];
            const uint8_t np = plen - 2;
            for (uint8_t k = 0; k < np && k < outMax; ++k) out[k] = buf[i + 5 + k];
            return np;
        }
    }
    return badSum ? BUS_CHECKSUM : BUS_TIMEOUT;
}

static int readRegs(uint8_t id, uint8_t reg, uint8_t n, uint8_t *out) {
    const uint8_t prm[2] = { reg, n };
    const int r = transact(id, INST_READ, prm, 2, out, n);
    return (r == n) ? r : (r < 0 ? r : BUS_TIMEOUT);
}

static bool writeRegs(uint8_t id, uint8_t reg, const uint8_t *data, uint8_t n) {
    uint8_t prm[16];
    prm[0] = reg;
    for (uint8_t i = 0; i < n; ++i) prm[1 + i] = data[i];
    uint8_t dummy[4];
    return transact(id, INST_WRITE, prm, n + 1, dummy, sizeof(dummy)) >= 0;
}

static bool ping(uint8_t id) {
    uint8_t dummy[4];
    return transact(id, INST_PING, nullptr, 0, dummy, sizeof(dummy)) >= 0;
}

static inline uint16_t u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
/* STS encodes load as magnitude in bits 0..9, direction in bit 10. */
static inline int16_t loadSigned(uint16_t raw) {
    const int16_t mag = raw & 0x3FF;
    return (raw & 0x400) ? -mag : mag;
}

static void torqueOffAll() {
    const uint8_t off = 0;
    for (uint8_t k = 0; k < 3; ++k) writeRegs(BROADCAST, REG_TORQUE, &off, 1);
}

/* ----------------------------------------------------------------- state - */
static bool     gSuspect[SCAN_MAX_ID + 1];     /* inconsistent replies = dup  */
static bool     gSeen[SCAN_MAX_ID + 1];
static uint16_t gMinLim[SCAN_MAX_ID + 1], gMaxLim[SCAN_MAX_ID + 1];
static int      gActive = -1;
static bool     gFaulted = false, gTelem = false;
static int32_t  gHoldPos = 0, gGoal = 0;
static int32_t  gLastPos = 0;
static int16_t  gLastLoad = 0;
static uint32_t gLastHostMs = 0, gT0 = 0;

static void stopAll(const char *why, bool fault) {
    torqueOffAll();
    gActive = -1;
    if (fault) gFaulted = true;
    Serial.print(fault ? "!! FAULT, TORQUE OFF: " : "## STOP, torque off: ");
    Serial.println(why);
}

static bool writeGoal(uint8_t id, int32_t pos) {
    /* acc, goal pos L/H, goal time L/H (0 = use speed), goal speed L/H */
    const uint8_t d[7] = { TEST_ACC, (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
                           0, 0, (uint8_t)(TEST_SPEED & 0xFF), (uint8_t)(TEST_SPEED >> 8) };
    return writeRegs(id, REG_ACC, d, sizeof(d));
}

/* ------------------------------------------------------------------ scan - */
static void scan() {
    Serial.println("## scanning ids 1..20 (read-only)");
    uint8_t found = 0;
    for (uint8_t id = 1; id <= SCAN_MAX_ID; ++id) {
        gSeen[id] = false; gSuspect[id] = false;
        if (!ping(id) && !ping(id)) continue;
        gSeen[id] = true; ++found;

        /* Read the position several times. Two servos answering to one ID
         * collide on the bus: checksum failures, timeouts, or positions that
         * jump between two values. Any of those marks the ID as suspect. */
        int32_t lo = 100000, hi = -100000;
        uint8_t fails = 0;
        for (uint8_t k = 0; k < 8; ++k) {
            uint8_t b[2];
            if (readRegs(id, REG_POS, 2, b) != 2) { ++fails; continue; }
            const int32_t p = u16(b);
            if (p < lo) lo = p;
            if (p > hi) hi = p;
        }
        if (fails > 0 || hi - lo > 20) gSuspect[id] = true;

        uint8_t st[8] = {0}, lim[4] = {0}, tq = 0;
        readRegs(id, REG_POS, 8, st);           /* pos(2) speed(2) load(2) volt temp */
        readRegs(id, REG_MIN_ANGLE, 4, lim);
        readRegs(id, REG_TORQUE, 1, &tq);
        gMinLim[id] = u16(lim); gMaxLim[id] = u16(lim + 2);

        Serial.print("S,"); Serial.print(id);
        Serial.print(','); Serial.print(u16(st));                       /* pos   */
        Serial.print(','); Serial.print(loadSigned(u16(st + 4)));       /* load  */
        Serial.print(','); Serial.print(st[6] / 10.0f, 1);              /* volts */
        Serial.print(','); Serial.print(st[7]);                         /* degC  */
        Serial.print(','); Serial.print(tq);                            /* torque*/
        Serial.print(','); Serial.print(gMinLim[id]);
        Serial.print(','); Serial.print(gMaxLim[id]);
        Serial.print(','); Serial.print(fails);
        Serial.print(','); Serial.println(gSuspect[id] ? "SUSPECT" : "ok");
    }
    Serial.print("## scan done, "); Serial.print(found); Serial.println(" responding");
}

/* -------------------------------------------------------------- checksum - */
/* Did the last line examined by checksumOk() carry a "*XX"? */
static bool gLineSigned = false;

static bool checksumOk(char *line) {
    char *star = strchr(line, '*');
    gLineSigned = (star != nullptr);
    if (!star) return true;
    uint8_t want = 0;
    for (const char *q = star + 1; *q; ++q) {
        const char c = *q;
        const uint8_t nib = (c >= '0' && c <= '9') ? (c - '0')
                          : (c >= 'A' && c <= 'F') ? (c - 'A' + 10)
                          : (c >= 'a' && c <= 'f') ? (c - 'a' + 10) : 0xFF;
        if (nib == 0xFF) return false;
        want = (uint8_t)((want << 4) | nib);
    }
    uint8_t have = 0;
    for (char *q = line; q < star; ++q) have ^= (uint8_t)*q;
    *star = '\0';                      /* hide the checksum from the parser */
    return have == want;
}

/* ------------------------------------------------------------------ hold - */
static void hold(int id) {
    if (id < 1 || id > SCAN_MAX_ID) { Serial.println("!! id out of range"); return; }
    if (gFaulted)       { Serial.println("!! faulted - find out why, then 'c'"); return; }
    if (gActive >= 0)   { Serial.println("!! another servo holds - 'x' first"); return; }
    if (!gSeen[id])     { Serial.println("!! id not found - run 'scan' first"); return; }
    if (gSuspect[id])   { Serial.println("!! id answered inconsistently - likely shared by two servos, refusing"); return; }
    if (gMinLim[id] == 0 && gMaxLim[id] == 0) {
        Serial.println("!! servo is in multi-turn / wheel mode (limits 0,0) - refusing");
        return;
    }
    uint8_t b[2];
    if (readRegs(id, REG_POS, 2, b) != 2) { Serial.println("!! could not read position"); return; }
    const int32_t pos = u16(b);

    /* Goal first, torque second: whichever the servo acts on, it acts on its
     * own present position, so enabling does not move it. */
    if (!writeGoal(id, pos)) { Serial.println("!! goal write failed"); return; }
    const uint8_t on = 1;
    if (!writeRegs(id, REG_TORQUE, &on, 1)) { stopAll("torque-on write failed", true); return; }

    gActive = id; gHoldPos = gGoal = gLastPos = pos;
    gLastHostMs = millis();
    Serial.print("## HOLDING "); Serial.print(id);
    Serial.print(" at "); Serial.println(pos);
}

static void moveBy(float deg) {
    if (gActive < 0) { Serial.println("!! nothing holding"); return; }
    const int32_t fence = (int32_t)(FENCE_DEG * STEPS_PER_DEG);
    int32_t g = gGoal + (int32_t)lroundf(deg * STEPS_PER_DEG);
    g = constrain(g, gHoldPos - fence, gHoldPos + fence);
    g = constrain(g, (int32_t)gMinLim[gActive], (int32_t)gMaxLim[gActive]);
    if (!writeGoal(gActive, g)) { stopAll("goal write failed", true); return; }
    gGoal = g;
    Serial.print("## goal "); Serial.println(gGoal);
}

/* ----------------------------------------------------------------- centre - */
/* Re-index a servo's zero onto its present position, by writing 128 to the
 * torque register (Feetech's CalibrationOfs): the servo computes an internal
 * offset so that where it stands now reads 2048, and stores it in EEPROM. So
 * the joint must ALREADY be sitting where you want its centre - drive it there,
 * look at it, then run this.
 *
 * Re-centring restores a joint's full range and makes the count-to-radian
 * mapping the clean zero_counts = 2048. Refuses while a servo holds: the
 * reading has to be a settled position, not one the loop is still chasing. */
static void centre(int id) {
    if (id < 1 || id > SCAN_MAX_ID) { Serial.println("!! id out of range"); return; }
    if (gActive >= 0) { Serial.println("!! a servo is holding - 'x' first, let it settle"); return; }
    uint8_t b[2];
    if (readRegs(id, REG_POS, 2, b) != 2) { Serial.println("!! could not read position"); return; }
    const int32_t before = u16(b);

    /* Relax first: centring a powered servo would store an offset measured
     * against a position the loop is actively holding against. */
    const uint8_t off = 0;
    if (!writeRegs(id, REG_TORQUE, &off, 1)) { Serial.println("!! torque-off write failed"); return; }
    delay(20);

    /* The EEPROM write takes longer than transact()'s 4 ms reply window, so
     * the ack arrives too late and writeRegs() reports a failure for a write
     * that did happen. Judge by the read-back below, not the ack. */
    const uint8_t calibrate = 128;
    (void)writeRegs(id, REG_TORQUE, &calibrate, 1);
    delay(150);

    if (readRegs(id, REG_POS, 2, b) != 2) { Serial.println("!! could not read back"); return; }
    const int32_t after = u16(b);
    Serial.print("## centred id "); Serial.print(id);
    Serial.print(": was "); Serial.print(before);
    Serial.print(" counts, now reads "); Serial.println(after);
    if (after < 2043 || after > 2053)
        Serial.println("!! expected ~2048 - the offset did NOT take, do not trust it");
    else
        Serial.println("## zero stored in the servo's EEPROM - it survives power off");
}

/* ------------------------------------------------------------------- I/O - */
static void handleSerial() {
    static char line[32];
    static uint8_t idx = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        gLastHostMs = millis();
        if (c == '\r') continue;
        if (c != '\n') { if (idx < sizeof(line) - 1) line[idx++] = c; continue; }
        line[idx] = '\0';
        idx = 0;
        if (!checksumOk(line)) { Serial.println("!! bad checksum - ignored"); continue; }
        char *p = line;
        while (*p == ' ') ++p;

        /* Unsigned lines may only do what cannot move a servo or rewrite a
         * stored calibration. Stopping is always allowed. */
        if (!gLineSigned && !(*p == 'x' || *p == 's' || *p == 'h' ||
                              *p == 't' || *p == '\0' || !strncmp(p, "scan", 4))) {
            Serial.println("!! refusing unsigned command - append *XX");
            continue;
        }

        if (!strncmp(p, "scan", 4))        { if (gActive >= 0) Serial.println("!! 'x' first"); else scan(); }
        else if (!strncmp(p, "centre", 6)) centre(atoi(p + 6));
        else if (!strncmp(p, "hold", 4)) hold(atoi(p + 4));
        else if (*p == 'm')              moveBy(atof(p + 1));
        else if (*p == 'x' || *p == 's') stopAll("operator stop", false);
        else if (*p == 'c')              { gFaulted = false; Serial.println("## fault cleared"); }
        else if (*p == 't') {
            /* t1 / t0 set it explicitly; bare t toggles. The host uses t1:
             * the ESP32 does not reset between sessions, so a toggle would
             * leave the second session's telemetry off. */
            gTelem = (p[1] == '1') ? true : (p[1] == '0') ? false : !gTelem;
            gT0 = millis();
            Serial.println(gTelem ? "## telemetry on" : "## telemetry off");
        }
        else if (*p == 'h' || *p == '\0') { }
        else Serial.println("!! unknown command");
    }
}

/* ------------------------------------------------------------ setup/loop - */
void setup() {
    Serial.begin(115200);
    Serial1.begin(BUS_BAUD, SERIAL_8N1, BUS_RX, BUS_TX);
    delay(50);
    torqueOffAll();
    Serial.println("\n=== st3215_test: SAFE - broadcast torque off ===");
    Serial.println("## ready");
}

void loop() {
    static uint32_t nextPollMs = 0, nextTelemMs = 0;
    handleSerial();

    if (gActive < 0 || millis() < nextPollMs) return;
    nextPollMs = millis() + 20;

    if (millis() - gLastHostMs > HOST_TIMEOUT_MS) { stopAll("host silent (dead-man)", true); return; }

    /* One missed reply is bus noise. Three CONSECUTIVE misses is not. */
    static uint8_t misses = 0;
    uint8_t b[4];
    if (readRegs(gActive, REG_POS, 2, b) != 2 || readRegs(gActive, REG_LOAD, 2, b + 2) != 2) {
        if (++misses >= 3) { misses = 0; stopAll("servo stopped answering", true); }
        return;
    }
    misses = 0;
    gLastPos  = u16(b);
    gLastLoad = loadSigned(u16(b + 2));
    if (abs(gLastLoad) > LOAD_LIMIT) { stopAll("load limit - collision or stall", true); return; }

    if (gTelem && millis() >= nextTelemMs) {
        nextTelemMs = millis() + 50;
        Serial.print("T,"); Serial.print(millis() - gT0);
        Serial.print(','); Serial.print(gActive);
        Serial.print(','); Serial.print(gGoal);
        Serial.print(','); Serial.print(gLastPos);
        Serial.print(','); Serial.println(gLastLoad);
    }
}
