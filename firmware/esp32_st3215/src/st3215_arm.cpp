/* st3215_arm - the three wrist joints, driven from ROS.
 *
 * Same protocol shape as the Mega's arm3 firmware, so the ros2_control
 * interface speaks one language to both boards and differs only in units
 * (counts here, degrees there).
 *
 * An ST3215 closes its own position loop around its own encoder, and this
 * firmware does not add a loop around that - it would fight it. What it adds is
 * what the servo cannot do for itself: a safe boot state, a dead-man, a
 * collision trip from the servo's own load reading, angle clamping, and one
 * command that moves all three together.
 *
 * Safe at boot: broadcast torque-off, so a reset - which opening the USB port
 * can cause - leaves the wrist limp. It never writes servo EEPROM unless asked
 * ('C', 'P', 'W'), so it cannot change an ID or baud rate by accident.
 *
 * Faults relax all three and latch until 'c':
 *   load      |load| over LOAD_LIMIT: a collision or stall. The ST3215
 *             measures its own load, so this is real contact sensing.
 *   bus       a servo stops answering (BUS_MISS_TRIP consecutive misses).
 *   dead-man  no byte from the host for HOST_TIMEOUT_MS while enabled.
 *   volts     any servo's supply below UNDERVOLT_TENTHS.
 * The physical e-stop remains the only thing that removes power.
 *
 * Wiring: ST3215 ids 1, 2, 3 are URDF joint4 (wrist roll), joint5 (pitch) and
 * joint6 (tool roll). The sign mapping lives in the ROS layer
 * (arm_description/config/actuators.yaml), not here. This firmware speaks raw
 * counts: 0..4095 over one turn, 2048 at mid-travel.
 *
 * Host protocol, 115200 baud, one command per line, XOR checksum "*XX"
 * required for anything that can move the arm:
 *   E              enable: goal := present position, then torque on, so
 *                  enabling never moves the wrist. Refuses unless all three
 *                  answer and none is in wheel mode.
 *   D | x | s      relax all (broadcast torque off)
 *   G <a> <b> <c>  stream absolute targets in counts for ids 1,2,3. Not acked;
 *                  the status frame echoes the targets every cycle.
 *   B <hz>         binary status frame at <hz> (0 = off, max 200)
 *   ?              human-readable status
 *   c              clear a latched fault
 *   h              heartbeat
 *   v <steps/s>    speed ceiling handed to the servos, 10..2000
 *   C <id>         re-index that servo's zero onto where it stands now
 *   Q              dump each servo's control constants
 *   P <id> <kp> <kd> <ki>   tune the servo's own position loop (EEPROM)
 *   W <id> <cw> <ccw>       widen its insensitive area, in counts (EEPROM)
 *
 * Binary status frame, 32 bytes, little-endian:
 *    0  2  A5 5A sync
 *    2  4  uint32 ms
 *    6  1  uint8  state, 1 = enabled
 *    7  1  uint8  fault: 0 none, 1 load, 2 bus, 3 dead-man, 4 refused, 5 volts
 *    8  1  uint8  fault servo id, or 0
 *    9  6  uint16 present position x3 (counts)
 *   15  6  int16  present load x3 (-1000..1000)
 *   21  6  uint16 target x3 (counts)
 *   27  3  uint8  supply per servo, 0.1 V units
 *   30  1  uint8  lowest supply seen since boot, 0.1 V units
 *   31  1  uint8  XOR of bytes 2..30
 */

#include <Arduino.h>

/* ---------------------------------------------------------------- limits - */
static const uint16_t HOST_TIMEOUT_MS = 400;
static const uint16_t LOAD_LIMIT      = 500;    /* 0..1000 = 0..100% of stall */
/* A ceiling for catching up, not the operating speed - writeGoal() computes
 * what each segment needs. 800 steps/s ~= 70 deg/s, above the 60 deg/s the URDF
 * allows, so it never binds on a valid trajectory while still bounding a servo
 * asked to close a large error. Judge any change at a pose that LOADS the
 * wrist, not at home: a high ceiling makes corrective moves fast, and an ST3215
 * holding the gripper off-axis corrects constantly. */
static const uint16_t DEFAULT_SPEED   = 800;
static const uint8_t  MOVE_ACC        = 20;     /* x100 steps/s^2             */
static const uint8_t  NJ              = 3;
static const uint8_t  IDS[NJ]         = { 1, 2, 3 };   /* joint4, joint5, joint6 */

/* ------------------------------------------------------------------- bus - */
static const int      BUS_RX = 18, BUS_TX = 19;
static const uint32_t BUS_BAUD = 1000000;
static const uint8_t  BROADCAST = 0xFE;

enum : uint8_t { INST_PING = 0x01, INST_READ = 0x02, INST_WRITE = 0x03 };
enum : uint8_t {
    /* Tuning registers, EEPROM region. Feetech's defaults are often Kd = 0,
     * leaving nothing to damp a servo holding an inertial load. */
    REG_KP = 21, REG_KD = 22, REG_KI = 23, REG_DEAD_CW = 26, REG_DEAD_CCW = 27,
    REG_LOCK = 55,
    REG_MIN_ANGLE = 9, REG_MAX_ANGLE = 11, REG_TORQUE = 40, REG_ACC = 41,
    REG_POS = 56, REG_LOAD = 60,
};

enum Fault : uint8_t { F_NONE = 0, F_LOAD = 1, F_BUS = 2, F_DEADMAN = 3, F_REFUSED = 4,
                       F_VOLTS = 5 };

static int transact(uint8_t id, uint8_t inst, const uint8_t *prm, uint8_t n,
                    uint8_t *out, uint8_t outMax) {
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
    if (id == BROADCAST) return 0;

    uint8_t buf[64];
    uint8_t got = 0;
    const uint32_t t0 = micros();
    while (micros() - t0 < 4000) {
        while (Serial1.available() && got < sizeof(buf)) buf[got++] = Serial1.read();
        for (uint8_t i = 0; i + 5 < got; ++i) {
            if (buf[i] != 0xFF || buf[i + 1] != 0xFF) continue;
            const uint8_t plen = buf[i + 3];
            if (i + 4 + plen > got) break;
            if (plen == len && i + pktLen <= got && memcmp(buf + i, pkt, pktLen) == 0) {
                i += pktLen - 1;                       /* our own echo */
                continue;
            }
            uint8_t s = buf[i + 2] + plen;
            for (uint8_t k = 0; k < plen - 1; ++k) s += buf[i + 4 + k];
            if ((uint8_t)~s != buf[i + 3 + plen]) continue;
            if (buf[i + 2] != id) continue;
            const uint8_t np = plen - 2;
            for (uint8_t k = 0; k < np && k < outMax; ++k) out[k] = buf[i + 5 + k];
            return np;
        }
    }
    return -1;
}

static bool readRegs(uint8_t id, uint8_t reg, uint8_t n, uint8_t *out) {
    const uint8_t prm[2] = { reg, n };
    return transact(id, INST_READ, prm, 2, out, n) == n;
}

static bool writeRegs(uint8_t id, uint8_t reg, const uint8_t *data, uint8_t n) {
    uint8_t prm[16];
    prm[0] = reg;
    for (uint8_t i = 0; i < n; ++i) prm[1 + i] = data[i];
    uint8_t dummy[4];
    return transact(id, INST_WRITE, prm, n + 1, dummy, sizeof(dummy)) >= 0;
}

static inline uint16_t u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
/* STS load: magnitude in bits 0..9, direction in bit 10. */
static inline int16_t loadSigned(uint16_t raw) {
    const int16_t mag = raw & 0x3FF;
    return (raw & 0x400) ? -mag : mag;
}

/* ----------------------------------------------------------------- state - */
static bool     gEnabled = false;
static uint8_t  gFault = F_NONE, gFaultId = 0;
static uint16_t gSpeed = DEFAULT_SPEED;
static uint16_t gPos[NJ], gTarget[NJ], gMinLim[NJ], gMaxLim[NJ];
static int16_t  gLoad[NJ];
static uint8_t  gMiss[NJ];
static uint32_t gLastHostMs = 0, gT0 = 0;
static float    gBinHz = 0;
static uint16_t gRxOk = 0, gRxBad = 0;

static void torqueOffAll() {
    const uint8_t off = 0;
    for (uint8_t k = 0; k < 3; ++k) writeRegs(BROADCAST, REG_TORQUE, &off, 1);
}

static void relaxAll(uint8_t fault, uint8_t id, const char *why) {
    torqueOffAll();
    gEnabled = false;
    for (uint8_t i = 0; i < NJ; ++i) gMiss[i] = 0;
    if (fault != F_NONE) { gFault = fault; gFaultId = id; }
    Serial.print(fault != F_NONE ? "!! FAULT " : "## relaxed: ");
    if (fault != F_NONE && id) { Serial.print("id "); Serial.print(id); Serial.print(' '); }
    Serial.println(why);
}

/* Bus health, for telling a noisy link from a browning-out battery. gVoltMin
 * is the lowest supply any servo has reported since boot. */
static uint8_t  gVolt[NJ];
static uint8_t  gVoltMin = 255;
static uint16_t gMissTotal[NJ];

/* What was last actually written to each servo, so an unchanged goal is not
 * written again. Every write restarts the ST3215's internal move planner, and
 * doing that at 100 Hz made the wrist hunt 3-7 counts peak-to-peak while
 * holding. The servo holds its goal without being told again. */
static uint16_t gWrote[NJ];
static bool     gWroteValid[NJ];

static uint32_t gGoalMs[NJ];

static bool writeGoal(uint8_t i) {
    const uint16_t p = gTarget[i];

    /* Speed matched to the motion asked for, not the ceiling.
     *
     * An ST3215 given a goal and a speed travels at that speed until it
     * arrives, then stops. Handed the ceiling every cycle it darts across each
     * 10 ms segment and waits out the rest of the tick, turning a 9 deg/s
     * trajectory into 100 Hz of micro-jumps (commanded 8.9 deg/s, measured
     * 53.3 deg/s peaks). Asking for just enough speed to cover the remaining
     * distance in the time available makes each segment take its whole
     * interval, which is continuous motion.
     *
     * Measuring from the servo's PRESENT position rather than the last goal
     * makes it self-correcting: a servo that has fallen behind gets
     * proportionally more speed, up to the ceiling. The 10% margin stops a
     * chronic deficit accumulating into lag; the floor keeps a stationary goal
     * from being written with speed 0, which the servo reads as "fastest". */
    uint16_t spd = gSpeed;
    const uint32_t now = millis();
    const uint32_t dt = now - gGoalMs[i];
    if (gWroteValid[i] && dt > 0 && dt < 200) {
        const int32_t togo = (int32_t)p - (int32_t)gPos[i];
        const uint32_t need = ((uint32_t)labs(togo) * 1100u) / dt;
        spd = (uint16_t)constrain((long)need, 20L, (long)gSpeed);
    }
    gGoalMs[i] = now;

    const uint8_t d[7] = { MOVE_ACC, (uint8_t)(p & 0xFF), (uint8_t)(p >> 8),
                           0, 0, (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8) };
    return writeRegs(IDS[i], REG_ACC, d, sizeof(d));
}

/* ------------------------------------------------------------- checksums - */
/* Did the last line examined by checksumOk() actually carry a "*XX"? */
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
    *star = '\0';
    return have == want;
}

/* --------------------------------------------------------------- tuning - */
/* One register at a time: a block read of 21..27 is refused by these servos,
 * so nothing here assumes a span can be read or written at once.
 *
 * Kp/Kd/Ki (21/22/23) and the insensitive areas (26/27) live in EEPROM behind
 * the LOCK register, which is opened and closed around each write. An EEPROM
 * write's ack routinely arrives after transact()'s 4 ms window, so success is
 * judged by reading the value back. */
static bool readReg1(uint8_t id, uint8_t reg, uint8_t *out) {
    return readRegs(id, reg, 1, out);        /* bool, not a count - see 'C' above */
}

static bool setReg1(uint8_t id, uint8_t reg, uint8_t val) {
    const uint8_t unlock = 0, lock = 1;
    writeRegs(id, REG_LOCK, &unlock, 1);
    delay(10);
    writeRegs(id, reg, &val, 1);
    delay(30);                          /* the servo writes its EEPROM here */
    writeRegs(id, REG_LOCK, &lock, 1);
    delay(10);
    uint8_t back = 0;
    return readReg1(id, reg, &back) && back == val;
}

/* --------------------------------------------------------------- reading - */
/* How many consecutive missed replies mean a servo is really gone. Polling
 * runs at 100 Hz, so 20 = 200 ms, half the host dead-man. Losing telemetry is
 * not losing control (an ST3215 holds its own goal), so tripping at 3 relaxed
 * the wrist mid-move for 30 ms of silence; a servo that has really lost power
 * misses continuously and still trips well inside a second. */
static const uint8_t BUS_MISS_TRIP = 20;

/* Relax below this supply, in 0.1 V units. 90 = 9.0 V = 3.0 V/cell on 3S, the
 * normal LiPo cutoff under load. An ST3215 is rated 6-12.6 V so it is nowhere
 * near browning out here; the threshold exists to protect the pack and to catch
 * a real collapse. It was 95, which tripped on this arm's own 2.8 V sag under
 * load and stopped the arm working at all. Raise it again once the power path
 * is fixed. */
static const uint8_t UNDERVOLT_TENTHS = 90;

static bool pollServos() {
    bool ok = true;
    for (uint8_t i = 0; i < NJ; ++i) {
        uint8_t b[8];
        if (readRegs(IDS[i], REG_POS, 8, b)) {      /* pos, speed, load, volt, temp */
            gPos[i] = u16(b);
            gLoad[i] = loadSigned(u16(b + 4));
            gVolt[i] = b[6];                        /* 0.1 V units */
            if (b[6] && b[6] < gVoltMin) gVoltMin = b[6];
            gMiss[i] = 0;
        } else if (++gMissTotal[i], ++gMiss[i] >= BUS_MISS_TRIP) {
            relaxAll(F_BUS, IDS[i], "servo stopped answering");
            ok = false;
            break;
        }
    }
    return ok;
}

/* ---------------------------------------------------------------- enable - */
static void enableAll() {
    if (gFault != F_NONE) { Serial.println("!! faulted - find out why, then 'c'"); return; }
    for (uint8_t i = 0; i < NJ; ++i) {
        uint8_t b[4];
        if (!readRegs(IDS[i], REG_POS, 2, b)) {
            Serial.print("!! refusing: id "); Serial.print(IDS[i]); Serial.println(" not answering");
            gFault = F_REFUSED; gFaultId = IDS[i];
            return;
        }
        gPos[i] = gTarget[i] = u16(b);
        if (!readRegs(IDS[i], REG_MIN_ANGLE, 4, b)) { gMinLim[i] = 0; gMaxLim[i] = 4095; }
        else { gMinLim[i] = u16(b); gMaxLim[i] = u16(b + 2); }
        if (gMinLim[i] == 0 && gMaxLim[i] == 0) {
            Serial.print("!! refusing: id "); Serial.print(IDS[i]);
            Serial.println(" is in wheel/multi-turn mode (limits 0,0)");
            gFault = F_REFUSED; gFaultId = IDS[i];
            return;
        }
    }
    /* Goal before torque, so whichever the servo acts on first it acts on its
     * own present position and the wrist does not move. */
    for (uint8_t i = 0; i < NJ; ++i) {
        gWroteValid[i] = false;
        if (writeGoal(i)) { gWrote[i] = gTarget[i]; gWroteValid[i] = true; }
    }
    const uint8_t on = 1;
    for (uint8_t i = 0; i < NJ; ++i) {
        if (!writeRegs(IDS[i], REG_TORQUE, &on, 1)) {
            relaxAll(F_BUS, IDS[i], "torque-on write failed");
            return;
        }
    }
    gEnabled = true;
    gLastHostMs = millis();
    Serial.print("## ENABLED at");
    for (uint8_t i = 0; i < NJ; ++i) { Serial.print(' '); Serial.print(gPos[i]); }
    Serial.println();
}

/* --------------------------------------------------------------- reports - */
static void binaryFrame() {
    uint8_t b[32];
    uint8_t k = 0;
    b[k++] = 0xA5; b[k++] = 0x5A;
    const uint32_t ms = millis() - gT0;
    b[k++] = (uint8_t)ms; b[k++] = (uint8_t)(ms >> 8);
    b[k++] = (uint8_t)(ms >> 16); b[k++] = (uint8_t)(ms >> 24);
    b[k++] = gEnabled ? 1 : 0;
    b[k++] = gFault;
    b[k++] = gFaultId;
    for (uint8_t i = 0; i < NJ; ++i) { b[k++] = (uint8_t)gPos[i]; b[k++] = (uint8_t)(gPos[i] >> 8); }
    for (uint8_t i = 0; i < NJ; ++i) { b[k++] = (uint8_t)gLoad[i]; b[k++] = (uint8_t)(gLoad[i] >> 8); }
    for (uint8_t i = 0; i < NJ; ++i) { b[k++] = (uint8_t)gTarget[i]; b[k++] = (uint8_t)(gTarget[i] >> 8); }
    /* Every servo's own supply reading, 0.1 V per count, plus the lowest seen
     * since boot. The minimum alone cannot say whether the supply sags at the
     * SOURCE or drops ALONG the daisy chain: all three falling together means
     * upstream (pack, leads, a shared connector), while the last in the chain
     * falling furthest means the loss is between the servos. */
    for (uint8_t i = 0; i < NJ; ++i) b[k++] = gVolt[i];
    b[k++] = (gVoltMin == 255) ? 0 : gVoltMin;
    uint8_t sum = 0;
    for (uint8_t i = 2; i < k; ++i) sum ^= b[i];
    b[k++] = sum;
    Serial.write(b, k);
}

static void report() {
    for (uint8_t i = 0; i < NJ; ++i) {
        Serial.print("S,"); Serial.print(IDS[i]);
        Serial.print(','); Serial.print(gPos[i]);
        Serial.print(','); Serial.print(gTarget[i]);
        Serial.print(','); Serial.print(gLoad[i]);
        Serial.print(','); Serial.print(gMinLim[i]);
        Serial.print(','); Serial.print(gMaxLim[i]);
        Serial.print(','); Serial.println(gVolt[i] / 10.0f, 1);
    }
    Serial.print("## enabled="); Serial.print(gEnabled ? 1 : 0);
    Serial.print(" fault="); Serial.print(gFault);
    Serial.print(" id="); Serial.print(gFaultId);
    Serial.print(" speed="); Serial.print(gSpeed);
    Serial.print(" rx_ok="); Serial.print(gRxOk);
    Serial.print(" rx_bad="); Serial.print(gRxBad);
    Serial.print(" bin="); Serial.print(gBinHz, 0);
    Serial.print("Hz vmin="); Serial.print(gVoltMin / 10.0f, 1);
    Serial.print("V miss=");
    for (uint8_t i = 0; i < NJ; ++i) { if (i) Serial.print('/'); Serial.print(gMissTotal[i]); }
    Serial.println();
}

/* ------------------------------------------------------------------- I/O - */
static void handleSerial() {
    static char line[64];
    static uint8_t idx = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        gLastHostMs = millis();
        if (c == '\r') continue;
        if (c != '\n') { if (idx < sizeof(line) - 1) line[idx++] = c; continue; }
        line[idx] = '\0';
        idx = 0;
        if (!checksumOk(line)) { ++gRxBad; continue; }
        ++gRxOk;
        char *p = line;
        while (*p == ' ') ++p;

        /* Unsigned lines are accepted so a human can type into a serial
         * monitor, but corruption can drop bytes - losing the tail of
         * "G 100 200 300*4B" leaves a valid "G 100 200" - so they may only ask
         * for things that cannot move the arm. Stopping is always allowed:
         * refusing a stop is never safe. */
        if (!gLineSigned && !(*p == 'D' || *p == 'x' || *p == 's' ||
                              *p == 'h' || *p == '?' || *p == '\0')) {
            ++gRxBad;
            Serial.println("!! refusing unsigned command - append *XX");
            continue;
        }

        if (*p == 'E')      enableAll();
        else if (*p == 'D' || *p == 'x' || *p == 's') relaxAll(F_NONE, 0, "operator stop");
        else if (*p == 'G') {
            long v[NJ];
            char *q = p + 1;
            uint8_t n = 0;
            while (n < NJ) {
                char *end = nullptr;
                const long val = strtol(q, &end, 10);
                if (end == q) break;
                v[n++] = val;
                q = end;
            }
            if (n == NJ) {
                for (uint8_t i = 0; i < NJ; ++i)
                    gTarget[i] = (uint16_t)constrain(v[i], (long)gMinLim[i], (long)gMaxLim[i]);
            }
        }
        else if (*p == 'B') { gBinHz = constrain(atof(p + 1), 0.0f, 200.0f); gT0 = millis();
                              Serial.print("## binary "); Serial.print(gBinHz, 0); Serial.println(" Hz"); }
        else if (*p == 'v') { gSpeed = (uint16_t)constrain(atol(p + 1), 10L, 2000L);
                              Serial.print("## speed "); Serial.println(gSpeed); }
        else if (*p == 'C') {
            /* C <id> - re-index this servo's zero onto where it stands now, by
             * writing 128 to the torque register (Feetech's CalibrationOfs):
             * the servo computes an internal offset so its present position
             * reads 2048 and stores it in EEPROM.
             *
             * This beats recording a far-from-centre zero in actuators.yaml,
             * which costs travel and parks the joint near the 0/4095 seam - a
             * goal cannot cross the seam and the reading flips 360 deg beside
             * it. Refuses while enabled, and judged by reading the position
             * back rather than by the ack. */
            if (gEnabled) { Serial.println("!! relax first - 'D' - then centre"); }
            else {
                const uint8_t id = (uint8_t)atoi(p + 1);
                bool known = false;
                for (uint8_t i = 0; i < NJ; ++i) if (IDS[i] == id) known = true;
                if (!known) { Serial.println("!! not one of this arm's servo ids"); }
                else {
                    uint8_t b[2];
                    int32_t before = -1;
                    /* readRegs returns bool, not a byte count. */
                    if (readRegs(id, REG_POS, 2, b)) before = u16(b);
                    const uint8_t off = 0, calibrate = 128;
                    writeRegs(id, REG_TORQUE, &off, 1);
                    delay(20);
                    (void)writeRegs(id, REG_TORQUE, &calibrate, 1);
                    delay(150);
                    int32_t after = -1;
                    if (readRegs(id, REG_POS, 2, b)) after = u16(b);
                    Serial.print("## centred id "); Serial.print(id);
                    Serial.print(": was "); Serial.print(before);
                    Serial.print(" now "); Serial.println(after);
                    if (after < 2043 || after > 2053)
                        Serial.println("!! expected ~2048 - the offset did NOT take");
                    else
                        Serial.println("## stored in the servo's EEPROM");
                }
            }
        }
        else if (*p == 'Q') {
            /* Read-only dump of each servo's own control constants. */
            static const uint8_t regs[5] = { REG_KP, REG_KD, REG_KI,
                                             REG_DEAD_CW, REG_DEAD_CCW };
            static const char *names[5] = { "kp", "kd", "ki", "dead_cw", "dead_ccw" };
            for (uint8_t i = 0; i < NJ; ++i) {
                Serial.print("P,"); Serial.print(IDS[i]);
                for (uint8_t r = 0; r < 5; ++r) {
                    uint8_t v = 0;
                    Serial.print(','); Serial.print(names[r]); Serial.print('=');
                    if (readReg1(IDS[i], regs[r], &v)) Serial.print(v);
                    else Serial.print('?');
                }
                Serial.println();
            }
        }
        else if (*p == 'P' || *p == 'W') {
            /* P <id> <kp> <kd> <ki>   tune the servo's own position loop
             * W <id> <cw> <ccw>       widen its insensitive area, in counts
             *
             * Both write EEPROM, and both are deliberately allowed while the arm
             * is held: an ST3215 only hunts under load, so the effect has to be
             * judged in the pose that loads it. Nothing polls during the
             * blocking write, so the bus-miss counter cannot trip on it. */
            const bool pid = (*p == 'P');
            long v[4];
            char *q = p + 1;
            uint8_t n = 0;
            while (n < 4) {
                char *end = nullptr;
                const long val = strtol(q, &end, 10);
                if (end == q) break;
                v[n++] = val;
                q = end;
            }
            const uint8_t want = pid ? 4 : 3;
            if (n != want) {
                Serial.println(pid ? "!! usage: P <id> <kp> <kd> <ki>"
                                   : "!! usage: W <id> <cw> <ccw>");
            } else {
                const uint8_t id = (uint8_t)v[0];
                bool ok = true;
                if (pid) {
                    ok &= setReg1(id, REG_KP, (uint8_t)constrain(v[1], 0L, 254L));
                    ok &= setReg1(id, REG_KD, (uint8_t)constrain(v[2], 0L, 254L));
                    ok &= setReg1(id, REG_KI, (uint8_t)constrain(v[3], 0L, 254L));
                } else {
                    ok &= setReg1(id, REG_DEAD_CW, (uint8_t)constrain(v[1], 0L, 32L));
                    ok &= setReg1(id, REG_DEAD_CCW, (uint8_t)constrain(v[2], 0L, 32L));
                }
                for (uint8_t i = 0; i < NJ; ++i) gMiss[i] = 0;
                Serial.print(ok ? "## tuned id " : "!! write NOT confirmed on id ");
                Serial.println(id);
            }
        }
        else if (*p == '?') report();
        else if (*p == 'c') { gFault = F_NONE; gFaultId = 0; Serial.println("## fault cleared"); }
        else if (*p == 'h' || *p == '\0') { }
        else { ++gRxBad; }
    }
}

/* ------------------------------------------------------------ setup/loop - */
void setup() {
    Serial.begin(115200);
    Serial1.begin(BUS_BAUD, SERIAL_8N1, BUS_RX, BUS_TX);
    delay(50);
    torqueOffAll();
    for (uint8_t i = 0; i < NJ; ++i) { gMinLim[i] = 0; gMaxLim[i] = 4095; }
    pollServos();
    for (uint8_t i = 0; i < NJ; ++i) gTarget[i] = gPos[i];
    gT0 = millis();
    Serial.println("\n=== st3215_arm: SAFE - broadcast torque off ===");
    report();
    Serial.println("## ready");
}

void loop() {
    static uint32_t nextPollMs = 0, nextBinMs = 0, nextGoalMs = 0;
    handleSerial();

    /* Status deadlines advance on a fixed grid. Setting the next one from
     * "now" makes every jitter permanent - that turned a 100 Hz request into
     * 78.7 Hz on the Mega. */
    if (gBinHz > 0 && (int32_t)(millis() - nextBinMs) >= 0) {
        const uint32_t period = (uint32_t)(1000.0f / gBinHz);
        nextBinMs += period;
        if ((int32_t)(millis() - nextBinMs) > 0) nextBinMs = millis() + period;
        binaryFrame();
    }

    if ((int32_t)(millis() - nextPollMs) < 0) return;
    nextPollMs = millis() + 10;                       /* 100 Hz */

    if (!pollServos()) return;

    if (!gEnabled) {
        /* Keep the targets glued to where the servos are, so enabling never
         * produces a jump. */
        for (uint8_t i = 0; i < NJ; ++i) gTarget[i] = gPos[i];
        return;
    }

    if (millis() - gLastHostMs > HOST_TIMEOUT_MS) { relaxAll(F_DEADMAN, 0, "host silent"); return; }

    for (uint8_t i = 0; i < NJ; ++i) {
        if (abs(gLoad[i]) > LOAD_LIMIT) {
            relaxAll(F_LOAD, IDS[i], "load limit - collision or stall");
            return;
        }
    }

    /* Undervoltage. Holding torque against a collapsing rail is the worst
     * thing to do with it: the servos draw more as the voltage falls, pulling
     * it down further, and the first thing to give is a servo browning out and
     * rebooting - which arrives as "stopped answering the bus" and hides the
     * cause. Let go and say why instead. */
    for (uint8_t i = 0; i < NJ; ++i) {
        if (gVolt[i] && gVolt[i] < UNDERVOLT_TENTHS) {
            relaxAll(F_VOLTS, IDS[i], "supply below 9.0 V - pack, wiring or a stall");
            return;
        }
    }

    if ((int32_t)(millis() - nextGoalMs) >= 0) {
        nextGoalMs = millis() + 10;
        for (uint8_t i = 0; i < NJ; ++i) {
            if (gWroteValid[i] && gTarget[i] == gWrote[i]) {
                continue;                    /* the servo already has this goal */
            }
            if (writeGoal(i)) { gWrote[i] = gTarget[i]; gWroteValid[i] = true; }
            /* A failed write leaves gWroteValid alone, so it is retried next
             * cycle rather than silently dropped. */
        }
    }
}
