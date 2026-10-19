"""
ACC-radar diagnostic poller (VW TP2.0 / KWP2000) for passive on-car data logging.

The B8 Q5 factory ACC radar (J428, 8R0907561A, "ACC3") is a KWP2000-over-TP2.0 module
reachable as TP2.0 LOGICAL ADDRESS 0x0B, directly on the extended CAN where stock ACC_01
is received (panda bus 2 / CanBus.cam). VCDS "address 13" maps to logical 0x0B on the wire.

This poller is TX-ONLY and OPEN-LOOP: it emits the diagnostic frames on a timer and never
parses responses in the control loop. The radar's replies (channel-setup D0, params A1,
KWP measuring-block data on the ECU->tester id) are captured in the normal rlog CAN stream
and decoded OFFLINE. Nothing here changes vehicle control; it only appends extra CAN frames.

Gated OFF by default (see carcontroller: env var VW_ACC_DIAG_POLL). Read-only diagnostics:
it opens a measuring-block read session (KWP 0x21) and never writes/codes/changes sessions
beyond StartDiagnosticSession, which VCDS/ODIS also use for live measured-value logging.

STAGED ROLLOUT
  MODE="probe": emit ONLY the channel-setup frame (CAN 0x200, 7 bytes) at ~2 Hz. The radar's
    setup response (D0) is logged; decode it offline to confirm bus-2 reachability and the
    radar-assigned channel ids (ECU_TX / TESTER_TX). Panda allowlist needs only {0x200,2,7}.
  MODE="poll": full open-loop read. Fill TESTER_TX/ECU_TX from the probe, add the Stage-2
    panda allowlist entries (see REQUIRED_TX_ALLOWLIST below), reflash panda, then drive.

On-wire framing (verified live + against the plaintext ODIS bench capture):
  setup   -> 0x200 : 0B C0 00 10 00 03 01
  resp    <- 0x20B : 00 D0 <ecuRX16LE> <ecuTX16LE> 01   (gateway gave ECU_TX=0x300, TESTER_TX=0x4AB)
  params  -> TESTER_TX : A0 0F 8A FF 32 FF ; ECU -> ECU_TX : A1 ...
  data    -> TESTER_TX : [0x1X|seq][len_hi len_lo][KWP...]  (single/last frame, nibble 0x1)
  measuring block N : KWP 21 NN  -> response 61 NN <fields...> on ECU_TX
"""

ACC_ADDR = 0x0B              # TP2.0 logical address of the ACC radar (VCDS "addr 13")
SETUP_ID = 0x200             # TP2.0 channel-setup request CAN id
SETUP_DATA = bytes([ACC_ADDR, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01])

# Radar-assigned channel ids. Defaults are the gateway-path values; CONFIRM from the probe
# log before switching MODE to "poll" (a direct bus-2 radar may assign different ids).
TESTER_TX = 0x4AB            # tester -> ECU (we transmit on this)
ECU_TX = 0x300              # ECU -> tester (responses arrive here; logged, not TXed)

MODE = "poll"                # "probe" (setup only) or "poll" (full read) -- see STAGED ROLLOUT
# Stage-1 probe (route 000001e7) confirmed the radar is direct on bus 2 and assigns the same
# channel as the gateway: ECU_TX=0x300, TESTER_TX=0x4AB (the defaults above), so no id change.

# Measuring-block groups to cycle through in poll mode (the ones found to carry data).
POLL_GROUPS = [1, 2, 7, 8, 10, 12, 13, 14, 15, 48, 49]

PARAMS_DATA = bytes([0xA0, 0x0F, 0x8A, 0xFF, 0x32, 0xFF])
KEEPALIVE_DATA = bytes([0xA3])

# Panda TX allowlist entries this poller needs on bus 2 (exact addr/bus/len match):
#   probe: {SETUP_ID(0x200), 2, 7}
#   poll : {SETUP_ID, 2, 7}, {TESTER_TX, 2, 6}(params), {TESTER_TX, 2, 1}(keepalive/ack),
#          {TESTER_TX, 2, 5}(block read)
REQUIRED_TX_ALLOWLIST = {
    "probe": [(SETUP_ID, 2, 7)],
    "poll": [(SETUP_ID, 2, 7), (TESTER_TX, 2, 6), (TESTER_TX, 2, 1), (TESTER_TX, 2, 5)],
}

# 100 Hz control loop (DT_CTRL = 0.01). Timings below are in frames.
_SETUP_EVERY = 50            # probe: re-send setup every 0.5 s
_CYCLE = 200                 # poll: re-open channel every 2.0 s
_READ_EVERY = 15             # poll: one block read every 0.15 s
_KEEPALIVE_EVERY = 50        # poll: channel test every 0.5 s


class AccDiagPoller:
    def __init__(self):
        self.seq = 0
        self.group_idx = 0

    def _read_frame(self, group):
        # [0x1{seq}][2-byte BE len=2][KWP: 21 NN]  -> single/last data frame, 5 bytes
        fr = bytes([0x10 | (self.seq & 0xF), 0x00, 0x02, 0x21, group & 0xFF])
        self.seq = (self.seq + 1) & 0xF
        return fr

    def update(self, frame, bus):
        """Return a list of (addr, data, bus) CAN frames to append to can_sends. Never raises."""
        try:
            if MODE == "probe":
                if frame % _SETUP_EVERY == 0:
                    return [(SETUP_ID, SETUP_DATA, bus)]
                return []

            # poll mode: open-loop channel maintenance + rotating block reads
            out = []
            phase = frame % _CYCLE
            if phase == 0:
                self.seq = 0
                out.append((SETUP_ID, SETUP_DATA, bus))                 # (re)open channel
            elif phase == 3:
                out.append((TESTER_TX, PARAMS_DATA, bus))               # channel params
            elif phase >= 5:
                if (phase % _KEEPALIVE_EVERY) == 0:
                    out.append((TESTER_TX, KEEPALIVE_DATA, bus))        # keep channel alive
                if (phase % _READ_EVERY) == 0:
                    group = POLL_GROUPS[self.group_idx % len(POLL_GROUPS)]
                    self.group_idx += 1
                    out.append((TESTER_TX, self._read_frame(group), bus))
            return out
        except Exception:
            return []
