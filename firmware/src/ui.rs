//! Two-screen UI in the kws-x1 style: a POWER monitor (live V/A/W) and a
//! PD screen (the engine view from the vendored fusb302-pd session,
//! r113). LEFT/RIGHT (and BACK) switch screens; OK on the PD screen
//! signals the PD task. Rendering goes through the Rust LCD driver —
//! every string here must only contain glyphs from lcd.rs FONT_CHARS.

use crate::buttons;
use crate::lcd::Lcd;
use crate::pd_task::View;

const BLACK: u16 = 0x0000;
const WHITE: u16 = 0xFFFF;
const GRAY: u16 = 0x7BEF;
const GREEN: u16 = 0x07E0;
const YELLOW: u16 = 0xFFE0;
const CYAN: u16 = 0x07FF;
const MAGENTA: u16 = 0xF81F;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Screen {
    Power,
    Pd,
    /// r189: the QC2.0 trigger screen (targets 5/9/12/20 V, OK = REQ).
    Qc,
    /// r196: the QC3.0 continuous screen (target in mV, 200 mV steps).
    Qc3,
    /// r198: the AFC screen (5 V / 9 V single-wire requests).
    Afc,
    /// r199: the ripple scope screen (kws-x1's Ripple tab).
    Ripple,
}

/// The screen cycle order (LEFT/RIGHT walk it in both directions).
const SCREEN_CYCLE: [Screen; 6] = [
    Screen::Power,
    Screen::Pd,
    Screen::Qc,
    Screen::Qc3,
    Screen::Afc,
    Screen::Ripple,
];

fn next_screen(screen: Screen, dir: i16) -> Screen {
    let i = SCREEN_CYCLE.iter().position(|&s| s == screen).unwrap_or(0);
    let next = (i as i16 + dir).rem_euclid(SCREEN_CYCLE.len() as i16) as usize;
    SCREEN_CYCLE[next]
}

/// What a button press means to the UI (buttons.rs order:
/// LEFT, OK, BACK, RIGHT).
pub enum Press {
    None,
    Switch(Screen),
    /// r173: the OK-commit in voltage selection — a PD-native voltage
    /// change (new Request from Ready), not a session restart.
    Commit,
    /// r171: cycle the request target one step (1 = RIGHT = next, -1 =
    /// LEFT = previous) while the PPS screen is in voltage selection.
    Voltage(i16),
    /// r172: the selection mode toggled — redraw the hint line.
    Hint,
    /// r199: cycle the ripple timebase (kws-x1 buttons.rs:1657 — a held
    /// capture cannot be re-timed, so this also drops the hold).
    RippleTime(i16),
    /// r199: cycle the ripple vertical scale (kws-x1 buttons.rs:1666).
    RippleScale(i16),
    /// r199: toggle the ripple run/hold (kws-x1 buttons.rs:1674).
    RippleHold,
}

/// r199: what LEFT/RIGHT adjust on the open ripple screen, in OK's order
/// (kws-x1 buttons.rs RippleFocus; long-OK's reset-to-defaults has no
/// fnb58 equivalent — no long-press support — so BACK only closes).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RippleFocus {
    Time,
    Scale,
    Capture,
}

impl RippleFocus {
    fn next(self) -> Self {
        match self {
            Self::Time => Self::Scale,
            Self::Scale => Self::Capture,
            Self::Capture => Self::Time,
        }
    }
}

pub struct Ui {
    pub screen: Screen,
    /// r172: the PPS screen's voltage-selection mode (OK enters, OK
    /// commits, BACK exits; LEFT/RIGHT cycle the target inside it).
    pub selecting: bool,
    /// PD-view identity at the last redraw (values that change rarely).
    pd_phase: u8,
    pd_mv: u16,
    pd_ma: u16,
    pd_err: u16,
    /// r171: the armed target voltage (PPS screen's TGT line).
    pd_tgt: u16,
    /// r189: QC-view identity at the last static redraw (state changes
    /// only — the fast numbers update in place, see draw_qc).
    qc_state: u8,
    qc_outcome: u8,
    qc_failure: u8,
    qc_tgt: u16,
    /// r195: the dynamic rows' last in-place update (ms, wrapping u32).
    qc_dyn_at: u32,
    /// r196: the QC3 screen's dynamic rows' last update (separate cadence
    /// from the QC2 screen's — each screen redraws independently).
    qc3_dyn_at: u32,
    /// r196: the QC3 screen's static-layer cache (state + target).
    qc3_state: u8,
    qc3_tgt: u16,
    /// r198: the AFC screen's caches (same pattern as the QC screens).
    afc_state: u8,
    afc_tgt: u16,
    afc_dyn_at: u32,
    /// r199: the ripple screen's state (kws-x1 Input's ripple_* fields,
    /// buttons.rs:964-968 — transient, never persisted).
    pub ripple_focus: Option<RippleFocus>,
    pub ripple_hold: bool,
    pub ripple_timebase: ripple_core::ripple::Timebase,
    pub ripple_scale: ripple_core::ripple::Scale,
    /// r199: the ripple plot's last redraw (100 ms cadence like the
    /// capture) and the chrome identity at it (focus/hold/scale/timebase
    /// packed — a change redraws immediately, not on the next capture).
    ripple_at: u32,
    ripple_tag: u16,
}

/// Draw a NUL-terminated copy of `s` (truncated to fit the buffer).
async fn text(lcd: &mut Lcd, x: u16, y: u16, s: &[u8], scale: u8, color: u16) {
    let mut buf = [0u8; 24];
    let n = s.len().min(23);
    buf[..n].copy_from_slice(&s[..n]);
    lcd.draw_text(x, y, &buf, scale, color, BLACK).await;
}

impl Ui {
    pub fn new() -> Self {
        Self {
            screen: Screen::Power,
            selecting: false,
            pd_phase: 0xFF,
            pd_mv: 0xFFFF,
            pd_ma: 0xFFFF,
            pd_err: 0xFFFF,
            pd_tgt: 0xFFFF,
            qc_state: 0xFF,
            qc_outcome: 0xFF,
            qc_failure: 0xFF,
            qc_tgt: 0xFFFF,
            qc_dyn_at: 0,
            qc3_dyn_at: 0,
            qc3_state: 0xFF,
            qc3_tgt: 0xFFFF,
            afc_state: 0xFF,
            afc_tgt: 0xFFFF,
            afc_dyn_at: 0,
            ripple_focus: None,
            ripple_hold: false,
            ripple_timebase: ripple_core::ripple::Timebase::default(),
            ripple_scale: ripple_core::ripple::Scale::default(),
            ripple_at: 0,
            ripple_tag: 0xFFFF,
        }
    }

    /// r172 navigation (user spec): LEFT/RIGHT move between screens; OK on
    /// the PPS screen ENTERS voltage selection; inside it LEFT/RIGHT choose
    /// the voltage, OK selects it (re-request), BACK returns to screen
    /// navigation. Only glyphs from lcd.rs FONT_CHARS (no '<'/'>'!).
    pub fn on_press(&mut self, i: usize) -> Press {
        // r199: the ripple screen's focus model (kws-x1 buttons.rs:1714
        // OK opens/cycles; 1657-1676 LEFT/RIGHT adjust the focused
        // setting). Unhandled presses fall through to the screen-level
        // navigation below.
        if self.screen == Screen::Ripple {
            match i {
                buttons::OK => {
                    self.ripple_focus = Some(match self.ripple_focus {
                        None => RippleFocus::Time,
                        Some(focus) => focus.next(),
                    });
                    return Press::Hint;
                }
                buttons::BACK if self.ripple_focus.is_some() => {
                    self.ripple_focus = None;
                    return Press::Hint;
                }
                buttons::LEFT | buttons::RIGHT if self.ripple_focus.is_some() => {
                    let dir: i16 = if i == buttons::RIGHT { 1 } else { -1 };
                    return match self.ripple_focus {
                        Some(RippleFocus::Time) => Press::RippleTime(dir),
                        Some(RippleFocus::Scale) => Press::RippleScale(dir),
                        Some(RippleFocus::Capture) => Press::RippleHold,
                        None => Press::None,
                    };
                }
                _ => {}
            }
        }
        match i {
            buttons::LEFT => {
                if self.selecting {
                    Press::Voltage(-1)
                } else {
                    Press::Switch(next_screen(self.screen, -1))
                }
            }
            buttons::RIGHT => {
                if self.selecting {
                    Press::Voltage(1)
                } else {
                    Press::Switch(next_screen(self.screen, 1))
                }
            }
            buttons::BACK => {
                if self.selecting {
                    self.selecting = false;
                    Press::Hint
                } else {
                    Press::Switch(next_screen(self.screen, -1))
                }
            }
            // r196 fix: the OK arm missed Screen::Qc3 — on that screen OK
            // fell through to Press::None (a dead button; four presses,
            // zero commits in the first hardware log). r198: Afc joined
            // the list up front.
            buttons::OK
                if matches!(
                    self.screen,
                    Screen::Pd | Screen::Qc | Screen::Qc3 | Screen::Afc
                ) =>
            {
                if self.selecting {
                    self.selecting = false;
                    Press::Commit
                } else {
                    self.selecting = true;
                    Press::Hint
                }
            }
            _ => Press::None,
        }
    }

    /// Switch screens: full clear, title and static chrome. Any switch
    /// also leaves voltage-selection mode. r199: a ripple hold lasts only
    /// while Ripple is shown (kws-x1 buttons.rs:1421-1424 — coming back
    /// starts live), so leaving clears it and stops captures.
    pub async fn enter(&mut self, lcd: &mut Lcd, screen: Screen) {
        if screen != Screen::Ripple {
            self.ripple_focus = None;
            self.ripple_hold = false;
        }
        self.screen = screen;
        self.selecting = false;
        // Force the next screen-specific redraw.
        self.pd_phase = 0xFF;
        self.qc_state = 0xFF;
        self.qc3_state = 0xFF;
        self.afc_state = 0xFF;
        self.ripple_at = 0;
        crate::ripple::LIVE.store(
            screen == Screen::Ripple && !self.ripple_hold,
            core::sync::atomic::Ordering::Relaxed,
        );
        lcd.fill(BLACK).await;
        match screen {
            Screen::Power => {
                text(lcd, 8, 8, b"POWER", 2, WHITE).await;
                text(lcd, 272, 8, b"1/6", 2, GRAY).await;
            }
            Screen::Pd => {
                text(lcd, 8, 8, b"PPS", 2, WHITE).await;
                text(lcd, 272, 8, b"2/6", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Qc => {
                text(lcd, 8, 8, b"QC", 2, WHITE).await;
                text(lcd, 272, 8, b"3/6", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Qc3 => {
                text(lcd, 8, 8, b"QC3", 2, WHITE).await;
                text(lcd, 272, 8, b"4/6", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Afc => {
                text(lcd, 8, 8, b"AFC", 2, WHITE).await;
                text(lcd, 272, 8, b"5/6", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Ripple => {
                text(lcd, 8, 8, b"RIPPLE", 2, WHITE).await;
                text(lcd, 272, 8, b"6/6", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
        }
    }

    /// r172: the PPS screen's mode-dependent hint line. A single small
    /// text draw — safe to run while a PD session is busy. r199: the
    /// ripple screen's focus-mode hints (kws-x1 ui.rs:1732-1737, the
    /// long-OK reset dropped — no long-press support). r203: the whole
    /// row is erased first — only the drawn glyphs repaint their cells,
    /// so a SHORTER hint left the previous one's tail (the 'rund' ghost
    /// class).
    pub async fn draw_hint(&self, lcd: &mut Lcd) {
        lcd.draw_text(8, 216, &[b' '; 26], 2, GRAY, BLACK).await;
        let hint: &[u8] = match self.screen {
            Screen::Ripple => match self.ripple_focus {
                None => b"OK:SET",
                Some(RippleFocus::Time) => b"LR:TIME OK:SCALE BK:X",
                Some(RippleFocus::Scale) => b"LR:SCALE OK:RUN BK:X",
                Some(RippleFocus::Capture) => b"LR:RUNHOLD OK:TIME BK:X",
            },
            _ if self.selecting => b"LR SEL OK:REQ BK:EX",
            _ => b"OK:TGT",
        };
        lcd.draw_text(8, 216, hint, 2, GRAY, BLACK).await;
    }

    /// r189: the QC screen — state, target, the measured line taps and
    /// the published output. r195 flicker fix: the static rows (fill +
    /// TGT + state + detail) redraw only when the state changes; the
    /// fast tap/output numbers update IN PLACE (fixed-width, no fill)
    /// at a 200 ms cadence — the r194 full fill re-fired on every 50 ms
    /// tap swing (released lines float by hundreds of mV) and the screen
    /// never stopped flashing.
    pub async fn draw_qc(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        let state = crate::qc_task::STATE.load(Ordering::Relaxed);
        let outcome = crate::qc_task::OUTCOME.load(Ordering::Relaxed);
        let failure = crate::qc_task::FAILURE.load(Ordering::Relaxed);
        let tgt_idx = crate::qc_task::TARGET.load(Ordering::Relaxed) as usize;
        let tgt = crate::qc_task::TARGETS_MV
            .get(tgt_idx)
            .copied()
            .unwrap_or(5000);
        let statics_changed = state != self.qc_state
            || outcome != self.qc_outcome
            || failure != self.qc_failure
            || tgt != self.qc_tgt;
        let now = embassy_time::Instant::now().as_millis() as u32;
        let dyn_due = now.wrapping_sub(self.qc_dyn_at) >= 200;
        if !statics_changed && !dyn_due {
            return;
        }
        if statics_changed {
            self.qc_state = state;
            self.qc_outcome = outcome;
            self.qc_failure = failure;
            self.qc_tgt = tgt;
            lcd.fill_window(0, 32, 319, 210, BLACK).await;

            // The next OK requests exactly this target.
            let mut line = [0u8; 24];
            let mut n = 0;
            for &b in b"TGT " {
                line[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut line, n, u32::from(tgt / 1000));
            line[n] = b'V';
            n += 1;
            line[n] = 0;
            text(lcd, 8, 48, &line, 2, YELLOW).await;

        // State row.
        const STATES: [&[u8]; 6] = [
            b"OFF",
            b"RELEASE",
            b"ENTRY",
            b"ACTIVE",
            b"FAIL",
            b"STEP",
        ];
            let name = STATES.get(state as usize).copied().unwrap_or(b"?");
            let mut line = [0u8; 24];
            let mut n = 0;
            for &b in b"QC " {
                line[n] = b;
                n += 1;
            }
            line[n..n + name.len()].copy_from_slice(name);
            n += name.len();
            line[n] = 0;
            text(lcd, 8, 88, &line, 3, WHITE).await;

            // Detail row: the failure reason, or the output-follow verdict.
            let detail: &[u8] = match (state, failure, outcome) {
                (4, 1, _) => b"FAIL NODCP",
                (4, 2, _) => b"FAIL NOENTRY",
                (3, _, 1) => b"WAIT",
                (3, _, 2) => b"FOLLOWED",
                (3, _, 3) => b"IGNORED",
                _ => b"",
            };
            if !detail.is_empty() {
                let mut line = [0u8; 24];
                line[..detail.len()].copy_from_slice(detail);
                line[detail.len()] = 0;
                text(lcd, 8, 132, &line, 2, CYAN).await;
            }
        }
        if !(statics_changed || dyn_due) {
            return;
        }
        self.qc_dyn_at = now;
        // The dynamic rows, in place: space-padded fixed-width numbers so
        // a redraw erases stale digits — no fill, no flicker.
        let dp = crate::qc_task::DP_MV.load(Ordering::Relaxed);
        let dm = crate::qc_task::DM_MV.load(Ordering::Relaxed);
        let vb = crate::qc_task::VBUS_MV.load(Ordering::Relaxed);
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"DP " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(dp));
        while n < 7 {
            line[n] = b' ';
            n += 1;
        }
        for &b in b" DM " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(dm));
        while n < 15 {
            line[n] = b' ';
            n += 1;
        }
        line[n] = 0;
        lcd.draw_text(8, 168, &line, 2, GREEN, BLACK).await;

        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"VB " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(vb));
        while n < 8 {
            line[n] = b' ';
            n += 1;
        }
        for &b in b"mV" {
            line[n] = b;
            n += 1;
        }
        line[n] = 0;
        lcd.draw_text(8, 196, &line, 2, WHITE, BLACK).await;
    }

    /// r196: the QC3 screen — target (mV), state, the crate's tracked
    /// continuous set point, and the taps/output. Same split as draw_qc:
    /// the static rows (fill + TGT + state) redraw only on state/target
    /// changes; the fast numbers update in place at 200 ms.
    pub async fn draw_qc3(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        let state = crate::qc_task::STATE.load(Ordering::Relaxed);
        let failure = crate::qc_task::FAILURE.load(Ordering::Relaxed);
        let tgt = crate::qc_task::M3_TARGET.load(Ordering::Relaxed);
        let set = crate::qc_task::SET_MV.load(Ordering::Relaxed);
        let dp = crate::qc_task::DP_MV.load(Ordering::Relaxed);
        let dm = crate::qc_task::DM_MV.load(Ordering::Relaxed);
        let vb = crate::qc_task::VBUS_MV.load(Ordering::Relaxed);
        let statics_changed = state != self.qc3_state || tgt != self.qc3_tgt;
        let now = embassy_time::Instant::now().as_millis() as u32;
        let dyn_due = now.wrapping_sub(self.qc3_dyn_at) >= 200;
        if !statics_changed && !dyn_due {
            return;
        }
        if statics_changed {
            self.qc3_state = state;
            self.qc3_tgt = tgt;
            lcd.fill_window(0, 32, 319, 210, BLACK).await;

            // The next OK steps to exactly this target.
            let mut line = [0u8; 24];
            let mut n = 0;
            for &b in b"TGT " {
                line[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut line, n, u32::from(tgt));
            for &b in b"mV" {
                line[n] = b;
                n += 1;
            }
            line[n] = 0;
            text(lcd, 8, 48, &line, 2, YELLOW).await;

            const STATES: [&[u8]; 6] = [
                b"OFF",
                b"RELEASE",
                b"ENTRY",
                b"ACTIVE",
                b"FAIL",
                b"STEP",
            ];
            let name = STATES.get(state as usize).copied().unwrap_or(b"?");
            let mut line = [0u8; 24];
            let mut n = 0;
            for &b in b"QC3 " {
                line[n] = b;
                n += 1;
            }
            line[n..n + name.len()].copy_from_slice(name);
            n += name.len();
            line[n] = 0;
            text(lcd, 8, 88, &line, 3, WHITE).await;

            let detail: &[u8] = match (state, failure) {
                (4, 1) => b"FAIL NODCP",
                (4, 2) => b"FAIL NOENTRY",
                _ => b"",
            };
            if !detail.is_empty() {
                let mut line = [0u8; 24];
                line[..detail.len()].copy_from_slice(detail);
                line[detail.len()] = 0;
                text(lcd, 8, 132, &line, 2, CYAN).await;
            }
        }
        if !(statics_changed || dyn_due) {
            return;
        }
        self.qc3_dyn_at = now;

        // SET row (the crate's tracked continuous set point) + the taps,
        // in place: space-padded fixed-width blocks erase stale digits.
        let mut line = [0u8; 32];
        let mut n = 0;
        for &b in b"SET " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(set));
        while n < 12 {
            line[n] = b' ';
            n += 1;
        }
        for &b in b"DP " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(dp));
        while n < 20 {
            line[n] = b' ';
            n += 1;
        }
        for &b in b"DM " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(dm));
        while n < 28 {
            line[n] = b' ';
            n += 1;
        }
        line[n] = 0;
        lcd.draw_text(8, 168, &line, 2, GREEN, BLACK).await;

        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"VB " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(vb));
        while n < 8 {
            line[n] = b' ';
            n += 1;
        }
        for &b in b"mV" {
            line[n] = b;
            n += 1;
        }
        line[n] = 0;
        lcd.draw_text(8, 196, &line, 2, WHITE, BLACK).await;
    }

    /// r198: the AFC screen — target (5 V/9 V), state, failure reason,
    /// the line taps and the output. Same flicker-free split.
    pub async fn draw_afc(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        let state = crate::qc_task::STATE.load(Ordering::Relaxed);
        let failure = crate::qc_task::FAILURE.load(Ordering::Relaxed);
        let tgt = crate::qc_task::AFC_TARGET.load(Ordering::Relaxed);
        let dp = crate::qc_task::DP_MV.load(Ordering::Relaxed);
        let dm = crate::qc_task::DM_MV.load(Ordering::Relaxed);
        let vb = crate::qc_task::VBUS_MV.load(Ordering::Relaxed);
        let statics_changed = state != self.afc_state || tgt != self.afc_tgt;
        let now = embassy_time::Instant::now().as_millis() as u32;
        let dyn_due = now.wrapping_sub(self.afc_dyn_at) >= 200;
        if !statics_changed && !dyn_due {
            return;
        }
        if statics_changed {
            self.afc_state = state;
            self.afc_tgt = tgt;
            lcd.fill_window(0, 32, 319, 210, BLACK).await;

            // The next OK requests exactly this target.
            let mut line = [0u8; 24];
            let mut n = 0;
            for &b in b"TGT " {
                line[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut line, n, u32::from(tgt / 1000));
            line[n] = b'V';
            n += 1;
            line[n] = 0;
            text(lcd, 8, 48, &line, 2, YELLOW).await;

            // State row: the shared STATE machine, AFC-labeled
            // (6 = the wire detection exchange, 7 = the AFC session).
            const STATES: [&[u8]; 8] = [
                b"OFF", b"RELEASE", b"ENTRY", b"ACTIVE", b"FAIL", b"STEP", b"DETECT", b"AFC",
            ];
            let name = STATES.get(state as usize).copied().unwrap_or(b"?");
            let mut line = [0u8; 24];
            let mut n = 0;
            for &b in b"AFC " {
                line[n] = b;
                n += 1;
            }
            line[n..n + name.len()].copy_from_slice(name);
            n += name.len();
            line[n] = 0;
            text(lcd, 8, 88, &line, 3, WHITE).await;

            // Failure reasons: the QC names plus the AFC-specific codes
            // (3 = SILENT, 4 = wire/promote damage).
            let detail: &[u8] = match (state, failure) {
                (4, 1) => b"FAIL NODCP",
                (4, 2) => b"FAIL NOENTRY",
                (4, 3) => b"FAIL SILENT",
                (4, 4) => b"FAIL WIRE",
                _ => b"",
            };
            if !detail.is_empty() {
                let mut line = [0u8; 24];
                line[..detail.len()].copy_from_slice(detail);
                line[detail.len()] = 0;
                text(lcd, 8, 132, &line, 2, CYAN).await;
            }
        }
        self.afc_dyn_at = now;

        // The dynamic rows, in place (fixed-width, space-padded).
        let mut line = [0u8; 32];
        let mut n = 0;
        for &b in b"DP " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(dp));
        while n < 12 {
            line[n] = b' ';
            n += 1;
        }
        for &b in b"DM " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(dm));
        while n < 20 {
            line[n] = b' ';
            n += 1;
        }
        line[n] = 0;
        lcd.draw_text(8, 168, &line, 2, GREEN, BLACK).await;

        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"VB " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(vb));
        while n < 8 {
            line[n] = b' ';
            n += 1;
        }
        for &b in b"mV" {
            line[n] = b;
            n += 1;
        }
        line[n] = 0;
        lcd.draw_text(8, 196, &line, 2, WHITE, BLACK).await;
    }

    /// r199: the ripple screen — the kws-x1 draw_ripple port
    /// (kws-x1 ui.rs:1465) on this 320x240 panel: p-p/RMS above a
    /// 224x96 plot with 8x4 divisions, the spectrum's 224 bars behind
    /// the 112-column envelope trace, peak frequencies at the plot's
    /// top-right, then bus + scale, run/hold + status, timebase.
    /// DEVIATION from kws-x1's drawing (analysis identical): the trace
    /// is 2-px envelope bars without the centre-to-centre connector
    /// segments (no arbitrary-line primitive on this LCD driver), and
    /// the per-bar digit markers at the bar feet are dropped in favour
    /// of the numbered peak list alone.
    pub async fn draw_ripple(
        &mut self,
        lcd: &mut Lcd,
        reading: Option<&ripple_core::ripple::Reading>,
        reference: Option<u16>,
    ) {
        use ripple_core::ripple::{
            FULL_SCALE, HORIZONTAL_DIVISIONS, VERTICAL_DIVISIONS, Scale, Timebase,
        };
        use ripple_core::spectrum::{self, FULL_HEIGHT};

        // Colors: the grid dim gray, bars dim purple (kws-x1's RULE /
        // SPECTRUM / SPECTRUM_LABEL roles).
        const RULE: u16 = 0x3186;
        const SPECTRUM: u16 = 0x200A;
        const SPECTRUM_LABEL: u16 = 0x60D4;
        const ORANGE: u16 = 0xFD20;
        // Pixels: metrics above the plot, live bus + scale below it,
        // then run/hold + status and the timebase (kws-x1 layout rows).
        const VALUE_Y: u16 = 34;
        const TOP: i32 = 56;
        const BOTTOM: i32 = 151;
        const LEFT: i32 = 8;
        const PLOT_W: i32 = 224;
        const BUS_Y: u16 = 160;
        const STATUS_Y: u16 = 188;
        const COLUMN_PITCH: i32 = 2;

        // Redraw on the capture cadence or when the chrome changed.
        let scale_index = Scale::ALL
            .iter()
            .position(|&s| s == self.ripple_scale)
            .unwrap_or(0);
        let tb_index = Timebase::ALL
            .iter()
            .position(|&t| t == self.ripple_timebase)
            .unwrap_or(0);
        let tag = (match self.ripple_focus {
            None => 0,
            Some(RippleFocus::Time) => 1,
            Some(RippleFocus::Scale) => 2,
            Some(RippleFocus::Capture) => 3,
        } | (u16::from(self.ripple_hold) << 2)
            | ((scale_index as u16) << 3)
            | ((tb_index as u16) << 7)) as u16;
        let now = embassy_time::Instant::now().as_millis() as u32;
        if tag == self.ripple_tag && now.wrapping_sub(self.ripple_at) < 100 {
            return;
        }
        self.ripple_tag = tag;
        self.ripple_at = now;

        let reference_mv = crate::ripple::REFERENCE_MV;
        let reference = reference.filter(|&count| count > 0 && count < FULL_SCALE);
        let normalized = reading.zip(reference);
        let mv = normalized
            .and_then(|(r, reference)| r.peak_to_peak_mv(reference, reference_mv));
        let rms = normalized
            .and_then(|(r, reference)| r.rms_tenths_mv(reference, reference_mv));

        // p-p left, RMS right — EVERY row is a fixed-width cell (leading
        // or trailing spaces painted at the same origin), so a shorter
        // value erases the previous draw's tail (the r202 'rund' ghosts).
        // The p-p cell is exactly 11 chars (4 value + " mV p-p").
        let mut line = [b' '; 11];
        let mut n = 0;
        // Four value digits ("  12"), right-aligned in the cell.
        let mut digits = [0u8; 8];
        let dn = match mv {
            Some(mv) => crate::push_dec(&mut digits, 0, u32::from(mv)),
            None => {
                digits[..2].copy_from_slice(b"--");
                2
            }
        };
        for _ in dn..4 {
            line[n] = b' ';
            n += 1;
        }
        line[n..n + dn].copy_from_slice(&digits[..dn]);
        n += dn;
        for &b in b" mV p-p" {
            line[n] = b;
            n += 1;
        }
        text(lcd, 8, VALUE_Y, &line, 2, GREEN).await;

        // "RMS 1650.0 mV": the number is 6 wide, the whole cell 13.
        let mut line = [b' '; 16];
        let mut n = 0;
        for &b in b"RMS " {
            line[n] = b;
            n += 1;
        }
        let mut digits = [0u8; 8];
        let dn = match rms {
            Some(tenths) => {
                let mut dn = crate::push_dec(&mut digits, 0, u32::from(tenths / 10));
                digits[dn] = b'.';
                digits[dn + 1] = b'0' + (tenths % 10) as u8;
                dn += 2;
                dn
            }
            None => {
                digits[..2].copy_from_slice(b"--");
                2
            }
        };
        for _ in dn..6 {
            line[n] = b' ';
            n += 1;
        }
        line[n..n + dn].copy_from_slice(&digits[..dn]);
        n += dn;
        for &b in b" mV" {
            line[n] = b;
            n += 1;
        }
        // Scale-2 text advances 12 px per char; fixed 13-char cell.
        text(lcd, 312 - 12 * 13, VALUE_Y, &line, 2, WHITE).await;

        // The plot: grid, spectrum bars, envelope trace.
        let plot = reading.map(|r| r.plot_scale(self.ripple_scale, reference, reference_mv));
        lcd.fill_window(LEFT as u16, TOP as u16, (LEFT + PLOT_W - 1) as u16, BOTTOM as u16, BLACK)
            .await;
        if let (Some(reading), Some(plot)) = (reading, plot) {
            let y = |count: u16| {
                let count = i32::from(count).clamp(plot.min, plot.max);
                BOTTOM - (count - plot.min) * (BOTTOM - TOP) / (plot.max - plot.min)
            };
            for division in 0..=VERTICAL_DIVISIONS as i32 {
                let row = TOP + division * (BOTTOM - TOP) / VERTICAL_DIVISIONS as i32;
                lcd.fill_window(LEFT as u16, row as u16, (LEFT + PLOT_W - 1) as u16, row as u16, RULE)
                    .await;
            }
            for division in 0..=HORIZONTAL_DIVISIONS as i32 {
                let x = LEFT + division * (PLOT_W - 1) / HORIZONTAL_DIVISIONS as i32;
                lcd.fill_window(x as u16, TOP as u16, x as u16, BOTTOM as u16, RULE)
                    .await;
            }
            // The spectrum behind the trace: frequency left to right,
            // heights relative to the strongest bin.
            for (column, &height) in reading.spectrum.bars.iter().enumerate() {
                let bar = i32::from(height) * (BOTTOM - TOP) / i32::from(FULL_HEIGHT);
                if bar > 0 {
                    let x = LEFT + column as i32;
                    lcd.fill_window(
                        x as u16,
                        (BOTTOM - bar) as u16,
                        x as u16,
                        BOTTOM as u16,
                        SPECTRUM,
                    )
                    .await;
                }
            }
            for (column, bin) in reading.trace.iter().enumerate() {
                let x = LEFT + column as i32 * COLUMN_PITCH;
                let low = y(bin.min);
                let high = y(bin.max);
                lcd.fill_window(
                    x as u16,
                    low as u16,
                    (x + 1) as u16,
                    high as u16,
                    GREEN,
                )
                .await;
            }
            // Peak frequencies at the plot's top-right (tenths of a kHz
            // below 100 kHz, whole kHz above).
            if let Some(rate) = reading
                .timebase
                .sampling(crate::ripple::ADC_CLOCK_HZ)
                .map(|sampling| sampling.rate_hz(crate::ripple::ADC_CLOCK_HZ))
            {
                let mut line = [0u8; 24];
                let mut n = 0;
                for (index, position) in reading.spectrum.peaks.iter().enumerate() {
                    let Some(position) = position else { break };
                    if index > 0 {
                        line[n] = b' ';
                        n += 1;
                    }
                    line[n] = b'1' + index as u8;
                    line[n + 1] = b':';
                    n += 2;
                    let hz = spectrum::frequency_hz(*position, rate);
                    if hz < 100_000 {
                        let tenths = (hz + 50) / 100;
                        n = crate::push_dec(&mut line, n, tenths / 10);
                        line[n] = b'.';
                        line[n + 1] = b'0' + (tenths % 10) as u8;
                        n += 2;
                    } else {
                        n = crate::push_dec(&mut line, n, (hz + 500) / 1000);
                    }
                    line[n] = b'k';
                    n += 1;
                }
                if n > 0 {
                    // Scale-1 text advances 6 px per char.
                    let x = LEFT + PLOT_W - 2 - 6 * n as i32;
                    text(lcd, x as u16, (TOP + 2) as u16, &line, 1, SPECTRUM_LABEL).await;
                }
            }
        } else {
            let label = b"NO CAPTURE";
            text(
                lcd,
                (LEFT + PLOT_W / 2 - 6 * label.len() as i32 / 2) as u16,
                ((TOP + BOTTOM) / 2 - 7) as u16,
                label,
                2,
                GRAY,
            )
            .await;
        }

        // Bus row: left the published output, right the scale setting.
        let vb = crate::qc_task::VBUS_MV.load(core::sync::atomic::Ordering::Relaxed);
        let mut line = [0u8; 16];
        let mut n = 0;
        for &b in b"VB " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(vb / 1000));
        line[n] = b'.';
        line[n + 1] = b'0' + ((vb / 100) % 10) as u8;
        n += 2;
        line[n] = b'V';
        n += 1;
        while n < 10 {
            line[n] = b' ';
            n += 1;
        }
        text(lcd, 8, BUS_Y, &line, 2, WHITE).await;

        // The scale label, kws-x1's four cases (ui.rs:1550-1566): the
        // plot's own mV/div once one exists, the chosen scale before a
        // capture, raw counts when no reference backs a voltage. Built in
        // a scratch, then drawn in a fixed 14-char right-aligned cell so
        // a SHORTER label's leading edge cannot leave the old one's tail.
        let mut scratch = [0u8; 16];
        let mut n = 0;
        let auto = self.ripple_scale == Scale::Auto;
        match (plot.and_then(|plot| plot.mv_per_div), reading.is_none()) {
            (Some(mv), _) => {
                for &b in if auto { b"AUTO " } else { b"LOCK " } {
                    scratch[n] = b;
                    n += 1;
                }
                n = crate::push_dec(&mut scratch, n, u32::from(mv));
                for &b in b"mV/div" {
                    scratch[n] = b;
                    n += 1;
                }
            }
            (None, true) if auto => {
                for &b in b"AUTO" {
                    scratch[n] = b;
                    n += 1;
                }
            }
            (None, true) => {
                for &b in b"LOCK " {
                    scratch[n] = b;
                    n += 1;
                }
                n = crate::push_dec(
                    &mut scratch,
                    n,
                    u32::from(self.ripple_scale.millivolts().unwrap_or(0)),
                );
                for &b in b"mV/div" {
                    scratch[n] = b;
                    n += 1;
                }
            }
            (None, false) => {
                for &b in b"RAW AUTO" {
                    scratch[n] = b;
                    n += 1;
                }
            }
        }
        let mut scale_buf = [b' '; 14];
        scale_buf[14 - n..].copy_from_slice(&scratch[..n]);
        text(
            lcd,
            312 - 12 * 14,
            BUS_Y,
            &scale_buf,
            2,
            if self.ripple_focus == Some(RippleFocus::Scale) {
                YELLOW
            } else {
                GRAY
            },
        )
        .await;

        // Run/hold + status, then the timebase — all fixed-width cells.
        let capture: &[u8] = if self.ripple_hold {
            b"HOLD"
        } else {
            b"RUN "
        };
        let mut buf = [0u8; 8];
        buf[..capture.len()].copy_from_slice(capture);
        text(
            lcd,
            8,
            STATUS_Y,
            &buf,
            2,
            if self.ripple_focus == Some(RippleFocus::Capture) {
                YELLOW
            } else if self.ripple_hold {
                WHITE
            } else {
                GRAY
            },
        )
        .await;

        let clipped = reading.is_some_and(|r| r.clipped());
        let limited = plot.is_some_and(|plot| plot.limited);
        let status: &[u8] = match reading {
            None => b"NO CAPTURE",
            Some(_) if clipped => b"CLIPPED",
            Some(_) if mv.is_none() => b"NO REF",
            Some(_) if limited => b"OFF SCALE",
            Some(r) if r.triggered => b"T",
            Some(_) => b" ",
        };
        // Ten-char cell at a fixed origin erases every predecessor.
        let mut buf = [b' '; 10];
        buf[..status.len()].copy_from_slice(status);
        text(
            lcd,
            68,
            STATUS_Y,
            &buf,
            2,
            if clipped || limited { ORANGE } else { GRAY },
        )
        .await;

        // `~` marks a nominal, unmeasured timebase (kws-x1's mark), in a
        // fixed 10-char right-aligned cell ("~500us/div" is the longest).
        let timebase = reading.map_or(self.ripple_timebase, |r| r.timebase);
        let timebase_ok = timebase.sampling(crate::ripple::ADC_CLOCK_HZ).is_some();
        let mut scratch = [0u8; 12];
        let mut n = 0;
        if timebase_ok {
            scratch[n] = b'~';
            n += 1;
            n = crate::push_dec(&mut scratch, n, u32::from(timebase.us_per_div()));
        } else {
            for &b in b"--" {
                scratch[n] = b;
                n += 1;
            }
        }
        for &b in b"us/div" {
            scratch[n] = b;
            n += 1;
        }
        let mut line = [b' '; 10];
        line[10 - n..].copy_from_slice(&scratch[..n]);
        text(
            lcd,
            312 - 12 * 10,
            STATUS_Y,
            &line,
            2,
            if self.ripple_focus == Some(RippleFocus::Time) {
                YELLOW
            } else {
                GRAY
            },
        )
        .await;
    }

    /// Show the reset-cause flags in the title bar so a reboot loop is
    /// visible standalone (no PC needed).
    pub async fn draw_rst(&self, lcd: &mut Lcd, flags: u8) {
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"rst=" {
            line[n] = b;
            n += 1;
        }
        const HEX: &[u8; 16] = b"0123456789abcdef";
        line[n] = HEX[(flags >> 4) as usize];
        line[n + 1] = HEX[(flags & 0xF) as usize];
        n += 2;
        line[n] = 0;
        lcd.draw_text(180, 8, &line, 2, GRAY, BLACK).await;
    }

    /// Power screen: the three big live values (12-char padded overdraws,
    /// so shrinking values leave no ghosts).
    pub async fn draw_power(&self, lcd: &mut Lcd, mv: u32, ua: i32, mw: i32) {
        // Voltage: "20.000 V"
        let mut l1 = Line12::new();
        l1.num_u32(mv, 3);
        l1.push(b" V");
        l1.pad();
        lcd.draw_text(20, 50, l1.bytes(), 3, GREEN, BLACK).await;

        // Current: A (>= 1 A) else mA.
        let mut l2 = Line12::new();
        let neg = ua < 0;
        let a = (if neg { -ua } else { ua }) as u32;
        if a >= 1_000_000 {
            l2.sign(neg);
            l2.num_u32(a / 1_000, 3);
            l2.push(b" A");
        } else {
            l2.sign(neg);
            l2.num_u32(a / 100, 1);
            l2.push(b" mA");
        }
        l2.pad();
        lcd.draw_text(20, 110, l2.bytes(), 3, YELLOW, BLACK).await;

        // Power: W (>= 1 W) else mW.
        let mut l3 = Line12::new();
        let negp = mw < 0;
        let p = (if negp { -mw } else { mw }) as u32;
        if p >= 1000 {
            l3.sign(negp);
            l3.num_u32(p, 3);
            l3.push(b" W");
        } else {
            l3.sign(negp);
            l3.num_u32(p, 0);
            l3.push(b" mW");
        }
        l3.pad();
        lcd.draw_text(20, 170, l3.bytes(), 3, CYAN, BLACK).await;

        // r192: the data-line taps (PA3 = D+, PA2 = D−, user trace) read
        // by the QC task — shown on the power screen so plug/unplug is
        // visible without navigating. Fixed-width decimal digits so a
        // redraw overwrites the previous value cleanly (no Line12: it
        // inserts decimal points and caps at 12 chars).
        let dp = crate::qc_task::DP_MV.load(core::sync::atomic::Ordering::Relaxed);
        let dm = crate::qc_task::DM_MV.load(core::sync::atomic::Ordering::Relaxed);
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"DP " {
            line[n] = b;
            n += 1;
        }
        for div in [1000u16, 100, 10, 1] {
            line[n] = b'0' + ((dp / div) % 10) as u8;
            n += 1;
        }
        for &b in b" DM " {
            line[n] = b;
            n += 1;
        }
        for div in [1000u16, 100, 10, 1] {
            line[n] = b'0' + ((dm / div) % 10) as u8;
            n += 1;
        }
        line[n] = 0;
        lcd.draw_text(20, 202, &line, 1, GRAY, BLACK).await;
    }

    /// PD screen: the engine view (vendored fusb302-pd session). Redrawn
    /// only when a tracked value changes.
    pub async fn draw_pd(&mut self, lcd: &mut Lcd, view: &View) {
        let phase = view.phase.load(core::sync::atomic::Ordering::Relaxed);
        let mv = view.selected_mv.load(core::sync::atomic::Ordering::Relaxed);
        let ma = view.selected_ma.load(core::sync::atomic::Ordering::Relaxed);
        let err = view.error.load(core::sync::atomic::Ordering::Relaxed);
        let tgt = crate::pd_task::DESIRED_MV.load(core::sync::atomic::Ordering::Relaxed);
        if phase == self.pd_phase
            && mv == self.pd_mv
            && ma == self.pd_ma
            && err == self.pd_err
            && tgt == self.pd_tgt
        {
            return;
        }
        self.pd_phase = phase;
        self.pd_mv = mv;
        self.pd_ma = ma;
        self.pd_err = err;
        self.pd_tgt = tgt;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;

        // r171: the armed request target, cycled with LEFT/RIGHT:
        // "TGT 9V" (the next OK requests exactly this).
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"TGT " {
            line[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut line, n, u32::from(tgt / 1000));
        line[n] = b'V';
        n += 1;
        line[n] = 0;
        text(lcd, 8, 72, &line, 2, YELLOW).await;

        const NAMES: [&[u8]; 13] = [
            b"WAIT", b"PASSIVE", b"FAIL", b"SKIP", b"MEAS", b"GUARD", b"VERIFY", b"RX",
            b"CAPTURED", b"NOCAPS", b"NEG", b"CONTRACT", b"HOLD",
        ];
        let name = NAMES.get(phase as usize).copied().unwrap_or(b"?");
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"PD " {
            line[n] = b;
            n += 1;
        }
        line[n..n + name.len()].copy_from_slice(name);
        n += name.len();
        line[n] = 0;
        text(lcd, 8, 40, &line, 3, WHITE).await;

        // Selected voltage/current: "SEL 5.0V 3.0A".
        let mut l = Line12::new();
        for &b in b"SEL " {
            l.push(&[b]);
        }
        if mv >= 1000 {
            // r142: mv is MILLIVOLTS - scale to volts first (5000 mV
            // printed as "500.0V" before). Line12 caps at 12 chars, so
            // whole volts + one-decimal amps fit: "SEL 5V 3.0A".
            l.num_u32(u32::from(mv) / 1000, 0);
            l.push(b"V ");
        } else {
            l.push(b"- ");
        }
        if ma >= 1000 {
            l.num_u32(u32::from(ma) / 100, 1);
            l.push(b"A");
        } else {
            l.push(b"-A");
        }
        l.pad();
        lcd.draw_text(8, 90, l.bytes(), 2, GREEN, BLACK).await;

        // Flags: "ACC:1 RDY:0 CON:1".
        let acc = u8::from(view.accept.load(core::sync::atomic::Ordering::Relaxed));
        let rdy = u8::from(view.psrdy.load(core::sync::atomic::Ordering::Relaxed));
        let con = u8::from(view.contract.load(core::sync::atomic::Ordering::Relaxed));
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"ACC:1 RDY:1 CON:1" {
            line[n] = b;
            n += 1;
        }
        line[4] = b'0' + acc;
        line[10] = b'0' + rdy;
        line[16] = b'0' + con;
        line[n] = 0;
        text(lcd, 8, 130, &line, 2, CYAN).await;

        // Diagnostics: "REV:3 ERR:0 PK:12".
        let rev = view.peer_rev.load(core::sync::atomic::Ordering::Relaxed);
        let pk = view.rx_packets.load(core::sync::atomic::Ordering::Relaxed);
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"REV:0 ERR:0 PK:0" {
            line[n] = b;
            n += 1;
        }
        line[4] = b'0' + (rev % 10);
        line[11] = b'0' + (err % 10) as u8;
        let mut pk_n = 0;
        let mut v = pk;
        while v > 0 && n + 1 < line.len() {
            line[n - pk_n] = b'0' + (v % 10) as u8;
            pk_n += 1;
            v /= 10;
        }
        if pk == 0 {
            line[n - 1] = b'0';
        }
        line[n] = 0;
        text(lcd, 8, 160, &line, 2, YELLOW).await;
    }
}

/// 12-char NUL-terminated line buffer for the meter rows.
pub struct Line12 {
    buf: [u8; 13],
    len: usize,
}

impl Line12 {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 13],
            len: 0,
        }
    }

    pub fn push(&mut self, s: &[u8]) {
        for &b in s {
            if self.len < 12 {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
    }

    pub fn sign(&mut self, neg: bool) {
        if neg {
            self.push(b"-");
        }
    }

    /// Append `v` with `dec` digits after the decimal point.
    pub fn num_u32(&mut self, v: u32, dec: usize) {
        let mut digits = [0u8; 10];
        let mut n = 0;
        let mut x = v;
        if x == 0 {
            digits[0] = b'0';
            n = 1;
        } else {
            while x > 0 && n < 10 {
                digits[n] = b'0' + (x % 10) as u8;
                x /= 10;
                n += 1;
            }
        }
        // digits[0] is the least-significant digit.
        if n <= dec {
            self.push(b"0");
            if dec > 0 {
                self.push(b".");
                for _ in 0..(dec - n) {
                    self.push(b"0");
                }
                for k in (0..n).rev() {
                    self.push(&[digits[k]]);
                }
            }
        } else {
            let mut k = n;
            while k > dec {
                k -= 1;
                self.push(&[digits[k]]);
            }
            if dec > 0 {
                self.push(b".");
                while k > 0 {
                    k -= 1;
                    self.push(&[digits[k]]);
                }
            }
        }
    }

    /// Space-pad to the full 12 chars so a redraw erases the previous
    /// (possibly longer) value.
    pub fn pad(&mut self) {
        while self.len < 12 {
            self.buf[self.len] = b' ';
            self.len += 1;
        }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
