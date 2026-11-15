//! Two-screen UI in the kws-x1 style: a POWER monitor (live V/A/W) and a
//! PD screen (the engine view from the vendored fusb302-pd session,
//! r113). LEFT/RIGHT (and BACK) switch screens; OK on the PD screen
//! signals the PD task. Rendering goes through the Rust LCD driver —
//! every string here must only contain glyphs from lcd.rs FONT_CHARS.

use crate::buttons;
use crate::lcd::Lcd;
use crate::pd_task::View;
use settings_core::calibration::{Calibration, Quantity, Step, Wizard};

const BLACK: u16 = 0x0000;
const WHITE: u16 = 0xFFFF;
const GRAY: u16 = 0x7BEF;
const GREEN: u16 = 0x07E0;
const YELLOW: u16 = 0xFFE0;
const CYAN: u16 = 0x07FF;
const MAGENTA: u16 = 0xF81F;
/// r221: the calibration refusal color (kws-x1's ORANGE).
const ORANGE: u16 = 0xF9E0;
/// r224: kws-x1's RULE — the dim structural color (header rule, page
/// dots, the idle protocol badge).
const RULE: u16 = 0x3186;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Screen {
    Power,
    /// r227: the graph screen (kws-x1's Graph tab — the per-second
    /// power/voltage/current history plot).
    Graph,
    /// r225: the lines screen (kws-x1's Lines tab with only the D+/D−
    /// cards — this tester's CC lines are not wired to the MCU).
    Lines,
    /// r226: the energy screen (kws-x1's Energy tab — the charge /
    /// energy / time totals, checkpointed to the journal).
    Energy,
    Pd,
    /// r189: the QC2.0 trigger screen (targets 5/9/12/20 V, OK = REQ).
    Qc,
    /// r196: the QC3.0 continuous screen (target in mV, 200 mV steps).
    Qc3,
    /// r198: the AFC screen (5 V / 9 V single-wire requests).
    Afc,
    /// r199: the ripple scope screen (kws-x1's Ripple tab).
    Ripple,
    /// r218: the settings screen (the stored settings; LR cycles the
    /// shown value, OK saves it to the external-flash store).
    Settings,
    /// r221: the calibration wizard (kws-x1's Calibration page as a
    /// screen of its own — zero current, then the current and voltage
    /// entries; OK walks the steps, leaving abandons the wizard).
    Cal,
}

/// The screen cycle order (LEFT/RIGHT walk it in both directions).
const SCREEN_CYCLE: [Screen; 11] = [
    Screen::Power,
    Screen::Graph,
    Screen::Lines,
    Screen::Energy,
    Screen::Pd,
    Screen::Qc,
    Screen::Qc3,
    Screen::Afc,
    Screen::Ripple,
    Screen::Settings,
    Screen::Cal,
];

/// r225: pub(crate) — the console 'x' command advances the screen
/// remotely (the screenshot workflow's navigation).
pub(crate) fn next_screen(screen: Screen, dir: i16) -> Screen {
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
    /// r221: the calibration screen's entry adjust — Left/Right walk
    /// the entered value toward the reference meter's reading (the
    /// held-button repeat comes from the main loop).
    CalAdjust(i16),
    /// r221: the calibration screen's OK — take the zero / confirm the
    /// entry / apply the finished calibration.
    CalOk,
    /// r226: the settings page's row cursor (Idle -> Row mode).
    SetRow(i16),
    /// r227: the graph's focused value (trace, or the time window).
    GraphTrace(i16),
    GraphWindow(i16),
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

/// r227: what LEFT/RIGHT change on the open graph (kws-x1's
/// GraphFocus; None = closed, LEFT/RIGHT navigate).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GraphFocus {
    Trace,
    Window,
}

/// r227: the graph's full scale is the peak plus 10% headroom, rounded
/// up to one of these steps (times a power of ten), so the top line is
/// a round number the trace stays below: 22 W becomes 25 W (kws-x1's
/// graph_scale).
const GRAPH_HEADROOM_PERCENT: u64 = 10;
const GRAPH_SCALE_STEPS: [u64; 10] = [10, 12, 15, 20, 25, 30, 40, 50, 60, 80];

fn graph_scale(peak: u32) -> u32 {
    let wanted = (u64::from(peak) * (100 + GRAPH_HEADROOM_PERCENT)).div_ceil(100);
    let mut decade = 1u64;
    loop {
        if let Some(step) = GRAPH_SCALE_STEPS
            .iter()
            .map(|step| step * decade)
            .find(|step| *step >= wanted)
        {
            return u32::try_from(step).unwrap_or(u32::MAX);
        }
        decade *= 10;
    }
}

/// r227: a full scale in milli-units as its shortest exact decimal,
/// e.g. 25000 is "25", 1500 is "1.5" and 250 is "0.25" (kws-x1's
/// scale_label). NUL-terminated; returns the length.
fn scale_label(buf: &mut [u8; 24], milli: u32) -> usize {
    let (v, dec) = if milli % 1000 == 0 {
        (milli / 1000, 0)
    } else if milli % 100 == 0 {
        (milli / 100, 1)
    } else {
        (milli / 10, 2)
    };
    let mut l = Line12::new();
    l.num_u32(v, dec);
    let n = l.len.min(buf.len() - 1);
    buf[..n].copy_from_slice(&l.buf[..n]);
    buf[n] = 0;
    n
}

/// r227: `content` wrapped in "< " / " >" when focused (kws-x1's
/// framed labels). NUL-terminated; returns the length.
fn framed(buf: &mut [u8; 24], content: &[u8], focused: bool) -> usize {
    let mut n = 0;
    if focused {
        for &b in b"< " {
            buf[n] = b;
            n += 1;
        }
    }
    for &b in content {
        if n < 23 {
            buf[n] = b;
            n += 1;
        }
    }
    if focused && n < 22 {
        buf[n] = b' ';
        buf[n + 1] = b'>';
        n += 2;
    }
    buf[n] = 0;
    n
}

/// r226: the settings page's three states (kws-x1's Row focus and
/// value editors): Idle (LEFT/RIGHT navigate; OK enters), Row (the
/// cursor picks the row; OK opens the editor), Edit (LEFT/RIGHT change
/// the row's value; OK saves; BACK steps back).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetMode {
    Idle,
    Row,
    Edit,
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
    /// r227: the graph screen's state (kws-x1 Input's graph fields —
    /// the trace/window are view prefs that persist; the focus closes
    /// on leaving, like the ripple's).
    pub graph_focus: Option<GraphFocus>,
    pub graph_trace: settings_core::power_history::GraphTrace,
    pub graph_window: settings_core::power_history::GraphWindow,
    /// r227: the graph body's identity (the latest second-average, the
    /// trace/window/focus) — a change redraws, so the ~600-fill plot
    /// runs at the 1 Hz sample rate, not every loop tick.
    graph_drawn: (
        Option<(u16, u16, u32)>,
        settings_core::power_history::GraphTrace,
        settings_core::power_history::GraphWindow,
        Option<GraphFocus>,
    ),
    /// r199: the ripple screen's state (kws-x1 Input's ripple_* fields,
    /// buttons.rs:964-968 — transient, never persisted).
    pub ripple_focus: Option<RippleFocus>,
    pub ripple_hold: bool,
    pub ripple_timebase: ripple_core::ripple::Timebase,
    pub ripple_scale: ripple_core::ripple::Scale,
    /// r218: the settings screen's selection — an index into
    /// `settings::DIM_CHOICES`, re-read from the applied value on
    /// screen entry (enter()). Pub: the main loop's commit arm reads it.
    pub set_sel: usize,
    /// r218: the settings body's cache (a change redraws). Pub: the
    /// commit arm forces a redraw after saving.
    pub set_drawn: i16,
    /// r226: the settings page's mode and row cursor, plus the edited
    /// energy-interval index (settings-core's ALL order).
    pub set_mode: SetMode,
    pub set_row: usize,
    pub set_energy: usize,
    /// r221: the calibration wizard (created fresh on the OK that
    /// enters it, from the applied calibration; nothing applies until
    /// the last OK). r226: `cal_active` gates the wizard — without it
    /// the screen is the landing (LEFT/RIGHT navigate, OK enters).
    pub cal_wizard: Wizard,
    pub cal_active: bool,
    /// r221: the calibration body's identity (step + entry + refusal —
    /// a change redraws the body) and the last-drawn live register pair
    /// (a change updates the zero box / NOW row in place).
    cal_ident: u32,
    cal_live: (u16, u16, bool),
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

/// r224: kws-x1's labelled_outline — a 1 px rounded outline in `color`
/// with `label` sitting ON the top edge, the edge cut away under it.
async fn card(lcd: &mut Lcd, x0: u16, y0: u16, x1: u16, y1: u16, label: &[u8], color: u16) {
    lcd.rounded_outline(x0, y0, x1, y1, 5, color).await;
    let lw = label.len() as u16 * 6;
    let lx = x0 + 8;
    lcd.fill_window(lx - 3, y0, lx + lw + 2, y0, BLACK).await;
    text(lcd, lx, y0 - 3, label, 1, color).await;
}

/// r224: a card's value — the number with 3 decimals and the unit glued
/// on (kws-x1's value_with_unit; the r222 V/A/W format). r227: padded
/// to `cells` characters, NOT Line12's 12 — the padded trailing spaces
/// are opaque black cells, and 12 chars overshoots a card's right
/// border (the flickering/partially-missing borders the user caught:
/// the value erased the border every redraw, the card restored it next
/// pass). `cells` must keep the drawn band inside the card.
async fn value_text(
    lcd: &mut Lcd,
    x: u16,
    y: u16,
    v: u32,
    scale: u8,
    unit: &[u8],
    color: u16,
    cells: u16,
) {
    let mut l = Line12::new();
    l.num_u32(v, 3);
    l.push(unit);
    while l.len < cells.min(12) as usize {
        l.push(b" ");
    }
    lcd.draw_text(x, y, l.bytes(), scale, color, BLACK).await;
}

/// r224: `v` with `dec` decimals appended at `n` (X.XXX style).
fn fixed(buf: &mut [u8], mut n: usize, v: u32, dec: usize) -> usize {
    let scale = [1u32, 10, 100, 1000][dec];
    n = crate::console::push_dec(buf, n, v / scale);
    buf[n] = b'.';
    n += 1;
    let mut div = scale / 10;
    let frac = v % scale;
    while div > 0 {
        buf[n] = b'0' + ((frac / div) % 10) as u8;
        n += 1;
        div /= 10;
    }
    n
}

/// r224: the protocol strip's badge and accent (kws-x1's
/// protocol_badge): the QC/AFC trigger supersedes the PD baseline while
/// its session is live (STATE Active/Stepping); a PD contract shows the
/// spec revision and the established voltage/current, session-less
/// phases degrade to the plain-USB badge.
fn protocol_badge() -> ([u8; 24], u16) {
    use core::sync::atomic::Ordering;
    let state = crate::qc_task::STATE.load(Ordering::Relaxed);
    if state == 3 || state == 5 {
        let mode = crate::qc_task::MODE.load(Ordering::Relaxed);
        let (name, mv): (&[u8], u16) = match mode {
            1 => (b"QC3 ", u32::from(crate::qc_task::M3_TARGET.load(Ordering::Relaxed)) as u16),
            2 => (b"AFC ", crate::qc_task::AFC_TARGET.load(Ordering::Relaxed)),
            _ => (
                b"QC2 ",
                crate::qc_task::TARGETS_MV
                    .get(crate::qc_task::TARGET.load(Ordering::Relaxed) as usize)
                    .copied()
                    .unwrap_or(5000),
            ),
        };
        let mut b = [0u8; 24];
        let mut n = 0;
        for &c in name {
            b[n] = c;
            n += 1;
        }
        n = fixed(&mut b, n, u32::from(mv), 3);
        b[n] = b'V';
        b[n + 1] = 0;
        let color = if mode == 2 { MAGENTA } else { YELLOW };
        return (b, color);
    }
    let view = &crate::pd_task::VIEW;
    let phase = view.phase.load(Ordering::Relaxed);
    let mut b = [0u8; 24];
    let mut n = 0;
    match phase {
        11 => {
            // Contracted: "PD <rev>.0 <mv>V <ma>A".
            for &c in b"PD " {
                b[n] = c;
                n += 1;
            }
            let rev = view.peer_rev.load(Ordering::Relaxed);
            if (1..=3).contains(&rev) {
                b[n] = b'0' + rev;
                n += 1;
                for &c in b".0 " {
                    b[n] = c;
                    n += 1;
                }
            } else {
                for &c in b"   " {
                    b[n] = c;
                    n += 1;
                }
            }
            n = fixed(&mut b, n, u32::from(view.selected_mv.load(Ordering::Relaxed)), 3);
            for &c in b"V " {
                b[n] = c;
                n += 1;
            }
            n = fixed(&mut b, n, u32::from(view.selected_ma.load(Ordering::Relaxed)), 3);
            for &c in b"A" {
                b[n] = c;
                n += 1;
            }
            b[n] = 0;
            (b, GREEN)
        }
        2 => {
            set(&mut b, b"PD FAIL");
            (b, ORANGE)
        }
        10 => {
            set(&mut b, b"PD NEG");
            (b, CYAN)
        }
        // Waiting/Passive/Skipped/NoCapabilities (and anything without a
        // live session): a plain-USB source.
        0 | 1 | 3 | 9 => {
            set(&mut b, b"USB");
            (b, RULE)
        }
        _ => {
            set(&mut b, b"PD");
            (b, RULE)
        }
    }
}

/// Copy `s` NUL-terminated into `b` (the caller returns the array by
/// value; draw_text stops at the NUL).
fn set(b: &mut [u8; 24], s: &[u8]) {
    let n = s.len().min(23);
    b[..n].copy_from_slice(&s[..n]);
    b[n] = 0;
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
            graph_focus: None,
            graph_trace: settings_core::power_history::GraphTrace::default(),
            graph_window: settings_core::power_history::GraphWindow::default(),
            graph_drawn: (
                Some((0, 0, 1)),
                settings_core::power_history::GraphTrace::default(),
                settings_core::power_history::GraphWindow::default(),
                None,
            ),
            ripple_focus: None,
            ripple_hold: false,
            ripple_timebase: ripple_core::ripple::Timebase::default(),
            ripple_scale: ripple_core::ripple::Scale::default(),
            ripple_at: 0,
            ripple_tag: 0xFFFF,
            set_sel: 0,
            set_drawn: -1,
            set_mode: SetMode::Idle,
            set_row: 0,
            set_energy: 0,
            cal_wizard: Wizard::new(Calibration::default()),
            cal_active: false,
            cal_ident: u32::MAX,
            cal_live: (0, 0, false),
        }
    }

    /// r172 navigation (user spec): LEFT/RIGHT move between screens; OK on
    /// the PPS screen ENTERS voltage selection; inside it LEFT/RIGHT choose
    /// the voltage, OK selects it (re-request), BACK returns to screen
    /// navigation. Only glyphs from lcd.rs FONT_CHARS (no '<'/'>'!).
    pub fn on_press(&mut self, i: usize) -> Press {
        // r226: the settings page is kws-x1's two-level machine: Idle
        // (LEFT/RIGHT navigate) -> OK -> Row (the cursor picks the row)
        // -> OK -> Edit (LEFT/RIGHT change the row's value, OK saves,
        // BACK steps back a level). The r222 OK-gate directive, grown
        // by the energy row.
        if self.screen == Screen::Settings {
            match self.set_mode {
                SetMode::Idle => {
                    if i == buttons::OK {
                        self.set_mode = SetMode::Row;
                        self.set_row = 0;
                        return Press::Hint;
                    }
                }
                SetMode::Row => match i {
                    buttons::LEFT => return Press::SetRow(-1),
                    buttons::RIGHT => return Press::SetRow(1),
                    buttons::OK => {
                        self.set_mode = SetMode::Edit;
                        return Press::Hint;
                    }
                    buttons::BACK => {
                        self.set_mode = SetMode::Idle;
                        return Press::Hint;
                    }
                    _ => {}
                },
                SetMode::Edit => match i {
                    buttons::LEFT => return Press::Voltage(-1),
                    buttons::RIGHT => return Press::Voltage(1),
                    buttons::OK => return Press::Commit,
                    buttons::BACK => {
                        self.set_mode = SetMode::Row;
                        return Press::Hint;
                    }
                    _ => {}
                },
            }
        }
        // r221: the calibration screen. r226 (user directive, the r222
        // pattern): LEFT/RIGHT always navigate unless the wizard is
        // RUNNING — OK enters the wizard (kws-x1's settings-row OK),
        // then LR adjusts, OK takes the zero/confirms/finishes, and
        // BACK cancels back to the landing.
        if self.screen == Screen::Cal {
            match i {
                buttons::LEFT | buttons::RIGHT if self.cal_active => {
                    return Press::CalAdjust(if i == buttons::RIGHT { 1 } else { -1 });
                }
                buttons::OK => {
                    if self.cal_active {
                        return Press::CalOk;
                    }
                    self.cal_active = true;
                    self.cal_wizard = Wizard::new(crate::settings::applied());
                    self.cal_ident = u32::MAX;
                    return Press::Hint;
                }
                buttons::BACK if self.cal_active => {
                    self.cal_active = false;
                    self.cal_ident = u32::MAX;
                    return Press::Hint;
                }
                _ => {}
            }
        }
        // r224: the overview's OK resets the session peaks (kws-x1's
        // long-OK reset — no long-press here, so a plain OK; the MAX
        // card sweeps back to the current sample).
        if self.screen == Screen::Power && i == buttons::OK {
            return Press::Commit;
        }
        // r227: the graph's focus model (kws-x1 buttons.rs:1677): OK
        // walks None -> Trace -> Window -> None; LR changes the focused
        // one; BACK closes the focus. Unfocused LR falls through to
        // screen navigation.
        if self.screen == Screen::Graph {
            match i {
                buttons::OK => {
                    self.graph_focus = match self.graph_focus {
                        None => Some(GraphFocus::Trace),
                        Some(GraphFocus::Trace) => Some(GraphFocus::Window),
                        Some(GraphFocus::Window) => None,
                    };
                    return Press::Hint;
                }
                buttons::BACK if self.graph_focus.is_some() => {
                    self.graph_focus = None;
                    return Press::Hint;
                }
                buttons::LEFT | buttons::RIGHT if self.graph_focus.is_some() => {
                    let dir = if i == buttons::RIGHT { 1 } else { -1 };
                    return match self.graph_focus {
                        Some(GraphFocus::Trace) => Press::GraphTrace(dir),
                        Some(GraphFocus::Window) => Press::GraphWindow(dir),
                        None => Press::None,
                    };
                }
                _ => {}
            }
        }
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

    /// r223: reset every draw cache so the next draw_* call repaints
    /// its whole body — the screenshot's forced repaint. None of
    /// enter()'s navigation/selection/wizard side effects: a 'shot'
    /// must never exit a selection mode or abandon the wizard.
    pub fn force_repaint(&mut self) {
        self.pd_phase = 0xFF;
        self.pd_mv = 0xFFFF;
        self.pd_ma = 0xFFFF;
        self.pd_err = 0xFFFF;
        self.pd_tgt = 0xFFFF;
        self.qc_state = 0xFF;
        self.qc_outcome = 0xFF;
        self.qc_failure = 0xFF;
        self.qc_tgt = 0xFFFF;
        self.qc_dyn_at = 0;
        self.qc3_dyn_at = 0;
        self.qc3_state = 0xFF;
        self.qc3_tgt = 0xFFFF;
        self.afc_state = 0xFF;
        self.afc_tgt = 0xFFFF;
        self.afc_dyn_at = 0;
        self.ripple_at = 0;
        self.ripple_tag = 0xFFFF;
        self.set_drawn = -1;
        self.cal_ident = u32::MAX;
        self.cal_live = (0, 0, false);
        // r227: the graph redraws its whole plot after a forced repaint.
        self.graph_drawn = (None, self.graph_trace, self.graph_window, self.graph_focus);
    }

    /// r223: the title, page tag and hint — enter()'s draw half without
    /// its state resets, so the screenshot's forced repaint re-draws
    /// the chrome without touching navigation/selection/wizard state.
    pub(crate) async fn draw_chrome(&mut self, lcd: &mut Lcd, screen: Screen) {
        match screen {
            Screen::Power => {
                // r224: kws-x1's header — the title in the tab's color,
                // the group-tab bars and page dots, a rule under all of
                // it. The bars/dots are decorative (fnb58's screens
                // aren't grouped; they carry the kws-x1 look).
                text(lcd, 8, 2, b"OVERVIEW", 3, CYAN).await;
                for (i, &c) in [YELLOW, GREEN, ORANGE, MAGENTA].iter().enumerate() {
                    let x = 180 + i as u16 * 20;
                    lcd.rounded_fill(x, 2, x + 14, 22, 6, c).await;
                }
                lcd.dot(290, 12, 5, CYAN).await;
                lcd.dot(302, 12, 5, RULE).await;
                lcd.dot(314, 12, 5, RULE).await;
                lcd.fill_window(4, 27, 315, 27, RULE).await;
                self.draw_hint(lcd).await;
            }
            Screen::Graph => {
                // r227: kws-x1's graph header (the accent + underline
                // bar over the rule, like the lines page's).
                text(lcd, 8, 2, b"GRAPH", 3, ORANGE).await;
                for (i, &c) in [YELLOW, GREEN, ORANGE, MAGENTA].iter().enumerate() {
                    let x = 180 + i as u16 * 20;
                    lcd.rounded_fill(x, 2, x + 14, 22, 6, c).await;
                }
                lcd.dot(290, 12, 5, RULE).await;
                lcd.dot(302, 12, 5, RULE).await;
                lcd.dot(314, 12, 5, RULE).await;
                lcd.fill_window(4, 27, 315, 27, RULE).await;
                lcd.fill_window(8, 25, 108, 27, ORANGE).await;
                self.draw_hint(lcd).await;
            }
            Screen::Lines => {
                // r225: kws-x1's lines header — the title in the screen's
                // accent with its underline bar over the rule; all page
                // dots dim (this page's picture).
                text(lcd, 8, 2, b"LINES", 3, YELLOW).await;
                for (i, &c) in [YELLOW, GREEN, ORANGE, MAGENTA].iter().enumerate() {
                    let x = 180 + i as u16 * 20;
                    lcd.rounded_fill(x, 2, x + 14, 22, 6, c).await;
                }
                lcd.dot(290, 12, 5, RULE).await;
                lcd.dot(302, 12, 5, RULE).await;
                lcd.dot(314, 12, 5, RULE).await;
                lcd.fill_window(4, 27, 315, 27, RULE).await;
                lcd.fill_window(8, 25, 100, 27, YELLOW).await;
                self.draw_hint(lcd).await;
            }
            Screen::Energy => {
                // r226: kws-x1's energy header (the accent + underline
                // bar over the rule, like the lines page's).
                text(lcd, 8, 2, b"ENERGY", 3, GREEN).await;
                for (i, &c) in [YELLOW, GREEN, ORANGE, MAGENTA].iter().enumerate() {
                    let x = 180 + i as u16 * 20;
                    lcd.rounded_fill(x, 2, x + 14, 22, 6, c).await;
                }
                lcd.dot(290, 12, 5, RULE).await;
                lcd.dot(302, 12, 5, RULE).await;
                lcd.dot(314, 12, 5, RULE).await;
                lcd.fill_window(4, 27, 315, 27, RULE).await;
                lcd.fill_window(8, 25, 116, 27, GREEN).await;
                self.draw_hint(lcd).await;
            }
            Screen::Pd => {
                text(lcd, 8, 8, b"PPS", 2, WHITE).await;
                text(lcd, 248, 8, b"5/11", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Qc => {
                text(lcd, 8, 8, b"QC", 2, WHITE).await;
                text(lcd, 248, 8, b"6/11", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Qc3 => {
                text(lcd, 8, 8, b"QC3", 2, WHITE).await;
                text(lcd, 248, 8, b"7/11", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Afc => {
                text(lcd, 8, 8, b"AFC", 2, WHITE).await;
                text(lcd, 248, 8, b"8/11", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Ripple => {
                text(lcd, 8, 8, b"RIPPLE", 2, WHITE).await;
                text(lcd, 248, 8, b"9/11", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Settings => {
                text(lcd, 8, 8, b"SET", 2, WHITE).await;
                text(lcd, 248, 8, b"10/11", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
            Screen::Cal => {
                text(lcd, 8, 8, b"CAL", 2, WHITE).await;
                text(lcd, 248, 8, b"11/11", 2, GRAY).await;
                self.draw_hint(lcd).await;
            }
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
        // r227: the graph's focus closes on leaving; the trace/window
        // view prefs persist (kws-x1's Input keeps them).
        if screen != Screen::Graph {
            self.graph_focus = None;
        }
        // r218: the settings screen starts at the APPLIED value.
        if screen == Screen::Settings {
            self.set_sel = crate::settings::sel_of(
                crate::settings::DIM_MINUTES.load(core::sync::atomic::Ordering::Relaxed),
            );
            self.set_energy =
                crate::settings::ENERGY_IDX.load(core::sync::atomic::Ordering::Relaxed);
            self.set_mode = SetMode::Idle;
            self.set_drawn = -1;
        }
        // r226: the calibration screen lands on the applied summary; the
        // wizard is created by the OK that enters it (kws-x1's
        // settings-row OK).
        self.screen = screen;
        self.selecting = false;
        self.cal_active = false;
        // Force the next screen-specific redraw.
        self.force_repaint();
        crate::ripple::LIVE.store(
            screen == Screen::Ripple && !self.ripple_hold,
            core::sync::atomic::Ordering::Relaxed,
        );
        lcd.fill(BLACK).await;
        self.draw_chrome(lcd, screen).await;
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
            // r224: the overview's gestures (LR navigates, OK resets the
            // session peaks — kws-x1's hold-OK reset without long-press).
            Screen::Power => b"< > SCRN OK:RST",
            // r226: the energy page's reset (the save note is drawn in
            // the body — it is dynamic).
            Screen::Energy => b"OK:RST",
            // r227: the graph's focus model (kws-x1's hints; no hold-OK
            // reset — the history is RAM-only and clears on reboot).
            Screen::Graph => match self.graph_focus {
                None => b"< > SCRN OK:SEL",
                Some(GraphFocus::Trace) => b"< > TRACE OK:TIME",
                Some(GraphFocus::Window) => b"< > TIME OK:DONE",
            },
            // r225: kws-x1's lines hint ("approximate" — the D± taps are
            // 1 kOhm-series ADC reads, not calibrated references).
            Screen::Lines => b"< > SCRN APPROX",
            // r226: the settings page's 3 modes.
            Screen::Settings => match self.set_mode {
                SetMode::Idle => b"OK:SET",
                SetMode::Row => b"LR:ROW OK:EDIT BK:X",
                SetMode::Edit => b"LR:ADJ OK:SAVE BK:X",
            },
            // r221: the calibration steps' gesture lines (kws-x1's
            // hints; the FNB58 font has no '<'/'>' so LR:ADJ stands in
            // for the angle brackets). r226: the landing's OK enters.
            Screen::Cal if !self.cal_active => b"OK:CAL",
            Screen::Cal => match self.cal_wizard.step {
                Step::Zero => b"OK:0 BK:X",
                Step::Entry {
                    quantity: Quantity::Current,
                    ..
                } => b"LR:ADJ OK:SET BK:X",
                Step::Entry {
                    quantity: Quantity::Voltage,
                    ..
                } => b"LR:ADJ OK:SAVE BK:X",
            },
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
                // y() maps LARGER counts to SMALLER pixel rows, so
                // y(bin.max) < y(bin.min): the window wants (top, bottom)
                // — reversed order underflows fill_window's height math
                // and painted the whole screen green once real (non-flat)
                // data arrived (the r208 hardware report).
                let low = y(bin.min);
                let high = y(bin.max);
                lcd.fill_window(
                    x as u16,
                    high as u16,
                    (x + 1) as u16,
                    low as u16,
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

    /// r218: the settings screen's body — the dim timeout value, cycled
    /// with LEFT/RIGHT (redrawn on change only; the save happens on OK
    /// and the screen re-reads the applied value on entry).
    /// r226: the settings page — two rows (the dim timeout and the
    /// energy checkpoint interval) with kws-x1's cursor model: Idle
    /// shows both flat, Row/Edit highlight the cursor row in yellow.
    pub async fn draw_settings(&mut self, lcd: &mut Lcd) {
        let ident = self.set_mode as i16 * 1000
            + self.set_row as i16 * 100
            + self.set_sel as i16 * 10
            + self.set_energy as i16;
        if ident == self.set_drawn {
            return;
        }
        self.set_drawn = ident;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        let dim_value: &[u8] = match crate::settings::DIM_CHOICES
            .get(self.set_sel)
            .copied()
            .unwrap_or(1)
        {
            0 => b"NEVER",
            1 => b"1 MIN",
            2 => b"2 MIN",
            5 => b"5 MIN",
            _ => b"10 MIN",
        };
        let energy_value: &[u8] = settings_core::EnergyMemory::ALL
            .get(self.set_energy)
            .map(|&m| m.label().as_bytes())
            .unwrap_or(b"OFF");
        let rows: [(u16, &[u8], &[u8]); 2] = [
            (56, b"DIM IDLE", dim_value),
            (116, b"ENERGY SAVE", energy_value),
        ];
        for (i, (y, label, value)) in rows.iter().enumerate() {
            let cursor = self.set_mode != SetMode::Idle && self.set_row == i;
            let color = if cursor { YELLOW } else { GRAY };
            text(lcd, 8, *y, label, 2, color).await;
            let mut v = [0u8; 24];
            let n = value.len().min(23);
            v[..n].copy_from_slice(&value[..n]);
            v[n] = 0;
            lcd.draw_text(210, *y, &v, 2, color, BLACK).await;
        }
    }

    /// r221: the calibration screen (kws-x1's draw_calibration): the
    /// step's title and what to do, the entry box (zero step: this
    /// meter's live current; entry steps: the entered value), the live
    /// reading under it and a refusal in orange. The body redraws on
    /// step/entry/refusal change; the zero box and the NOW row update
    /// in place on a fresh meter read (fixed-width Line12 overdraws —
    /// no fills at the 200 ms meter cadence, the r194 flicker class).
    /// r226: without the wizard running the screen is the LANDING —
    /// the applied calibration's three fields in a card, OK enters.
    pub async fn draw_cal(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        if !self.cal_active {
            if self.cal_ident != 0 {
                self.cal_ident = 0;
                lcd.fill_window(0, 32, 319, 210, BLACK).await;
                let cal = crate::settings::applied();
                card(lcd, 4, 34, 315, 134, b"APPLIED", CYAN).await;
                let rows: [(i32, &[u8]); 3] = [
                    (cal.zero_ua, b"0 CUR"),
                    (cal.current_ppm, b"CUR PPM"),
                    (cal.voltage_ppm, b"V PPM"),
                ];
                for (i, (v, label)) in rows.iter().enumerate() {
                    let y = 50 + i as u16 * 26;
                    let mut l = Line12::new();
                    l.push(*label);
                    l.pad();
                    lcd.draw_text(12, y, l.bytes(), 2, GRAY, BLACK).await;
                    let mut v2 = Line12::new();
                    v2.sign(*v < 0);
                    v2.num_u32(v.unsigned_abs(), 0);
                    // r227: the cell bound keeps the erase band inside the
                    // card (the pad-to-12 overshot its right border).
                    while v2.len < 9 {
                        v2.push(b" ");
                    }
                    lcd.draw_text(200, y, v2.bytes(), 2, WHITE, BLACK).await;
                }
                self.draw_hint(lcd).await;
            }
            return;
        }
        let bus = crate::qc_task::METER_BUS_RAW.load(Ordering::Relaxed);
        let cur = crate::qc_task::METER_CUR_RAW.load(Ordering::Relaxed);
        let fresh = crate::qc_task::METER_FRESH.load(Ordering::Relaxed);
        let wizard = self.cal_wizard;
        let entered = wizard.step.entered();
        // Identity fields: the step in 0..8, the entry value in 8..26
        // (a bus reading reaches 81919 mV — 17 bits), the refusal above
        // bit 26 (no aliasing: 81919 << 8 < 2^25 <= 1 << 26).
        let ident = match wizard.step {
            Step::Zero => 0u32,
            Step::Entry {
                quantity: Quantity::Current,
                ..
            } => 1u32,
            Step::Entry {
                quantity: Quantity::Voltage,
                ..
            } => 2u32,
        } | (entered.unwrap_or(0xFFFF)) << 8
            | (wizard.refusal.map(|r| r as u32 + 1).unwrap_or(0)) << 26;
        let live_changed = (bus, cur, fresh) != self.cal_live;
        let zero = wizard.step == Step::Zero;
        let voltage = matches!(
            wizard.step,
            Step::Entry {
                quantity: Quantity::Voltage,
                ..
            }
        );
        if ident != self.cal_ident {
            self.cal_ident = ident;
            self.cal_live = (bus, cur, fresh);
            lcd.fill_window(0, 32, 319, 210, BLACK).await;
            let (title, detail): (&[u8], &[u8]) = if zero {
                (b"1/3 0 CURRENT", b"UNPLUG LOAD KEEP PWR")
            } else if voltage {
                (b"3/3 VOLTAGE", b"ENTER METER VOLTS")
            } else {
                (b"2/3 CURRENT", b"STEADY LOAD: METER A")
            };
            text(lcd, 8, 48, title, 2, YELLOW).await;
            text(lcd, 8, 78, detail, 2, WHITE).await;
            text(lcd, 8, 112, if zero { b"THIS METER" } else { b"YOUR METER" }, 1, GRAY).await;
            if let Some(refusal) = wizard.refusal {
                text(lcd, 8, 192, refusal.message().as_bytes(), 2, ORANGE).await;
            }
            self.draw_hint(lcd).await;
            // The value box and (entry steps) the NOW row, at the
            // redraw's own live values.
            self.draw_cal_value(lcd, wizard, entered, bus, cur, fresh).await;
            if !zero {
                self.draw_cal_now(lcd, wizard, bus, cur, fresh).await;
            }
        } else if live_changed {
            self.cal_live = (bus, cur, fresh);
            // Zero step: the box IS the live reading; entry steps: the
            // NOW row moves instead (the box only follows the entry).
            if zero {
                self.draw_cal_value(lcd, wizard, entered, bus, cur, fresh).await;
            } else {
                self.draw_cal_now(lcd, wizard, bus, cur, fresh).await;
            }
        }
    }

    /// The calibration value box (y=126, scale 3): the zero step's live
    /// current or the entry's entered value, "X.XXX A/V"; "-.---" when
    /// nothing to show. Fixed-width overdraw (Line12::pad).
    async fn draw_cal_value(
        &self,
        lcd: &mut Lcd,
        wizard: Wizard,
        entered: Option<u32>,
        _bus: u16,
        cur: u16,
        fresh: bool,
    ) {
        let voltage = matches!(
            wizard.step,
            Step::Entry {
                quantity: Quantity::Voltage,
                ..
            }
        );
        let unit: &[u8] = if voltage { b" V" } else { b" A" };
        let mut line = Line12::new();
        if let Some(value) = entered {
            line.num_u32(value, 3);
            line.push(unit);
        } else if wizard.step == Step::Zero {
            // This meter's live current through the pending calibration.
            if fresh {
                line.num_u32(wizard.pending.milliamps(cur).unsigned_abs(), 3);
                line.push(b" A");
            } else {
                line.push(b"-.--- A");
            }
        } else {
            line.push(b"-.---");
            line.push(unit);
        }
        line.pad();
        lcd.draw_text(20, 126, line.bytes(), 3, YELLOW, BLACK).await;
    }

    /// The calibration screen's live row (y=180, scale 2): "NOW X.XXX
    /// A/V" — this meter's reading through the pending calibration, so
    /// the user can see the entry and the meter at once (kws-x1's
    /// "This meter now" line). Fixed-width overdraw.
    async fn draw_cal_now(&self, lcd: &mut Lcd, wizard: Wizard, bus: u16, cur: u16, fresh: bool) {
        let voltage = matches!(
            wizard.step,
            Step::Entry {
                quantity: Quantity::Voltage,
                ..
            }
        );
        let mut line = Line12::new();
        line.push(b"NOW ");
        if fresh {
            if voltage {
                line.num_u32(wizard.pending.millivolts(bus), 3);
                line.push(b" V");
            } else {
                line.num_u32(wizard.pending.milliamps(cur).unsigned_abs(), 3);
                line.push(b" A");
            }
        } else {
            line.push(b"-.---");
            line.push(if voltage { b" V" } else { b" A" });
        }
        line.pad();
        lcd.draw_text(20, 180, line.bytes(), 2, GRAY, BLACK).await;
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

    /// r227: the graph screen — kws-x1's draw_graph: the NOW/MAX card
    /// (the latest second and the window's highest, in the overview's
    /// colours), then the plot: full/half/zero rules, an auto-scaled
    /// full-scale label, the framed trace and window labels top right,
    /// and one column per second (or the highest second per pixel
    /// column when the window outgrows the plot) in the trace's colour
    /// over its tint.
    pub async fn draw_graph(
        &mut self,
        lcd: &mut Lcd,
        history: &settings_core::power_history::History,
    ) {
        use settings_core::power_history::{GraphTrace, GraphWindow};
        // Redraw only on a new second-average (or a view change) — the
        // ~600-fill plot at the 1 Hz sample rate, not every loop tick.
        let identity = (
            history.latest().map(|s| (s.mv, s.ma, s.mw)),
            self.graph_trace,
            self.graph_window,
            self.graph_focus,
        );
        if identity == self.graph_drawn {
            return;
        }
        self.graph_drawn = identity;
        let max = history.max(self.graph_window);
        // The NOW/MAX row (kws-x1's graph_row): "5.062/5.130V" pairs in
        // the overview's colours under a NOW/MAX label — NOW white,
        // /MAX magenta (the r222 3-decimal format).
        lcd.rounded_outline(4, 34, 315, 78, 5, WHITE).await;
        lcd.fill_window(12, 34, 12 + 7 * 6 + 2, 34, BLACK).await;
        text(lcd, 14, 31, b"NOW", 1, WHITE).await;
        text(lcd, 14 + 3 * 6, 31, b"/MAX", 1, MAGENTA).await;
        let pairs: [(Option<u32>, Option<u32>, &[u8], u16); 3] = [
            (
                history.latest().map(|s| u32::from(s.mv)),
                max.map(|s| u32::from(s.mv)),
                b"V",
                YELLOW,
            ),
            (
                history.latest().map(|s| u32::from(s.ma)),
                max.map(|s| u32::from(s.ma)),
                b"A",
                CYAN,
            ),
            (
                history.latest().map(|s| s.mw),
                max.map(|s| s.mw),
                b"W",
                GREEN,
            ),
        ];
        // Spread with equal gaps (kws-x1's widths/gap computation).
        let mut x = 10u16;
        for (now, mx, unit, color) in pairs {
            let mut l = Line12::new();
            for half in [now, mx] {
                match half {
                    Some(v) => l.num_u32(v, 3),
                    None => l.push(b"--.---"),
                }
            }
            l.push(unit);
            let w = l.len as u16 * 6;
            l.pad();
            lcd.draw_text(x, 52, l.bytes(), 1, color, BLACK).await;
            x += w + 12;
        }

        // The plot: y 84..204, x 4..315.
        let (top, bottom, left, right) = (84u16, 204u16, 4u16, 315u16);
        for y in [top, (top + bottom) / 2, bottom] {
            lcd.fill_window(left, y, right, y, RULE).await;
        }
        let (color, tint) = match self.graph_trace {
            GraphTrace::Power => (GREEN, 0x0941),    // kws-x1's WATTS_TINT
            GraphTrace::Voltage => (YELLOW, 0x2940), // VOLTS_TINT
            GraphTrace::Current => (CYAN, 0x0926),   // AMPS_TINT
        };
        let peak = max.map_or(0, |m| self.graph_trace.value(m));
        let scale = graph_scale(peak);
        // The scale label at the plot's top left, then the framed trace
        // and window labels top right — drawn before the trace so it
        // stays on top where they meet.
        let mut label = [0u8; 24];
        let mut n = scale_label(&mut label, scale);
        for &b in self.graph_trace.unit().as_bytes() {
            label[n] = b;
            n += 1;
        }
        label[n] = 0;
        text(lcd, left + 2, top + 4, &label, 1, GRAY).await;
        let mut span = [0u8; 24];
        let focused_window = self.graph_focus == Some(GraphFocus::Window);
        let focused_trace = self.graph_focus == Some(GraphFocus::Trace);
        let wl = framed(&mut span, self.graph_window.label().as_bytes(), focused_window);
        let wx = right - 2 - wl as u16 * 6;
        text(lcd, wx, top + 4, &span, 1, if focused_window { CYAN } else { GRAY }).await;
        let tl = framed(&mut span, self.graph_trace.label().as_bytes(), focused_trace);
        let tx = wx - 12 - tl as u16 * 6;
        text(lcd, tx, top + 4, &span, 1, if focused_trace { color } else { GRAY }).await;

        // One column per second, or the highest second per pixel column
        // when the window outgrows the plot (kws-x1's column fold).
        let seconds = self.graph_window.seconds();
        let columns = (seconds as usize).min((right - left + 1) as usize);
        let mut previous: Option<(u16, u16)> = None;
        let mut column: (usize, Option<u32>) = (0, None);
        for (second, sample) in history.recent(self.graph_window).enumerate() {
            let index = second * columns / seconds;
            if index != column.0 {
                self.plot_column(
                    lcd, column, columns, scale, top, bottom, left, right, color, tint,
                    &mut previous,
                )
                .await;
                column = (index, None);
            }
            column.1 = column.1.max(sample.map(|s| self.graph_trace.value(s)));
        }
        self.plot_column(
            lcd, column, columns, scale, top, bottom, left, right, color, tint, &mut previous,
        )
        .await;
    }

    /// Plot column `index` of `columns` at `value` (none: a gap) against
    /// `scale`, as a line in `color` over a `tint` fill, joined to the
    /// `previous` column's point (kws-x1's plot_column).
    async fn plot_column(
        &mut self,
        lcd: &mut Lcd,
        (index, value): (usize, Option<u32>),
        columns: usize,
        scale: u32,
        top: u16,
        bottom: u16,
        left: u16,
        right: u16,
        color: u16,
        tint: u16,
        previous: &mut Option<(u16, u16)>,
    ) {
        let Some(value) = value else {
            *previous = None;
            return;
        };
        let width = (right - left + 1) as usize;
        let lx = left + (width * index / columns) as u16;
        let rx = left + (width * (index + 1) / columns) as u16;
        let rise = (u64::from(value) * u64::from(bottom - top) / u64::from(scale)) as u16;
        let py = bottom - 1 - rise.min(bottom - top - 1);
        lcd.fill_window(lx, py, rx.max(lx + 1) - 1, bottom, tint).await;
        let point = ((lx + rx.max(lx + 1) - 1) / 2, py);
        let (px, py2) = previous.unwrap_or(point);
        lcd.line(px, py2, point.0, point.1, color).await;
        *previous = Some(point);
    }

    /// r226: the energy screen — kws-x1's draw_energy: the CHARGE /
    /// ENERGY / TIME totals in stacked cards. `save_ok` carries the
    /// journal's last checkpoint outcome (the note row below the cards;
    /// the interval comes from the applied setting).
    pub async fn draw_energy(
        &self,
        lcd: &mut Lcd,
        totals: settings_core::energy::Totals,
        save_ok: bool,
    ) {
        use core::sync::atomic::Ordering;
        let cards: [(&[u8], u64, &[u8], u16); 2] = [
            (b"CHARGE", totals.milliamp_hours(), b"Ah", CYAN),
            (b"ENERGY", totals.milliwatt_hours(), b"Wh", GREEN),
        ];
        for (i, (label, value, unit, color)) in cards.iter().enumerate() {
            let y0 = 34 + i as u16 * 62;
            card(lcd, 4, y0, 315, y0 + 56, label, *color).await;
            // The mAh (mWh) count IS the thousandths of an Ah (Wh) —
            // three decimals of it read directly in Ah (Wh), kws-x1's
            // fixed_u64(mAh, 3) trick.
            let mut l = Line12::new();
            l.num_u32(*value as u32, 3);
            l.push(unit);
            l.pad();
            lcd.draw_text(12, y0 + 10, l.bytes(), 3, *color, BLACK).await;
        }
        // TIME: hours grow past 99 (kws-x1's duration format).
        card(lcd, 4, 158, 315, 214, b"TIME", WHITE).await;
        let seconds = (totals.elapsed_ms / 1000) as u32;
        let mut l = Line12::new();
        l.num_u32(seconds / 3600, 0);
        l.push(b":");
        let two = |v: u32| [b'0' + (v / 10) as u8, b'0' + (v % 10) as u8];
        let mm = two(seconds / 60 % 60);
        let ss = two(seconds % 60);
        l.push(&mm);
        l.push(b":");
        l.push(&ss);
        l.pad();
        lcd.draw_text(12, 168, l.bytes(), 3, WHITE, BLACK).await;
        // The note: how the totals are kept (kws-x1's hint note).
        let idx = crate::settings::ENERGY_IDX.load(Ordering::Relaxed);
        let memory = settings_core::EnergyMemory::ALL
            .get(idx)
            .copied()
            .unwrap_or(settings_core::EnergyMemory::Off);
        let mut note = [0u8; 24];
        match (memory, save_ok) {
            (settings_core::EnergyMemory::Off, _) => set(&mut note, b"RAM ONLY"),
            (_, false) => set(&mut note, b"SAVE FAIL"),
            (_, true) => {
                let mut line = [0u8; 24];
                set(&mut line, b"CHECKPOINT ");
                let mut n = 11;
                for &b in memory.label().as_bytes() {
                    if n < 23 {
                        line[n] = b;
                        n += 1;
                    }
                }
                line[n] = 0;
                note = line;
            }
        }
        lcd.draw_text(12, 220 - 16, &note, 1, GRAY, BLACK).await;
    }

    /// r225: the lines screen (kws-x1's Lines tab with only the D+/D−

    /// r225: the lines screen — kws-x1's draw_lines with only the D+/D−
    /// cards (this tester's CC lines are not wired to the MCU, the user
    /// board fact). The taps are the QC task's 20 Hz reads, so the
    /// values are live; the 3-decimal V format (r222).
    pub async fn draw_lines(&self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        let dp = crate::qc_task::DP_MV.load(Ordering::Relaxed);
        let dm = crate::qc_task::DM_MV.load(Ordering::Relaxed);
        card(lcd, 4, 34, 156, 134, b"D+", YELLOW).await;
        value_text(lcd, 12, 44, u32::from(dp), 3, b"V", YELLOW, 7).await;
        card(lcd, 164, 34, 315, 134, b"D-", CYAN).await;
        value_text(lcd, 172, 44, u32::from(dm), 3, b"V", CYAN, 7).await;
    }

    /// r224: the overview screen — kws-x1's draw_meter ported onto the
    /// 320x240 panel: the VOLTAGE/CURRENT cards on top, the POWER and
    /// MAX (session peaks) cards below, the PROTOCOL strip at the
    /// bottom, all as rounded outlined cards with notched labels. The
    /// metrics keep the r222 three-decimal V/A/W format. Redraws fully
    /// at the meter cadence (idempotent overdraw, like the old rows).
    pub async fn draw_power(&self, lcd: &mut Lcd, mv: u32, ua: i32, mw: i32) {
        use core::sync::atomic::Ordering;
        // Row 1: the value cards (kws-x1's VOLTS_CARD / AMPS_CARD).
        card(lcd, 4, 34, 156, 82, b"VOLTAGE", YELLOW).await;
        value_text(lcd, 12, 46, mv, 3, b"V", YELLOW, 7).await;
        card(lcd, 164, 34, 315, 82, b"CURRENT", CYAN).await;
        // The meter works in either direction, so the current is shown
        // as a magnitude (kws-x1's note).
        value_text(lcd, 172, 46, ua.unsigned_abs() / 1000, 3, b"A", CYAN, 7).await;
        // Row 2: the power card and the session peaks.
        card(lcd, 4, 90, 240, 154, b"POWER", GREEN).await;
        value_text(lcd, 12, 104, mw.unsigned_abs(), 4, b"W", GREEN, 9).await;
        card(lcd, 248, 90, 315, 154, b"MAX", MAGENTA).await;
        let peaks = [
            (crate::qc_task::PEAK_MV.load(Ordering::Relaxed), b"V", YELLOW),
            (crate::qc_task::PEAK_UA.load(Ordering::Relaxed) / 1000, b"A", CYAN),
            (crate::qc_task::PEAK_MW.load(Ordering::Relaxed), b"W", GREEN),
        ];
        for (i, (v, unit, color)) in peaks.iter().enumerate() {
            // Right-aligned rows, clear of the label (kws-x1's PEAKS_*).
            // r227: erase the row band INSIDE the card first — the old
            // pad-to-12 pushed black cells past the right border.
            let y = 100 + i as u16 * 14;
            lcd.fill_window(252, y, 310, y + 6, BLACK).await;
            let mut l = Line12::new();
            l.num_u32(*v, 3);
            l.push(*unit);
            let w = l.len as u16 * 6;
            lcd.draw_text(310 - w, y, l.bytes(), 1, *color, BLACK).await;
        }
        // The protocol strip with its heartbeat square (kws-x1's
        // protocol badge + STRIP_DOT).
        let (badge, color) = protocol_badge();
        card(lcd, 4, 162, 315, 188, b"PROTOCOL", color).await;
        text(lcd, 12, 170, &badge, 1, color).await;
        lcd.rounded_fill(300, 170, 308, 178, 2, color).await;
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
