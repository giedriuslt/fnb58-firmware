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
    /// r239: the REC screen — the offline recorder (calibrated V/I
    /// samples into the external flash's free area; the interval is a
    /// settings row). OK opens an action menu (START / STOP / RESUME /
    /// CLEAR by state); CLEAR erases one sector per main-loop tick.
    Rec,
    Pd,
    /// r233: the PD SNIFF screen (kws-x1's PD Sniff — the messages the
    /// passthrough sniffer hears between the charger and the device).
    Sniff,
    /// r229: the PD PROFILES screen (kws-x1's draw_fixed — the
    /// advertised fixed PDOs as a selector; OK requests the previewed
    /// profile through the r173 PD-native re-request).
    Profiles,
    /// r238: the CABLE screen (kws-x1's Detail::Cable page): the
    /// e-marker's identity summary, raw VDOs, SVID modes, maker info and
    /// statuses, in sections browsed OK-gated; the browse mode's OK asks
    /// (kws-x1's long-OK → SOP' discovery at fixed 5 V).
    Cable,
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
const SCREEN_CYCLE: [Screen; 15] = [
    Screen::Power,
    Screen::Graph,
    Screen::Lines,
    Screen::Energy,
    Screen::Rec,
    Screen::Pd,
    Screen::Sniff,
    Screen::Profiles,
    Screen::Cable,
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
    /// r229: the profiles selector's previewed row (editing mode).
    Profile(i16),
    /// r235: the PPS page's focused value (volts, or amps).
    PpsVolts(i16),
    PpsAmps(i16),
    /// r238: the cable page's section flip (browsing mode).
    CablePage(i16),
    /// r238: the cable page's ask — the browse mode's OK (kws-x1's
    /// long-OK → the SOP' e-marker discovery).
    CableAsk,
    /// r239: the REC menu's cursor move (browsing the action list).
    RecSel(i16),
    /// r239: the REC menu's execute — the highlighted action (START /
    /// STOP / RESUME / CLEAR by state).
    RecRun(usize),
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

/// r229: the overview's palette per voltage level (kws-x1's
/// level_color) — the PD PROFILES and QC2 rows use it for the volts.
fn level_color(mv: u16) -> u16 {
    match mv {
        0..=5_500 => WHITE,
        5_501..=9_500 => CYAN,
        9_501..=12_500 => GREEN,
        12_501..=15_500 => YELLOW,
        _ => ORANGE,
    }
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

/// r235: what LEFT/RIGHT adjust on the PPS page (kws-x1's PpsFocus;
/// None = closed, LEFT/RIGHT navigate, OK opens).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PpsFocus {
    Volts,
    Amps,
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
    /// r235: the PPS page's state — the focus (None = closed), the set
    /// voltage/current, and the body's identity cache.
    pub pps_focus: Option<PpsFocus>,
    pub pps_mv: u16,
    pub pps_ma: u16,
    pps_drawn: (u16, u16, u16, usize, u16, u16, bool),
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
    /// r229: the profiles selector's state (kws-x1's editing flag and
    /// the previewed row) plus the body's identity cache.
    pub profiles_edit: bool,
    pub profiles_sel: usize,
    pub profiles_drawn: (bool, usize, usize, u16),
    /// r233: the sniff body's identity (the message count).
    sniff_drawn: u16,
    /// r238: the cable page — the browse mode (OK-gated sections, the
    /// r222 directive), the section on view, and the two draw gates (the
    /// static passthrough notice; the live state tuple).
    pub cable_browse: bool,
    pub cable_page: u16,
    cable_notice: bool,
    pub cable_ident: Option<(crate::pd_task::CableData, bool, u32, Option<u16>)>,
    /// r239: the REC screen — the action menu (OK-gated, the r222
    /// directive) and its cursor, plus the two draw gates (the static
    /// state line; the live counters redraw on change).
    pub rec_menu: bool,
    pub rec_sel: usize,
    pub rec_ident: (u8, u32, bool, usize, u16, u32),
    rec_notice: bool,
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
    pub set_drawn: i32,
    /// r226: the settings page's mode and row cursor, plus the edited
    /// energy-interval index (settings-core's ALL order) and the r233
    /// PD passthrough toggle.
    pub set_mode: SetMode,
    pub set_row: usize,
    pub set_energy: usize,
    pub set_passthrough: bool,
    /// r239: the REC-interval row's edited value (the REC_INTERVALS
    /// index), re-read from the applied value on entry.
    pub set_rec: usize,
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
            pps_focus: None,
            pps_mv: 5000,
            pps_ma: 1000,
            pps_drawn: (u16::MAX, 0, 0, 0, 0, 0, true),
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
            profiles_edit: false,
            profiles_sel: 0,
            profiles_drawn: (true, usize::MAX, 0, 0),
            sniff_drawn: u16::MAX,
            cable_browse: false,
            cable_page: 0,
            cable_notice: false,
            cable_ident: None,
            rec_menu: false,
            rec_sel: 0,
            rec_ident: (0xFF, 0, false, 0, 0, 0),
            rec_notice: false,
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
            set_rec: 0,
            set_mode: SetMode::Idle,
            set_row: 0,
            set_energy: 0,
            set_passthrough: false,
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
        // r235: the PPS page's focus machine (kws-x1's PpsFocus): OK
        // walks None -> volts -> amps -> request, LEFT/RIGHT adjust the
        // focused value (clamped to the advertised range), BACK closes.
        if self.screen == Screen::Pd {
            match i {
                buttons::OK => {
                    return match self.pps_focus {
                        None => {
                            self.pps_focus = Some(PpsFocus::Volts);
                            Press::Hint
                        }
                        Some(PpsFocus::Volts) => {
                            self.pps_focus = Some(PpsFocus::Amps);
                            Press::Hint
                        }
                        Some(PpsFocus::Amps) => Press::Commit,
                    };
                }
                buttons::BACK if self.pps_focus.is_some() => {
                    self.pps_focus = None;
                    return Press::Hint;
                }
                buttons::LEFT | buttons::RIGHT if self.pps_focus.is_some() => {
                    let dir = if i == buttons::RIGHT { 1 } else { -1 };
                    return match self.pps_focus {
                        Some(PpsFocus::Volts) => Press::PpsVolts(dir),
                        Some(PpsFocus::Amps) => Press::PpsAmps(dir),
                        None => Press::None,
                    };
                }
                _ => {}
            }
        }
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
        // r229: the profiles selector (kws-x1's draw_fixed flow): OK
        // enters, LEFT/RIGHT move the preview, OK requests the
        // previewed profile through the r173 PD-native re-request, BACK
        // cancels. Unfocused LEFT/RIGHT navigate.
        if self.screen == Screen::Profiles {
            match i {
                buttons::OK => {
                    if self.profiles_edit {
                        return Press::Commit;
                    }
                    self.profiles_edit = true;
                    // Seed the preview at the ACTIVE row (kws-x1 seeds
                    // the preview from the contract in force).
                    let n = crate::pd_task::CAPS_N.load(core::sync::atomic::Ordering::Relaxed);
                    let active = crate::pd_task::VIEW.selected_mv
                        .load(core::sync::atomic::Ordering::Relaxed);
                    self.profiles_sel = (0..n)
                        .find(|&i| {
                            crate::pd_task::CAPS_MV[i]
                                .load(core::sync::atomic::Ordering::Relaxed)
                                == active
                        })
                        .unwrap_or(0);
                    return Press::Hint;
                }
                buttons::BACK if self.profiles_edit => {
                    self.profiles_edit = false;
                    return Press::Hint;
                }
                buttons::LEFT | buttons::RIGHT if self.profiles_edit => {
                    return Press::Profile(if i == buttons::RIGHT { 1 } else { -1 });
                }
                _ => {}
            }
        }
        // r238: the cable page (kws-x1's Detail::Cable): OK enters the
        // section browser (the r222 OK-gate — LEFT/RIGHT navigate
        // screens unfocused), inside it LEFT/RIGHT flip sections, OK asks
        // for the SOP' read (kws-x1's long-OK; the browse-OK here), BACK
        // steps out.
        if self.screen == Screen::Cable {
            match i {
                buttons::OK => {
                    if self.cable_browse {
                        return Press::CableAsk;
                    }
                    self.cable_browse = true;
                    self.cable_page = 0;
                    // Section 0 (+1) is on view (the closure's
                    // scroll-driven MODES read keys on this marker).
                    crate::pd_task::CABLE_SECTION.store(1, core::sync::atomic::Ordering::Relaxed);
                    return Press::Hint;
                }
                buttons::BACK if self.cable_browse => {
                    self.cable_browse = false;
                    crate::pd_task::CABLE_SECTION.store(0, core::sync::atomic::Ordering::Relaxed);
                    return Press::Hint;
                }
                buttons::LEFT | buttons::RIGHT if self.cable_browse => {
                    return Press::CablePage(if i == buttons::RIGHT { 1 } else { -1 });
                }
                _ => {}
            }
        }
        // r239: the REC screen (kws-x1's OK-gated rows): OK opens the
        // action menu (START / STOP / RESUME / CLEAR by state), inside
        // it LEFT/RIGHT move the cursor, OK executes, BACK steps out.
        if self.screen == Screen::Rec {
            match i {
                buttons::OK => {
                    if self.rec_menu {
                        return Press::RecRun(self.rec_sel);
                    }
                    self.rec_menu = true;
                    self.rec_sel = 0;
                    return Press::Hint;
                }
                buttons::BACK if self.rec_menu => {
                    self.rec_menu = false;
                    return Press::Hint;
                }
                buttons::LEFT | buttons::RIGHT if self.rec_menu => {
                    return Press::RecSel(if i == buttons::RIGHT { 1 } else { -1 });
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
                    Screen::Qc | Screen::Qc3 | Screen::Afc
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

    /// r235: step the focused PPS editor (kws-x1's ±20 mV / ±50 mA per
    /// press, ±100 / ±250 held), clamped to the FIRST advertised range
    /// and the engine's own request rules (20 mV / 50 mA quanta, the
    /// 1 A PPS operating minimum — preview() would refuse anything
    /// else). The single-range clamp is a documented simplification:
    /// kws-x1 edits within the previewed range; this charger class
    /// advertises one.
    pub fn pps_step(&mut self, volts: bool, dir: i16, amount: i32) {
        use core::sync::atomic::Ordering;
        let n = crate::pd_task::PPS_N
            .load(Ordering::Relaxed)
            .min(crate::pd_task::PPS_OBJ.len());
        if n == 0 {
            return;
        }
        if volts {
            let min = crate::pd_task::PPS_MIN_MV[0].load(Ordering::Relaxed) as i32;
            let max = crate::pd_task::PPS_MAX_MV[0].load(Ordering::Relaxed) as i32;
            let next = (self.pps_mv as i32 + i32::from(dir) * amount).clamp(min, max);
            self.pps_mv = next as u16;
        } else {
            let max = crate::pd_task::PPS_MAX_MA[0].load(Ordering::Relaxed) as i32;
            let next = (self.pps_ma as i32 + i32::from(dir) * amount).clamp(1000, max);
            self.pps_ma = next as u16;
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
        // r235: the PPS page too.
        self.pps_drawn = (u16::MAX, 0, 0, 0, 0, 0, true);
        // r229/r233: the profiles and sniff bodies too (the notice and
        // the message list re-arm on a forced repaint).
        self.profiles_drawn = (true, usize::MAX, 0, 0);
        self.sniff_drawn = u16::MAX;
        // r238: the cable page's two gates too.
        self.cable_notice = false;
        self.cable_ident = None;
        // r239: the REC screen's gates too.
        self.rec_notice = false;
        self.rec_ident = (0xFF, 0, false, 0, 0, 0);
    }

    /// r230c: the per-screen header, uniform across all screens —
    /// kws-x1's style: the title in the tab's accent with its underline
    /// bar over the rule, the four group-tab pills and the page dots
    /// (decorative — fnb58's screens aren't grouped), then the screen's
    /// hint. The ripple title is plain magenta on black (user request —
    /// the r228 filled box read badly).
    pub(crate) async fn draw_chrome(&mut self, lcd: &mut Lcd, screen: Screen) {
        let (title, accent): (&[u8], u16) = match screen {
            Screen::Power => (b"OVERVIEW", CYAN),
            Screen::Graph => (b"GRAPH", ORANGE),
            Screen::Lines => (b"LINES", YELLOW),
            Screen::Energy => (b"ENERGY", GREEN),
            Screen::Rec => (b"REC", GREEN),
            Screen::Pd => (b"PPS", CYAN),
            Screen::Sniff => (b"PD SNIFF", GREEN),
            Screen::Profiles => (b"PD PROFILES", GREEN),
            Screen::Cable => (b"CABLE", GREEN),
            Screen::Qc => (b"QC 2.0", ORANGE),
            Screen::Qc3 => (b"QC 3.0", ORANGE),
            Screen::Afc => (b"AFC", CYAN),
            Screen::Ripple => (b"RIPPLE", MAGENTA),
            Screen::Settings => (b"SET", CYAN),
            Screen::Cal => (b"CAL", MAGENTA),
        };
        text(lcd, 8, 2, title, 3, accent).await;
        // The pills sit clear of the longest title ("PD PROFILES" is 11
        // chars = 198 px); the dots follow them.
        let (pill_x, dots): (u16, [u16; 3]) = if title.len() >= 10 {
            (212, [292, 303, 314])
        } else {
            (180, [290, 302, 314])
        };
        for (i, &c) in [YELLOW, GREEN, ORANGE, MAGENTA].iter().enumerate() {
            let x = pill_x + i as u16 * 20;
            lcd.rounded_fill(x, 2, x + 14, 22, 6, c).await;
        }
        for (i, &d) in dots.iter().enumerate() {
            lcd.dot(d, 12, 5, if i == 0 && screen == Screen::Power { CYAN } else { RULE })
                .await;
        }
        lcd.fill_window(4, 27, 315, 27, RULE).await;
        lcd.fill_window(8, 25, 8 + title.len() as u16 * 18 + 4, 27, accent).await;
        self.draw_hint(lcd).await;
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
        // r229: the profiles selector resets on leaving (kws-x1's
        // editing flag is page state).
        if screen != Screen::Profiles {
            self.profiles_edit = false;
        }
        // r238: the cable browse mode closes on leaving (the profiles
        // precedent), and the section-on-view marker goes with it (the
        // control closure's scroll-driven MODES read keys on it).
        if screen != Screen::Cable {
            self.cable_browse = false;
            crate::pd_task::CABLE_SECTION.store(0, core::sync::atomic::Ordering::Relaxed);
        }
        // r239: the REC action menu closes on leaving too.
        if screen != Screen::Rec {
            self.rec_menu = false;
        }
        // r235: the PPS page opens unfocused, seeded from the first
        // advertised range (min voltage / max current, floored at the
        // 1 A PPS operating minimum).
        if screen == Screen::Pd {
            self.pps_focus = None;
            let n = crate::pd_task::PPS_N
                .load(core::sync::atomic::Ordering::Relaxed)
                .min(crate::pd_task::PPS_OBJ.len());
            if n > 0 {
                self.pps_mv = crate::pd_task::PPS_MIN_MV[0]
                    .load(core::sync::atomic::Ordering::Relaxed);
                self.pps_ma = crate::pd_task::PPS_MAX_MA[0]
                    .load(core::sync::atomic::Ordering::Relaxed)
                    .max(1000);
            }
        }
        // r218: the settings screen starts at the APPLIED value.
        if screen == Screen::Settings {
            self.set_sel = crate::settings::sel_of(
                crate::settings::DIM_MINUTES.load(core::sync::atomic::Ordering::Relaxed),
            );
            self.set_energy =
                crate::settings::ENERGY_IDX.load(core::sync::atomic::Ordering::Relaxed);
            self.set_passthrough =
                crate::settings::PASSTHROUGH.load(core::sync::atomic::Ordering::Relaxed);
            self.set_rec =
                crate::settings::REC_IDX.load(core::sync::atomic::Ordering::Relaxed);
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
            // r239: the REC screen's action menu.
            Screen::Rec if self.rec_menu => b"LR:SEL OK:RUN BK:X",
            Screen::Rec => b"OK:MENU",
            // r229: the profiles selector's two states (kws-x1's
            // "< > choose OK request BACK cancel").
            Screen::Profiles if self.profiles_edit => b"LR:SEL OK:REQ BK:X",
            Screen::Profiles => b"OK:SEL",
            // r238: the cable page's browse mode (kws-x1's long-OK ask
            // is the browse OK — fnb58 has no long-press, the r226
            // energy-reset precedent).
            Screen::Cable if self.cable_browse => b"LR:PAGE OK:ASK BK:X",
            Screen::Cable => b"OK:SECTIONS",
            // r233: the sniff screen's scroll hint.
            Screen::Sniff => b"OK:SCROLL",
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
            Screen::Pd => match self.pps_focus {
                None => b"OK:PPS",
                Some(PpsFocus::Volts) => b"LR:VOLT OK:AMPS BK:X",
                Some(PpsFocus::Amps) => b"LR:AMPS OK:REQ BK:X",
            },
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
    /// r230: the QC2 screen — kws-x1's draw_qc2 laid out as the fixed PD
    /// profiles: one row per target, the pending request framed in the
    /// accent, ACTIVE badging the target the charger is holding, and the
    /// failure/follow verdict on a status row. The DP/DM/VBUS numbers
    /// moved to the lines and ripple screens (r225/r228).
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
        // r230b: the identity compares the target VALUE, not the index —
        // index-vs-value was never equal, so the whole body (a full
        // black fill + every row) redrew at the 100 Hz tick rate and the
        // screen flickered (the r189 code compared value-vs-value).
        let statics_changed = state != self.qc_state
            || outcome != self.qc_outcome
            || failure != self.qc_failure
            || tgt != self.qc_tgt;
        if !statics_changed {
            return;
        }
        self.qc_state = state;
        self.qc_outcome = outcome;
        self.qc_failure = failure;
        self.qc_tgt = tgt;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        for (i, &mv) in crate::qc_task::TARGETS_MV.iter().enumerate() {
            let y = 40 + i as u16 * 34;
            let selected = tgt_idx == i;
            if selected {
                // The pending request framed in the accent (kws-x1's
                // choice frame).
                lcd.rounded_outline(4, y - 6, 315, y + 22, 5, ORANGE).await;
            }
            let mut l = Line12::new();
            // kws-x1's row label: whole volts ("5V", "20V").
            l.num_u32(u32::from(mv) / 1000, 0);
            l.push(b"V");
            l.pad();
            lcd.draw_text(12, y, l.bytes(), 2, level_color(mv), BLACK).await;
            if state == 3 && selected {
                // ACTIVE: the charger is holding this target.
                lcd.rounded_outline(258, y - 2, 312, y + 16, 5, GREEN).await;
                text(lcd, 265, y + 2, b"ACTIVE", 1, GREEN).await;
            }
        }
        // The status row: the failure reason, or the follow verdict
        // (kws-x1's per-row status, kept as one row here).
        const STATES: [&[u8]; 6] = [
            b"OFF", b"RELEASE", b"ENTRY", b"ACTIVE", b"FAIL", b"STEP",
        ];
        let name = STATES.get(state as usize).copied().unwrap_or(b"?");
        let detail: &[u8] = match (state, failure, outcome) {
            (4, 1, _) => b"FAIL NODCP",
            (4, 2, _) => b"FAIL NOENTRY",
            (3, _, 1) => b"OUTPUT WAIT",
            (3, _, 2) => b"OUTPUT FOLLOWED",
            (3, _, 3) => b"OUTPUT IGNORED",
            _ => b"",
        };
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"QC " {
            line[n] = b;
            n += 1;
        }
        line[n..n + name.len()].copy_from_slice(name);
        n += name.len();
        if !detail.is_empty() {
            line[n] = b' ';
            n += 1;
            line[n..n + detail.len()].copy_from_slice(detail);
            n += detail.len();
        }
        line[n] = 0;
        text(
            lcd,
            8,
            184,
            &line,
            2,
            if state == 4 { ORANGE } else { CYAN },
        )
        .await;
    }

    /// r196: the QC3 screen — target (mV), state, the crate's tracked
    /// continuous set point, and the taps/output. Same split as draw_qc:
    /// the static rows (fill + TGT + state) redraw only on state/target
    /// changes; the fast numbers update in place at 200 ms.
    /// r230: the QC3 screen — kws-x1's card layout: the STEPS card (the
    /// continuous range), the SET VOLTAGE and VBUS cards side by side
    /// (the tracked set point and the live calibrated bus, big in the
    /// trace's colour), and the LINES card with the D± taps and the
    /// ACTIVE badge (or the failure, in orange). Redraws on state or
    /// set-point change; the VBUS card rides the meter's 200 ms values.
    pub async fn draw_qc3(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        let state = crate::qc_task::STATE.load(Ordering::Relaxed);
        let failure = crate::qc_task::FAILURE.load(Ordering::Relaxed);
        let set = crate::qc_task::SET_MV.load(Ordering::Relaxed);
        let statics_changed = state != self.qc3_state
            || set != self.qc3_tgt
            || self.qc3_dyn_at != 1;
        self.qc3_state = state;
        self.qc3_tgt = set;
        self.qc3_dyn_at = 1;
        if !statics_changed {
            return;
        }
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        // The STEPS card: the continuous range (qc3's own constants).
        card(lcd, 4, 34, 315, 66, b"STEPS", ORANGE).await;
        let mut steps = [0u8; 24];
        let mut n = 0;
        // "3.6-12V BY 0.2V" from the crate's milli-volt constants.
        n = fixed(&mut steps, n, u32::from(crate::qc_task::M3_MIN_MV), 3);
        steps[n] = b'-';
        n += 1;
        n = fixed(&mut steps, n, u32::from(crate::qc_task::M3_MAX_MV), 3);
        for &b in b"V BY " {
            steps[n] = b;
            n += 1;
        }
        n = fixed(&mut steps, n, u32::from(qc_protocol::qc3::STEP_MV), 3);
        for &b in b"V" {
            steps[n] = b;
            n += 1;
        }
        steps[n] = 0;
        text(lcd, 12, 42, &steps, 2, WHITE).await;

        // The SET VOLTAGE and VBUS cards side by side.
        card(lcd, 4, 72, 156, 146, b"SET VOLTAGE", YELLOW).await;
        card(lcd, 164, 72, 315, 146, b"VBUS", YELLOW).await;
        let mut l = Line12::new();
        if set > 0 {
            l.num_u32(u32::from(set), 3);
        } else {
            l.num_u32(u32::from(crate::qc_task::M3_TARGET.load(Ordering::Relaxed)), 3);
        }
        l.push(b"V");
        l.pad();
        lcd.draw_text(12, 86, l.bytes(), 3, YELLOW, BLACK).await;
        let cal = crate::settings::applied();
        let bus_raw = crate::qc_task::METER_BUS_RAW.load(Ordering::Relaxed);
        let bus_fresh = crate::qc_task::METER_FRESH.load(Ordering::Relaxed);
        let mut v = Line12::new();
        if bus_fresh {
            v.num_u32(cal.millivolts(bus_raw), 3);
        } else {
            v.push(b"--.---");
        }
        v.push(b"V");
        v.pad();
        lcd.draw_text(172, 86, v.bytes(), 3, YELLOW, BLACK).await;

        // The LINES card: the D± taps left, ACTIVE or the failure right.
        card(lcd, 4, 152, 315, 190, b"LINES", GREEN).await;
        // Two fixed cells — the joined line is 19 chars and Line12 caps
        // at 12 (the D- value clipped, caught by the screenshot).
        let dp = crate::qc_task::DP_MV.load(Ordering::Relaxed);
        let dm = crate::qc_task::DM_MV.load(Ordering::Relaxed);
        let mut ldp = Line12::new();
        ldp.push(b"D+ ");
        ldp.num_u32(u32::from(dp), 3);
        ldp.push(b"V");
        ldp.pad();
        lcd.draw_text(12, 162, ldp.bytes(), 2, WHITE, BLACK).await;
        let mut ldm = Line12::new();
        ldm.push(b"D- ");
        ldm.num_u32(u32::from(dm), 3);
        ldm.push(b"V");
        ldm.pad();
        lcd.draw_text(150, 162, ldm.bytes(), 2, WHITE, BLACK).await;
        let (badge, color): (&[u8], u16) = match (state as u32, failure as u32) {
            (4, 1) => (b"FAIL NODCP", ORANGE),
            (4, 2) => (b"FAIL NOENTRY", ORANGE),
            (3, _) | (5, _) => (b"ACTIVE", GREEN),
            _ => (b"", GRAY),
        };
        if !badge.is_empty() {
            text(lcd, 313 - badge.len() as u16 * 6, 164, badge, 2, color).await;
        }
    }


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
    /// r228: the ripple screen — kws-x1's draw_ripple layout: the big
    /// p-p value with the RMS at right, the gridded plot with the
    /// spectrum bars behind the yellow envelope trace (peak frequencies
    /// listed top right, digits at the bar feet), the live bus + scale
    /// row, then run/hold + status + the nominal timebase. Redraws on a
    /// fresh capture or a chrome change (`fresh` = the watch's
    /// try_changed — the ~500-fill plot must not run per loop tick).
    pub async fn draw_ripple(
        &mut self,
        lcd: &mut Lcd,
        reading: Option<&ripple_core::ripple::Reading>,
        reference: Option<u16>,
        fresh: bool,
    ) {
        use ripple_core::ripple::{
            FULL_SCALE, HORIZONTAL_DIVISIONS, Scale, TRACE_COLUMNS, Timebase, VERTICAL_DIVISIONS,
        };
        use ripple_core::spectrum::{self, FULL_HEIGHT};

        const SPECTRUM: u16 = 0x200A;
        const SPECTRUM_LABEL: u16 = 0x60D4;
        // Pixels: metrics above the full-width plot, live bus + scale
        // below it, then run/hold + status and the timebase (kws-x1's
        // rows, widened for 320 px).
        const PLOT_TOP: u16 = 60;
        const PLOT_BOTTOM: u16 = 184;
        const PLOT_LEFT: u16 = 4;
        const PLOT_RIGHT: u16 = 315;
        const BUS_Y: u16 = 188;
        const STATUS_Y: u16 = 204;

        // Redraw on a fresh capture or a chrome change.
        let scale_index = Scale::ALL
            .iter()
            .position(|&s| s == self.ripple_scale)
            .unwrap_or(0);
        let tb_index = Timebase::ALL
            .iter()
            .position(|&t| t == self.ripple_timebase)
            .unwrap_or(0);
        let tag = (match self.ripple_focus {
            None => 0u16,
            Some(RippleFocus::Time) => 1,
            Some(RippleFocus::Scale) => 2,
            Some(RippleFocus::Capture) => 3,
        } | (u16::from(self.ripple_hold) << 2)
            | ((scale_index as u16) << 3)
            | ((tb_index as u16) << 7));
        if !fresh && tag == self.ripple_tag {
            return;
        }
        self.ripple_tag = tag;

        let reference_mv = crate::ripple::REFERENCE_MV;
        let reference = reference.filter(|&count| count > 0 && count < FULL_SCALE);
        let normalized = reading.zip(reference);
        let mv = normalized.and_then(|(r, reference)| r.peak_to_peak_mv(reference, reference_mv));
        let rms = normalized.and_then(|(r, reference)| r.rms_tenths_mv(reference, reference_mv));

        // The p-p value, big in the trace's colour (kws-x1's BOLD row).
        let mut line = [0u8; 24];
        let mut n = match mv {
            Some(mv) => crate::push_dec(&mut line, 0, u32::from(mv)),
            None => {
                line[..2].copy_from_slice(b"--");
                2
            }
        };
        for &b in b" mV p-p" {
            line[n] = b;
            n += 1;
        }
        line[n] = 0;
        text(lcd, 8, 34, &line, 3, YELLOW).await;

        // RMS right-aligned on the same row (kws-x1's tenths format).
        let mut line = [0u8; 24];
        let mut n = 0;
        for &b in b"RMS " {
            line[n] = b;
            n += 1;
        }
        match rms {
            Some(tenths) => {
                n = crate::push_dec(&mut line, n, u32::from(tenths / 10));
                line[n] = b'.';
                line[n + 1] = b'0' + (tenths % 10) as u8;
                n += 2;
            }
            None => {
                line[n] = b'-';
                line[n + 1] = b'-';
                n += 2;
            }
        }
        for &b in b"mV" {
            line[n] = b;
            n += 1;
        }
        line[n] = 0;
        text(lcd, PLOT_RIGHT - 2 - n as u16 * 6, 40, &line, 1, WHITE).await;

        // The plot: cleared per capture (the panel has no framebuffer —
        // without the clear each capture's trace draws over the last
        // one, oscilloscope-persistence style), then grid, spectrum
        // bars behind, the envelope trace over.
        lcd.fill_window(PLOT_LEFT, PLOT_TOP, PLOT_RIGHT, PLOT_BOTTOM, BLACK).await;
        let width = (PLOT_RIGHT - PLOT_LEFT + 1) as u32;
        let plot = reading
            .map(|r| (r, r.plot_scale(self.ripple_scale, reference, reference_mv)));
        if let Some((reading, plot)) = plot {
            let y = |count: u16| {
                let count = i32::from(count).clamp(plot.min, plot.max);
                (i32::from(PLOT_BOTTOM)
                    - (count - plot.min)
                        * (i32::from(PLOT_BOTTOM) - i32::from(PLOT_TOP))
                        / (plot.max - plot.min)) as u16
            };
            for division in 0..=VERTICAL_DIVISIONS {
                let division = division as u16;
                let row =
                    PLOT_TOP + division * (PLOT_BOTTOM - PLOT_TOP) / VERTICAL_DIVISIONS as u16;
                lcd.fill_window(PLOT_LEFT, row, PLOT_RIGHT, row, RULE).await;
            }
            for division in 0..=HORIZONTAL_DIVISIONS {
                let division = division as u16;
                let x =
                    PLOT_LEFT + division * (PLOT_RIGHT - PLOT_LEFT) / HORIZONTAL_DIVISIONS as u16;
                lcd.fill_window(x, PLOT_TOP, x, PLOT_BOTTOM, RULE).await;
            }
            // The spectrum behind the trace: heights relative to the
            // strongest bin, one bar per column scaled to the plot.
            let bars = reading.spectrum.bars.len() as u32;
            for (column, &height) in reading.spectrum.bars.iter().enumerate() {
                let bar = u16::from(height) * (PLOT_BOTTOM - PLOT_TOP) / u16::from(FULL_HEIGHT);
                if bar > 0 {
                    let x = PLOT_LEFT + (column as u32 * width / bars) as u16;
                    lcd.fill_window(x, PLOT_BOTTOM - bar, x, PLOT_BOTTOM, SPECTRUM).await;
                }
            }
            // Peak frequencies at the plot's top right (tenths of a kHz
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
                    line[n] = 0;
                    text(
                        lcd,
                        PLOT_RIGHT - 2 - n as u16 * 6,
                        PLOT_TOP + 2,
                        &line,
                        1,
                        SPECTRUM_LABEL,
                    )
                    .await;
                }
            }
            // The envelope trace: one vertical span per bin (r208: the
            // y() mapping is inverted — the span wants top, bottom
            // ordered), joined centre-to-centre (r227's line()).
            let mut previous: Option<(u16, u16)> = None;
            for (column, bin) in reading.trace.iter().enumerate() {
                let x = PLOT_LEFT + (column as u32 * width / TRACE_COLUMNS as u32) as u16;
                let low = y(bin.min);
                let high = y(bin.max);
                let (a, b) = if low <= high { (low, high) } else { (high, low) };
                lcd.fill_window(x, a, x, b, YELLOW).await;
                let centre = (x, (a + b) / 2);
                if let Some((px, py)) = previous {
                    lcd.line(px, py, centre.0, centre.1, YELLOW).await;
                }
                previous = Some(centre);
            }
            // Peak digits at the bar feet, beside rather than over the
            // bar, over the trace so they stay legible (kws-x1).
            for (index, position) in reading.spectrum.peaks.iter().enumerate() {
                let Some(position) = position else { break };
                let digit = [b'1' + index as u8, 0];
                let x = PLOT_LEFT
                    + ((spectrum::column(*position) as u32) * width / bars) as u16;
                let dx = if x + 8 < PLOT_RIGHT { x + 2 } else { x - 7 };
                text(lcd, dx, PLOT_BOTTOM - 9, &digit, 1, SPECTRUM_LABEL).await;
            }
        } else {
            let label = b"NO CAPTURE";
            text(
                lcd,
                PLOT_LEFT + width as u16 / 2 - 6 * label.len() as u16 / 2,
                PLOT_TOP + (PLOT_BOTTOM - PLOT_TOP) / 2 - 7,
                label,
                2,
                GRAY,
            )
            .await;
        }

        // The bus row: the live calibrated output left, the scale right
        // (framed when focused — kws-x1's setting()).
        let cal = crate::settings::applied();
        let bus_raw = crate::qc_task::METER_BUS_RAW.load(core::sync::atomic::Ordering::Relaxed);
        let cur_raw = crate::qc_task::METER_CUR_RAW.load(core::sync::atomic::Ordering::Relaxed);
        let bus_fresh = crate::qc_task::METER_FRESH.load(core::sync::atomic::Ordering::Relaxed);
        // Two fixed cells — "0.064V 0.000A" is 13 chars and Line12 caps
        // at 12 (the A unit clipped, caught by the first screenshot).
        let mut lv = Line12::new();
        if bus_fresh {
            lv.num_u32(cal.millivolts(bus_raw), 3);
            lv.push(b"V");
        } else {
            lv.push(b"--.---V");
        }
        lv.pad();
        lcd.draw_text(8, BUS_Y, lv.bytes(), 2, WHITE, BLACK).await;
        let mut la = Line12::new();
        if bus_fresh {
            let ua = cal.milliamps(cur_raw);
            la.sign(ua < 0);
            la.num_u32(ua.unsigned_abs(), 3);
            la.push(b"A");
        } else {
            la.push(b"--.---A");
        }
        la.pad();
        lcd.draw_text(104, BUS_Y, la.bytes(), 2, WHITE, BLACK).await;
        let mut scratch = [0u8; 20];
        let mut n = 0;
        let auto = self.ripple_scale == Scale::Auto;
        match (
            plot.as_ref().and_then(|(_, p)| p.mv_per_div),
            reading.is_none(),
        ) {
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
        let focused = self.ripple_focus == Some(RippleFocus::Scale);
        let mut span = [0u8; 24];
        let sl = framed(&mut span, &scratch[..n], focused);
        text(
            lcd,
            PLOT_RIGHT - 2 - sl as u16 * 6,
            BUS_Y + 6,
            &span,
            1,
            if focused { YELLOW } else { GRAY },
        )
        .await;

        // Run/hold (framed when focused), the capture status beside it,
        // the nominal timebase right — the r203 fixed-cell rule for the
        // status (its length varies).
        let hold_label: &[u8] = if self.ripple_hold { b"HOLD" } else { b"RUN" };
        let focused = self.ripple_focus == Some(RippleFocus::Capture);
        let mut span = [0u8; 24];
        let hl = framed(&mut span, hold_label, focused);
        text(
            lcd,
            8,
            STATUS_Y,
            &span,
            1,
            if focused {
                YELLOW
            } else if self.ripple_hold {
                WHITE
            } else {
                GRAY
            },
        )
        .await;
        let clipped = reading.is_some_and(|r| r.clipped());
        let limited = plot.as_ref().map_or(false, |(_, p)| p.limited);
        let status: &[u8] = match reading {
            None => b"NO CAPTURE",
            Some(_) if clipped => b"CLIPPED",
            Some(_) if mv.is_none() => b"NO REF",
            Some(_) if limited => b"OFF SCALE",
            Some(r) if r.triggered => b"T",
            Some(_) => b" ",
        };
        let mut cell = [b' '; 11];
        cell[..status.len()].copy_from_slice(status);
        text(
            lcd,
            8 + hl as u16 * 6 + 12,
            STATUS_Y,
            &cell,
            1,
            if clipped || limited { ORANGE } else { GRAY },
        )
        .await;
        let timebase = reading.map_or(self.ripple_timebase, |r| r.timebase);
        let timebase_ok = timebase.sampling(crate::ripple::ADC_CLOCK_HZ).is_some();
        let mut scratch = [0u8; 12];
        let mut n = 0;
        if timebase_ok {
            scratch[n] = b'~';
            n += 1;
            n = crate::push_dec(&mut scratch, n, u32::from(timebase.us_per_div()));
        } else {
            scratch[n] = b'-';
            scratch[n + 1] = b'-';
            n += 2;
        }
        for &b in b"us/div" {
            scratch[n] = b;
            n += 1;
        }
        let focused = self.ripple_focus == Some(RippleFocus::Time);
        let tl = framed(&mut span, &scratch[..n], focused);
        text(
            lcd,
            PLOT_RIGHT - 2 - tl as u16 * 6,
            STATUS_Y,
            &span,
            1,
            if focused { YELLOW } else { GRAY },
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
        // r239: the REC row joins the identity (the old i16 packing
        // had no room for a 4th field; i32 with wide spacing).
        let ident = self.set_mode as i32 * 1_000_000
            + self.set_row as i32 * 100_000
            + self.set_sel as i32 * 10_000
            + self.set_energy as i32 * 1_000
            + self.set_rec as i32 * 10
            + i32::from(self.set_passthrough);
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
        let pass_value: &[u8] = if self.set_passthrough { b"ON" } else { b"OFF" };
        // r239: the REC interval row — the interval a FRESH recording
        // starts with (a resumed log keeps its own).
        let rec_value: &[u8] = match crate::settings::REC_INTERVALS
            .get(self.set_rec)
            .copied()
            .unwrap_or(1)
        {
            1 => b"1 S",
            2 => b"2 S",
            3 => b"3 S",
            5 => b"5 S",
            10 => b"10 S",
            20 => b"20 S",
            30 => b"30 S",
            _ => b"60 S",
        };
        // r232: the settings as outlined cards (the box style the
        // graphical screens use) — the cursor row's card in yellow, the
        // other in gray, the value big inside. r239: four rows now
        // (the REC interval joins).
        let rows: [(u16, u16, &[u8], &[u8]); 4] = [
            (34, 76, b"DIM IDLE", dim_value),
            (78, 120, b"ENERGY SAVE", energy_value),
            (122, 164, b"PD PASS", pass_value),
            (166, 208, b"REC INT", rec_value),
        ];
        for (i, (y0, y1, label, value)) in rows.iter().enumerate() {
            let cursor = self.set_mode != SetMode::Idle && self.set_row == i;
            let color = if cursor { YELLOW } else { GRAY };
            card(lcd, 4, *y0, 315, *y1, label, color).await;
            let mut v = Line12::new();
            v.push(value);
            v.pad();
            lcd.draw_text(20, y0 + 18, v.bytes(), 2, color, BLACK).await;
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

    /// r229: the PD PROFILES screen — kws-x1's draw_fixed: one row per
    /// advertised fixed PDO ("5V  3.000A  15W", the voltage in the
    /// overview's level colour), the contract in force badged ACTIVE,
    /// the previewed row framed while editing. Before any caps the page
    /// only shows the notice. Redraws on the state identity.
    pub async fn draw_profiles(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        let n = crate::pd_task::CAPS_N.load(Ordering::Relaxed);
        let contract_mv = crate::pd_task::VIEW.selected_mv.load(Ordering::Relaxed);
        let contracted = crate::pd_task::VIEW.contract.load(Ordering::Relaxed);
        let ident = (
            self.profiles_edit,
            self.profiles_sel,
            n,
            if contracted { contract_mv } else { 0 },
        );
        if ident == self.profiles_drawn {
            return;
        }
        self.profiles_drawn = ident;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        if crate::settings::passthrough() {
            // kws-x1 HIDES the sink pages in passthrough (the meter never
            // negotiates); fnb58's flat screen list shows the notice.
            text(lcd, 76, 100, b"PASSTHROUGH: SINK OFF", 2, GRAY).await;
            text(lcd, 100, 130, b"SET: PD PASS OFF", 1, GRAY).await;
            self.draw_hint(lcd).await;
            return;
        }
        if n == 0 {
            // kws-x1's notice(target, "NO PD CHARGER", ...).
            text(lcd, 92, 100, b"NO PD CHARGER", 2, GREEN).await;
            text(lcd, 100, 130, b"PLUG IN A PD SOURCE", 1, GRAY).await;
            self.draw_hint(lcd).await;
            return;
        }
        for i in 0..n {
            let mv = crate::pd_task::CAPS_MV[i].load(Ordering::Relaxed);
            let ma = crate::pd_task::CAPS_MA[i].load(Ordering::Relaxed);
            let y = 40 + i as u16 * 30;
            if self.profiles_edit && self.profiles_sel == i {
                // The previewed row framed (kws-x1's editing frame).
                lcd.rounded_outline(4, y - 6, 315, y + 22, 5, GREEN).await;
            }
            // The voltage in the overview's level colour (kws-x1's
            // level_color), the amps and watts in gray.
            let mut l = Line12::new();
            l.num_u32(u32::from(mv), 0);
            l.push(b"V");
            l.pad();
            lcd.draw_text(12, y, l.bytes(), 2, level_color(mv), BLACK).await;
            let mut a = Line12::new();
            a.num_u32(u32::from(ma), 3);
            a.push(b"A");
            a.pad();
            lcd.draw_text(110, y, a.bytes(), 2, GRAY, BLACK).await;
            let mut w = Line12::new();
            w.num_u32(u32::from(mv) * u32::from(ma) / 1_000_000, 0);
            w.push(b"W");
            w.pad();
            lcd.draw_text(210, y, w.bytes(), 2, GRAY, BLACK).await;
            if contracted && mv == contract_mv {
                // The ACTIVE badge (kws-x1's green rounded outline).
                lcd.rounded_outline(258, y - 2, 312, y + 16, 5, GREEN).await;
                text(lcd, 265, y + 2, b"ACTIVE", 1, GREEN).await;
            }
        }
        self.draw_hint(lcd).await;
    }


    /// r239: the REC screen — the offline recorder's state, counters
    /// and action menu. The state line is big and colored (green
    /// RUNNING, gray IDLE, yellow STOPPED, orange CLEARING/FULL); the
    /// samples / time / interval / used rows follow; the OK-gated menu
    /// (the r222 directive) draws its actions as outlined boxes with
    /// the cursor in yellow. Redraws on the state/count/menu identity.
    pub async fn draw_rec(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        let state = crate::rec::state();
        let count = crate::rec::count();
        let interval = crate::rec::interval();
        let items = crate::rec::menu(state);
        let sel = self.rec_sel.min(items.len().saturating_sub(1));
        let ident = (
            state,
            count.min(0xFFFF_FFFF),
            self.rec_menu,
            sel,
            crate::rec::CLEAR_DONE.load(Ordering::Relaxed),
            interval,
        );
        if ident == self.rec_ident {
            return;
        }
        self.rec_ident = ident;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        // The state line; FULL overrides STOPPED (the log ran to the
        // area's end — the distinction matters for what to do next).
        let full = state == crate::rec::STOPPED && count >= crate::rec::capacity();
        let (label, color): (&[u8], u16) = match (state, full) {
            (crate::rec::RUNNING, _) => (b"REC RUNNING", GREEN),
            (crate::rec::STOPPED, true) => (b"REC FULL", ORANGE),
            (crate::rec::STOPPED, _) => (b"REC STOPPED", YELLOW),
            (crate::rec::CLEARING, _) => (b"REC CLEARING", ORANGE),
            _ => (b"REC IDLE", GRAY),
        };
        let x = (320 - label.len() as u16 * 15) / 2;
        text(lcd, x, 44, label, 3, color).await;
        if state == crate::rec::CLEARING {
            // The clear's own progress (one sector per loop tick).
            let mut l = Line12::new();
            l.num_u32(u32::from(crate::rec::CLEAR_DONE.load(Ordering::Relaxed)), 0);
            l.push(b"/");
            l.num_u32(u32::from(crate::rec::CLEAR_TOTAL.load(Ordering::Relaxed)), 0);
            l.push(b" SECT");
            let x = (320 - l.len as u16 * 12) / 2;
            lcd.draw_text(x, 90, l.bytes(), 2, ORANGE, BLACK).await;
        } else {
            // SAMPLES n
            let mut l = Line12::new();
            l.push(b"SAMPLES ");
            l.num_u32(count, 0);
            lcd.draw_text(12, 92, l.bytes(), 2, WHITE, BLACK).await;
            // TIME h:mm:ss (n x interval — pauses stretch nothing, no
            // samples are written while stopped).
            let seconds = count.saturating_mul(interval);
            let mut l = Line12::new();
            l.push(b"TIME ");
            l.num_u32(seconds / 3600, 0);
            l.push(b":");
            let m = (seconds / 60) % 60;
            if m < 10 {
                l.push(b"0");
            }
            l.num_u32(m, 0);
            l.push(b":");
            let s = seconds % 60;
            if s < 10 {
                l.push(b"0");
            }
            l.num_u32(s, 0);
            lcd.draw_text(12, 122, l.bytes(), 2, CYAN, BLACK).await;
            // INTERVAL (the LOG's while one exists; the setting's value
            // is what a fresh START would take) and the flash use.
            let shown = if state == crate::rec::NO_LOG {
                let idx = crate::settings::REC_IDX.load(Ordering::Relaxed);
                crate::settings::REC_INTERVALS.get(idx).copied().unwrap_or(1)
            } else {
                interval
            };
            let mut l = Line12::new();
            l.push(b"INT ");
            l.num_u32(shown, 0);
            l.push(b"s  USED ");
            l.num_u32(crate::rec::used_sectors() * 4, 0);
            l.push(b"K");
            lcd.draw_text(12, 152, l.bytes(), 1, GRAY, BLACK).await;
        }
        // The action menu: outlined boxes, the cursor's in yellow.
        if self.rec_menu && state != crate::rec::CLEARING {
            let items = crate::rec::menu(state);
            for (i, item) in items.iter().enumerate() {
                let x0 = if items.len() == 1 {
                    85
                } else {
                    8 + i as u16 * 158
                };
                let color = if i == sel { YELLOW } else { GRAY };
                lcd.rounded_outline(x0, 170, x0 + 148, 206, 5, color).await;
                let x = x0 + (148 - item.len() as u16 * 12) / 2;
                lcd.draw_text(x, 180, item, 2, color, BLACK).await;
            }
        }
        self.draw_hint(lcd).await;
    }

    /// r238: the CABLE screen — kws-x1's Detail::Cable page: the
    /// e-marker's readable summary, raw identity VDOs, the SVID set and
    /// the selected SVID's modes, maker info and both cable statuses,
    /// one section per view (their 8-row pages at scale 1 here, 30
    /// columns = 180 px). The browse mode flips sections; its OK asks
    /// for the SOP' read. Redraws when the published state tuple moves
    /// (the inspection's breadcrumb publishes each change the data).
    pub async fn draw_cable(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        if crate::settings::passthrough() {
            // kws-x1 HIDES the sink pages in passthrough (the meter never
            // negotiates; the r233 profiles notice).
            if !self.cable_notice {
                self.cable_notice = true;
                self.cable_ident = None;
                lcd.fill_window(0, 32, 319, 210, BLACK).await;
                text(lcd, 76, 100, b"PASSTHROUGH: SINK OFF", 2, GRAY).await;
                text(lcd, 100, 130, b"SET: PD PASS OFF", 1, GRAY).await;
                self.draw_hint(lcd).await;
            }
            return;
        }
        let data = crate::pd_task::CABLE.lock(|cell| *cell.borrow());
        let sections = crate::cable_page::sections(&data).max(1);
        let page = if self.cable_browse {
            u32::from(self.cable_page).min(sections - 1)
        } else {
            0
        };
        let svid = crate::pd_task::cable_cursor_svid();
        let ident = (data, self.cable_browse, page, svid);
        if self.cable_ident == Some(ident) {
            return;
        }
        self.cable_ident = Some(ident);
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        if self.cable_browse {
            // The editing frame (the profiles selector's yellow outline).
            lcd.rounded_outline(2, 34, 317, 168, 5, YELLOW).await;
        }
        let rows = crate::cable_page::lines(
            &data,
            page,
            svid,
            &crate::pd_task::CABLE_SVID_STORE,
        );
        for (i, row) in rows.iter().enumerate() {
            let color = if i == 0 { GREEN } else { WHITE };
            // The heading in the screen's accent; 17 px rows like the
            // sniff list.
            lcd.draw_text(10, 40 + i as u16 * 17, row, 1, color, BLACK)
                .await;
        }
        // The section position, kws-x1's page dots as a count.
        let mut l = Line12::new();
        l.num_u32(u32::from(page) + 1, 0);
        l.push(b"/");
        l.num_u32(u32::from(sections), 0);
        let x = 300 - l.len as u16 * 6;
        lcd.draw_text(x, 190, l.bytes(), 1, GRAY, BLACK).await;
        self.draw_hint(lcd).await;
    }

    /// r233: the PD SNIFF screen — kws-x1's PD Sniff: the head line
    /// (the CC pin, the charger's Rp advertisement and the message
    /// count) and the heard messages, oldest first, via the vendored
    /// describe(). Without the passthrough setting: the notice.
    /// Redraws on the published message count.
    pub async fn draw_sniff(&mut self, lcd: &mut Lcd) {
        use core::sync::atomic::Ordering;
        if !crate::settings::passthrough() {
            // The notice is static: draw it once (the r230b flicker
            // class — an ungated redraw per tick strobes the screen).
            if self.sniff_drawn != 0 {
                self.sniff_drawn = 0;
                lcd.fill_window(0, 32, 319, 210, BLACK).await;
                text(lcd, 84, 92, b"PASSTHROUGH OFF", 2, GRAY).await;
                text(lcd, 76, 122, b"SET: PD PASS ON", 1, GRAY).await;
                self.draw_hint(lcd).await;
            }
            return;
        }
        let m = crate::pd_task::SNIFF_MESSAGES.load(Ordering::Relaxed);
        let tag = m.min(999);
        if tag == self.sniff_drawn {
            return;
        }
        self.sniff_drawn = tag;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        // The head: "SNIFF CC1 RP3.0A M:9" (kws-x1's SNIFF line).
        let head = *crate::pd_task::SNIFF_HEAD.lock().await;
        let mut h = [0u8; 24];
        let hn = head.iter().position(|&b| b == 0).unwrap_or(24);
        h[..hn].copy_from_slice(&head[..hn]);
        h[hn] = 0;
        text(lcd, 8, 38, &h, 2, CYAN).await;
        // "PASSTHROUGH RX ONLY NO TX NO GOODCRC" (kws-x1's notices), and
        // the waiting state while no message has arrived (kws-x1's
        // "WAITING FOR PD TRAFFIC" — a source only transmits to a sink's
        // Rd, so without a device sharing the CC line there is nothing
        // to hear).
        text(lcd, 8, 62, b"RX ONLY - NO TX NO CRC", 1, GRAY).await;
        if m == 0 {
            text(lcd, 84, 92, b"WAITING FOR PD", 2, GRAY).await;
            text(lcd, 68, 122, b"TRAFFIC", 2, GRAY).await;
        }
        // The messages, oldest first, from the published rows.
        let rows = crate::pd_task::SNIFF_ROWS.lock().await;
        for (i, row) in rows.iter().enumerate() {
            let mut buf = [0u8; 24];
            let n = row.iter().position(|&b| b == 0).unwrap_or(24);
            buf[..n].copy_from_slice(&row[..n]);
            buf[n] = 0;
            text(lcd, 8, 86 + i as u16 * 17, &buf, 1, WHITE).await;
        }
        self.draw_hint(lcd).await;
    }

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
        value_text(lcd, 172, 46, ua.unsigned_abs(), 3, b"A", CYAN, 7).await;
        // Row 2: the power card and the session peaks.
        card(lcd, 4, 90, 240, 154, b"POWER", GREEN).await;
        value_text(lcd, 12, 104, mw.unsigned_abs(), 4, b"W", GREEN, 9).await;
        card(lcd, 248, 90, 315, 154, b"MAX", MAGENTA).await;
        let peaks = [
            (crate::qc_task::PEAK_MV.load(Ordering::Relaxed), b"V", YELLOW),
            (crate::qc_task::PEAK_UA.load(Ordering::Relaxed), b"A", CYAN),
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
    /// r235: the PPS screen — kws-x1's draw_pps: the PPS RANGE card
    /// (each advertised APDO "3.300-11.000V 3.000A"), the SET VOLTAGE
    /// and CURRENT LIMIT boxes (framed while LEFT/RIGHT adjust them:
    /// ±20 mV / ±50 mA per press, ±100 / ±250 held — clamped to the
    /// range and the 1 A PPS minimum), and the OUTPUT row (the
    /// established contract vs the set level). OK walks
    /// None → volts → amps → request. Before any caps with a PPS APDO:
    /// the NO PPS notice (fixed profiles live on PD PROFILES).
    pub async fn draw_pd(&mut self, lcd: &mut Lcd, view: &View) {
        use core::sync::atomic::Ordering;
        let n = crate::pd_task::PPS_N.load(Ordering::Relaxed);
        let sel_mv = view.selected_mv.load(Ordering::Relaxed);
        let sel_ma = view.selected_ma.load(Ordering::Relaxed);
        // r236: the phase is the truthful contract signal — r178 sets
        // Contracted at PS_RDY while the observation's contract flag
        // lags (the OUTPUT row read NO CONTRACT at ph=11).
        let contracted =
            view.phase.load(Ordering::Relaxed) == 11 || view.contract.load(Ordering::Relaxed);
        let focus_code = match self.pps_focus {
            None => 0u16,
            Some(PpsFocus::Volts) => 1,
            Some(PpsFocus::Amps) => 2,
        };
        let ident = (
            focus_code,
            self.pps_mv,
            self.pps_ma,
            n,
            sel_mv,
            sel_ma,
            contracted,
        );
        if ident == self.pps_drawn {
            return;
        }
        self.pps_drawn = ident;
        lcd.fill_window(0, 32, 319, 210, BLACK).await;
        if n == 0 {
            text(lcd, 116, 100, b"NO PPS", 2, CYAN).await;
            text(lcd, 56, 130, b"CHARGER OFFERS NO PPS", 1, GRAY).await;
            self.draw_hint(lcd).await;
            return;
        }
        // The ranges card, one row per APDO.
        let rows = 34 + (n as u16 - 1) * 14 + 12;
        card(lcd, 4, 34, 315, 34 + 20 + n as u16 * 14, b"PPS RANGE", CYAN).await;
        for k in 0..n {
            let min_mv = crate::pd_task::PPS_MIN_MV[k].load(Ordering::Relaxed);
            let max_mv = crate::pd_task::PPS_MAX_MV[k].load(Ordering::Relaxed);
            let max_ma = crate::pd_task::PPS_MAX_MA[k].load(Ordering::Relaxed);
            let mut l = Line12::new();
            l.num_u32(u32::from(min_mv), 3);
            l.push(b"-");
            l.num_u32(u32::from(max_mv), 3);
            l.push(b"V ");
            l.num_u32(u32::from(max_ma), 3);
            l.push(b"A");
            let w = l.len as u16 * 6;
            l.pad();
            lcd.draw_text(305 - w, 44 + k as u16 * 14, l.bytes(), 1, CYAN, BLACK)
                .await;
        }
        let _ = rows;
        // The SET VOLTAGE and CURRENT LIMIT boxes side by side, framed
        // (yellow) while adjusted.
        let volts_focused = self.pps_focus == Some(PpsFocus::Volts);
        let amps_focused = self.pps_focus == Some(PpsFocus::Amps);
        card(
            lcd,
            4,
            74 + n as u16 * 14,
            156,
            150 + n as u16 * 14,
            b"SET VOLTAGE",
            if volts_focused { YELLOW } else { CYAN },
        )
        .await;
        let mut v = Line12::new();
        v.num_u32(u32::from(self.pps_mv), 3);
        v.push(b"V");
        while v.len < 9 {
            v.push(b" ");
        }
        lcd.draw_text(
            12,
            88 + n as u16 * 14,
            v.bytes(),
            2,
            if volts_focused { YELLOW } else { CYAN },
            BLACK,
        )
        .await;
        card(
            lcd,
            164,
            74 + n as u16 * 14,
            315,
            150 + n as u16 * 14,
            b"CURRENT LIMIT",
            if amps_focused { YELLOW } else { CYAN },
        )
        .await;
        let mut a = Line12::new();
        a.num_u32(u32::from(self.pps_ma), 3);
        a.push(b"A");
        while a.len < 9 {
            a.push(b" ");
        }
        lcd.draw_text(
            172,
            88 + n as u16 * 14,
            a.bytes(),
            2,
            if amps_focused { YELLOW } else { CYAN },
            BLACK,
        )
        .await;
        // The OUTPUT row (kws-x1's supply_status): the established
        // contract, ACTIVE when it is the set level.
        let applied = contracted
            && sel_mv == self.pps_mv
            && sel_ma == self.pps_ma;
        let mut o = Line12::new();
        o.push(b"OUTPUT ");
        o.num_u32(u32::from(sel_mv), 3);
        o.push(b"V ");
        o.num_u32(u32::from(sel_ma), 3);
        o.push(b"A");
        o.pad();
        lcd.draw_text(
            8,
            164 + n as u16 * 14,
            o.bytes(),
            1,
            if applied { GREEN } else { GRAY },
            BLACK,
        )
        .await;
        let state: &[u8] = if applied {
            b"ACTIVE"
        } else if contracted {
            b"NOT APPLIED"
        } else {
            b"NO CONTRACT"
        };
        // FONT_CHARS-safe ("NOT APPLIED" has no lowercase issue).
        text(
            lcd,
            313 - state.len() as u16 * 6,
            164 + n as u16 * 14,
            state,
            1,
            if applied { GREEN } else { ORANGE },
        )
        .await;
        self.draw_hint(lcd).await;
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
