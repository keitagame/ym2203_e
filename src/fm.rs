//! FM (OPN) synthesis core: 3 channels x 4 operators.
//!
//! This models the register-level behaviour of the YM2203 FM section
//! (algorithms, feedback, key on/off, envelope generator phases, rate
//! scaling, SSG-EG loop mode, channel-3 special/CSM frequency mode)
//! using floating point DSP internally. Pitch (frequency) is computed
//! from the exact hardware F-Number/Block formula, so intervals and
//! tuning are accurate. The envelope generator's *shape* (attack /
//! decay / sustain / release with rate scaling) follows the real
//! chip's behaviour qualitatively, but the exact per-step timing is a
//! calibrated exponential approximation rather than a bit-exact
//! reproduction of the original silicon's internal counter tables.
//! See README.md for details.

use std::f64::consts::TAU;

pub const NUM_CH: usize = 3;
const NUM_OP: usize = 4;
const MAX_ATTEN_DB: f64 = 96.0;

/// Maps the 4-bit "keycode note" nibble taken from the top bits of the
/// F-Number to the 2-bit note-group used for envelope rate scaling and
/// detune. This is the standard OPN keycode table.
const NOTE_TABLE: [u8; 16] = [0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 3, 3, 3, 3, 3];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum EgPhase {
    Attack,
    Decay,
    Sustain,
    Release,
    Off,
}

#[derive(Clone)]
pub(crate) struct Operator {
    // ---- register-controlled parameters ----
    pub mul: u8,    // 0..=15  (0 => x0.5)
    pub dt: u8,     // 0..=7   (detune, bit2 = sign)
    pub tl: u8,     // 0..=127 total level (0.75dB/step)
    pub ks: u8,     // 0..=3   key scale (rate scaling amount)
    pub ar: u8,     // 0..=31  attack rate
    pub dr: u8,     // 0..=31  decay rate (D1R)
    pub sr: u8,     // 0..=31  sustain rate (D2R)
    pub sl: u8,     // 0..=15  sustain level (D1L)
    pub rr: u8,     // 0..=15  release rate
    pub ssg_eg: u8, // 0..=15  SSG-EG register (bit3 = enable, bits0-2 = mode)

    // ---- runtime state ----
    phase: f64,      // 0..1 (turns)
    freq_hz: f64,
    atten_db: f64,   // current envelope attenuation (0 = full volume)
    eg_phase: EgPhase,
    key_on: bool,
    ssg_invert: bool,
}

impl Operator {
    fn new() -> Self {
        Operator {
            mul: 0,
            dt: 0,
            tl: 127,
            ks: 0,
            ar: 0,
            dr: 0,
            sr: 0,
            sl: 0,
            rr: 0,
            ssg_eg: 0,
            phase: 0.0,
            freq_hz: 0.0,
            atten_db: MAX_ATTEN_DB,
            eg_phase: EgPhase::Off,
            key_on: false,
            ssg_invert: false,
        }
    }

    fn keycode(block: u8, fnum: u16) -> u8 {
        let note4 = ((fnum >> 7) & 0xF) as usize;
        ((block & 7) << 2) | NOTE_TABLE[note4]
    }

    /// Recompute this operator's output frequency from the channel's
    /// F-Number/Block plus this operator's own MUL/DT, using the
    /// standard OPN frequency formula:
    ///   f = fnum * clock * 2^block / (144 * 2^20)
    fn set_freq(&mut self, fnum: u16, block: u8, clock: u32) {
        let base = fnum as f64 * clock as f64 * (1u64 << (block & 7)) as f64
            / (144.0 * (1u64 << 20) as f64);
        let mul = if self.mul == 0 { 0.5 } else { self.mul as f64 };
        // Detune: small approximate pitch offset in cents, scaled by DT (0..3
        // magnitude, bit2 = sign), consistent with the real chip's DT table
        // shape (bigger DT => bigger spread, symmetrical +/-).
        const DT_MAG_CENTS: [f64; 4] = [0.0, 6.0, 12.0, 24.0];
        let mag = DT_MAG_CENTS[(self.dt & 3) as usize];
        let cents = if self.dt & 4 != 0 { -mag } else { mag };
        self.freq_hz = base * mul * 2f64.powf(cents / 1200.0);
    }

    fn key_on(&mut self) {
        if !self.key_on {
            self.key_on = true;
            self.eg_phase = EgPhase::Attack;
            self.phase = 0.0;
            self.ssg_invert = false;
            // AR==0 means "never attacks" on real hardware; leave atten as-is
            // (will simply not progress, matching that quirky behaviour).
        }
    }

    fn key_off(&mut self) {
        if self.key_on {
            self.key_on = false;
            if self.eg_phase != EgPhase::Off {
                self.eg_phase = EgPhase::Release;
            }
        }
    }

    fn effective_rate(&self, base_rate: u8, keycode: u8) -> u8 {
        if base_rate == 0 {
            return 0;
        }
        let ks_shift = 3 - self.ks; // ks 0..3 -> shift 3..0
        let ks_scale = keycode >> ks_shift;
        ((base_rate as u16 * 2) + ks_scale as u16).min(63) as u8
    }

    fn rate_to_db_per_sample(rate: u8, sample_rate: f64) -> f64 {
        if rate == 0 {
            return 0.0;
        }
        const K: f64 = 1.5;
        K * 2f64.powf(rate as f64 / 4.0) / sample_rate
    }

    fn rate_to_attack_coeff(rate: u8, sample_rate: f64) -> f64 {
        if rate == 0 {
            return 0.0;
        }
        const KA: f64 = 5.0;
        (KA * 2f64.powf(rate as f64 / 4.0) / sample_rate).min(1.0)
    }

    fn sustain_level_db(&self) -> f64 {
        if self.sl == 15 {
            93.0
        } else {
            self.sl as f64 * 3.0
        }
    }

    /// Handle SSG-EG hardware envelope looping (register 0x90-0x9F).
    /// This is a simplified but characteristic reproduction: when the
    /// envelope reaches silence while a key is still held and SSG-EG is
    /// enabled, the envelope loops (attack/alternate bits shape the
    /// resulting buzzy AY-style waveform) instead of staying silent.
    fn handle_ssg_eg_loop(&mut self) {
        let enable = self.ssg_eg & 0x08 != 0;
        if !enable || !self.key_on {
            return;
        }
        let attack = self.ssg_eg & 0x04 != 0;
        let alternate = self.ssg_eg & 0x02 != 0;
        let hold = self.ssg_eg & 0x01 != 0;

        if hold && !alternate {
            // Hold at silence/full depending on attack polarity.
            self.atten_db = if attack { 0.0 } else { MAX_ATTEN_DB };
            self.eg_phase = EgPhase::Off;
            return;
        }
        if alternate {
            self.ssg_invert = !self.ssg_invert;
        }
        self.atten_db = 0.0;
        self.eg_phase = EgPhase::Attack;
        if !hold {
            // keep looping (Continue behaviour)
        }
    }

    fn update_envelope(&mut self, keycode: u8, sample_rate: f64) {
        match self.eg_phase {
            EgPhase::Attack => {
                let r = self.effective_rate(self.ar, keycode);
                if r == 0 {
                    return;
                }
                let coeff = Self::rate_to_attack_coeff(r, sample_rate);
                self.atten_db -= self.atten_db * coeff + 0.001;
                if self.atten_db <= 0.02 {
                    self.atten_db = 0.0;
                    self.eg_phase = EgPhase::Decay;
                }
            }
            EgPhase::Decay => {
                let r = self.effective_rate(self.dr, keycode);
                if r == 0 {
                    return;
                }
                self.atten_db += Self::rate_to_db_per_sample(r, sample_rate);
                let sl_db = self.sustain_level_db();
                if self.atten_db >= sl_db {
                    self.atten_db = sl_db;
                    self.eg_phase = EgPhase::Sustain;
                }
            }
            EgPhase::Sustain => {
                let r = self.effective_rate(self.sr, keycode);
                if r == 0 {
                    return;
                }
                self.atten_db += Self::rate_to_db_per_sample(r, sample_rate);
                if self.atten_db >= MAX_ATTEN_DB {
                    self.atten_db = MAX_ATTEN_DB;
                    self.eg_phase = EgPhase::Off;
                    self.handle_ssg_eg_loop();
                }
            }
            EgPhase::Release => {
                // RR is 4 bits; real hardware maps it so release always
                // progresses (never frozen), roughly rate = RR*2+1.
                let base = (self.rr * 2 + 1).min(31);
                let r = self.effective_rate(base, keycode).max(1);
                self.atten_db += Self::rate_to_db_per_sample(r, sample_rate);
                if self.atten_db >= MAX_ATTEN_DB {
                    self.atten_db = MAX_ATTEN_DB;
                    self.eg_phase = EgPhase::Off;
                    self.handle_ssg_eg_loop();
                }
            }
            EgPhase::Off => {
                self.handle_ssg_eg_loop();
            }
        }
    }

    fn advance_phase(&mut self, sample_rate: f64) {
        self.phase += self.freq_hz / sample_rate;
        self.phase -= self.phase.floor();
    }

    /// Compute this operator's output sample given a phase-modulation
    /// input (in "turns", i.e. fractions of a full cycle).
    fn output(&self, modulation: f64) -> f64 {
        if self.eg_phase == EgPhase::Off {
            return 0.0;
        }
        let amp_db = self.atten_db + self.tl as f64 * 0.75;
        let amp = 10f64.powf(-amp_db / 20.0);
        let mut phase = self.phase + modulation;
        phase -= phase.floor();
        let s = (phase * TAU).sin() * amp;
        if self.ssg_invert {
            -s
        } else {
            s
        }
    }
}

#[derive(Clone)]
pub(crate) struct Channel {
    pub ops: [Operator; NUM_OP],
    pub algorithm: u8, // 0..=7
    pub feedback: u8,  // 0..=7
    pub fnum: u16,
    pub block: u8,
    // Channel-3 "special mode" per-operator frequencies (slots S1,S2,S3;
    // S4 keeps using the normal fnum/block above).
    pub special_fnum: [u16; 3],
    pub special_block: [u8; 3],
    pub special_mode: bool,

    fb_hist: [f64; 2],
}

impl Channel {
    fn new() -> Self {
        Channel {
            ops: [Operator::new(), Operator::new(), Operator::new(), Operator::new()],
            algorithm: 0,
            feedback: 0,
            fnum: 0,
            block: 0,
            special_fnum: [0; 3],
            special_block: [0; 3],
            special_mode: false,
            fb_hist: [0.0; 2],
        }
    }

    fn op_freq_source(&self, slot: usize) -> (u16, u8) {
        if self.special_mode && slot < 3 {
            (self.special_fnum[slot], self.special_block[slot])
        } else {
            (self.fnum, self.block)
        }
    }

    pub fn update_freqs(&mut self, clock: u32) {
        for slot in 0..NUM_OP {
            let (fnum, block) = self.op_freq_source(slot);
            self.ops[slot].set_freq(fnum, block, clock);
        }
    }

    fn keycodes(&self) -> [u8; NUM_OP] {
        let mut kc = [0u8; NUM_OP];
        for slot in 0..NUM_OP {
            let (fnum, block) = self.op_freq_source(slot);
            kc[slot] = Operator::keycode(block, fnum);
        }
        kc
    }

    /// Render one sample for this channel and advance internal state by
    /// one sample period.
    pub fn render(&mut self, sample_rate: f64) -> f64 {
        let fb_scale = if self.feedback == 0 {
            0.0
        } else {
            (1u32 << self.feedback) as f64 / 128.0
        };
        let fb_in = (self.fb_hist[0] + self.fb_hist[1]) * 0.5 * fb_scale;

        let o = &self.ops;
        let out1 = o[0].output(fb_in);

        let (mix, norm): (f64, f64) = match self.algorithm {
            0 => {
                let out2 = o[1].output(out1);
                let out3 = o[2].output(out2);
                let out4 = o[3].output(out3);
                (out4, 1.0)
            }
            1 => {
                let out2 = o[1].output(0.0);
                let out3 = o[2].output(out1 + out2);
                let out4 = o[3].output(out3);
                (out4, 1.0)
            }
            2 => {
                let out2 = o[1].output(0.0);
                let out3 = o[2].output(out2);
                let out4 = o[3].output(out1 + out3);
                (out4, 1.0)
            }
            3 => {
                let out2 = o[1].output(out1);
                let out3 = o[2].output(0.0);
                let out4 = o[3].output(out2 + out3);
                (out4, 1.0)
            }
            4 => {
                let out2 = o[1].output(out1);
                let out3 = o[2].output(0.0);
                let out4 = o[3].output(out3);
                (out2 + out4, 2.0)
            }
            5 => {
                let out2 = o[1].output(out1);
                let out3 = o[2].output(out1);
                let out4 = o[3].output(out1);
                (out2 + out3 + out4, 3.0)
            }
            6 => {
                let out2 = o[1].output(out1);
                let out3 = o[2].output(0.0);
                let out4 = o[3].output(0.0);
                (out2 + out3 + out4, 3.0)
            }
            _ => {
                let out2 = o[1].output(0.0);
                let out3 = o[2].output(0.0);
                let out4 = o[3].output(0.0);
                (out1 + out2 + out3 + out4, 4.0)
            }
        };

        self.fb_hist[1] = self.fb_hist[0];
        self.fb_hist[0] = out1;

        let kc = self.keycodes();
        for slot in 0..NUM_OP {
            self.ops[slot].advance_phase(sample_rate);
            self.ops[slot].update_envelope(kc[slot], sample_rate);
        }

        mix / norm
    }

    /// CSM mode: fire a brief key-on/key-off pulse on all 4 slots
    /// (used to auto-trigger channel 3 from Timer A overflow).
    pub fn csm_trigger(&mut self) {
        for op in self.ops.iter_mut() {
            op.key_on();
        }
    }
}

fn decode_op_addr(addr: u8, base: u8) -> Option<(usize, usize)> {
    if addr < base || addr > base + 0x0F {
        return None;
    }
    let off = addr - base; // 0..15
    let ch = (off & 3) as usize;
    if ch >= NUM_CH {
        return None;
    }
    let group = (off >> 2) as usize; // 0..3
    const GROUP_TO_SLOT: [usize; 4] = [0, 2, 1, 3]; // physical 1,3,2,4 addressing quirk
    Some((ch, GROUP_TO_SLOT[group]))
}

pub(crate) struct FmCore {
    pub channels: [Channel; NUM_CH],
    clock: u32,
}

impl FmCore {
    pub fn new(clock: u32) -> Self {
        FmCore {
            channels: [Channel::new(), Channel::new(), Channel::new()],
            clock,
        }
    }

    pub fn set_clock(&mut self, clock: u32) {
        self.clock = clock;
        for ch in self.channels.iter_mut() {
            ch.update_freqs(self.clock);
        }
    }

    pub fn set_special_mode(&mut self, enabled: bool) {
        self.channels[2].special_mode = enabled;
        self.channels[2].update_freqs(self.clock);
    }

    /// Register 0x28: key on/off control.
    /// bits0-1: channel select (0,1,2 -- value 3 is invalid/ignored)
    /// bits4-7: slot mask (S1,S2,S3,S4)
    pub fn key_control(&mut self, data: u8) {
        let ch = (data & 0x03) as usize;
        if ch >= NUM_CH {
            return;
        }
        let mask = (data >> 4) & 0x0F;
        for slot in 0..NUM_OP {
            let bit = 1 << slot;
            if mask & bit != 0 {
                self.channels[ch].ops[slot].key_on();
            } else {
                self.channels[ch].ops[slot].key_off();
            }
        }
    }

    /// CSM mode auto key-on/off pulse for channel 3, triggered by Timer A.
    pub fn csm_trigger(&mut self) {
        self.channels[2].csm_trigger();
    }

    pub fn write(&mut self, addr: u8, data: u8) {
        match addr {
            0x30..=0x3F => {
                if let Some((ch, slot)) = decode_op_addr(addr, 0x30) {
                    let op = &mut self.channels[ch].ops[slot];
                    op.dt = (data >> 4) & 0x07;
                    op.mul = data & 0x0F;
                    self.channels[ch].update_freqs(self.clock);
                }
            }
            0x40..=0x4F => {
                if let Some((ch, slot)) = decode_op_addr(addr, 0x40) {
                    self.channels[ch].ops[slot].tl = data & 0x7F;
                }
            }
            0x50..=0x5F => {
                if let Some((ch, slot)) = decode_op_addr(addr, 0x50) {
                    let op = &mut self.channels[ch].ops[slot];
                    op.ks = (data >> 6) & 0x03;
                    op.ar = data & 0x1F;
                }
            }
            0x60..=0x6F => {
                if let Some((ch, slot)) = decode_op_addr(addr, 0x60) {
                    // bit7 = AM enable; YM2203 has no LFO so AM has no
                    // audible effect here and is accepted but ignored.
                    self.channels[ch].ops[slot].dr = data & 0x1F;
                }
            }
            0x70..=0x7F => {
                if let Some((ch, slot)) = decode_op_addr(addr, 0x70) {
                    self.channels[ch].ops[slot].sr = data & 0x1F;
                }
            }
            0x80..=0x8F => {
                if let Some((ch, slot)) = decode_op_addr(addr, 0x80) {
                    let op = &mut self.channels[ch].ops[slot];
                    op.sl = (data >> 4) & 0x0F;
                    op.rr = data & 0x0F;
                }
            }
            0x90..=0x9F => {
                if let Some((ch, slot)) = decode_op_addr(addr, 0x90) {
                    self.channels[ch].ops[slot].ssg_eg = data & 0x0F;
                }
            }
            0xA0..=0xA2 => {
                let ch = (addr - 0xA0) as usize;
                self.channels[ch].fnum = (self.channels[ch].fnum & 0x0700) | data as u16;
                self.channels[ch].update_freqs(self.clock);
            }
            0xA4..=0xA6 => {
                let ch = (addr - 0xA4) as usize;
                self.channels[ch].fnum =
                    (self.channels[ch].fnum & 0x00FF) | (((data & 0x07) as u16) << 8);
                self.channels[ch].block = (data >> 3) & 0x07;
                self.channels[ch].update_freqs(self.clock);
            }
            0xA8..=0xAA => {
                let idx = (addr - 0xA8) as usize;
                let ch3 = &mut self.channels[2];
                ch3.special_fnum[idx] = (ch3.special_fnum[idx] & 0x0700) | data as u16;
                ch3.update_freqs(self.clock);
            }
            0xAC..=0xAE => {
                let idx = (addr - 0xAC) as usize;
                let ch3 = &mut self.channels[2];
                ch3.special_fnum[idx] = (ch3.special_fnum[idx] & 0x00FF) | (((data & 0x07) as u16) << 8);
                ch3.special_block[idx] = (data >> 3) & 0x07;
                ch3.update_freqs(self.clock);
            }
            0xB0..=0xB2 => {
                let ch = (addr - 0xB0) as usize;
                self.channels[ch].algorithm = data & 0x07;
                self.channels[ch].feedback = (data >> 3) & 0x07;
            }
            _ => { /* unused on YM2203 */ }
        }
    }

    pub fn render(&mut self, sample_rate: f64) -> f64 {
        let mut sum = 0.0;
        for ch in self.channels.iter_mut() {
            sum += ch.render(sample_rate);
        }
        sum / NUM_CH as f64
    }
}
