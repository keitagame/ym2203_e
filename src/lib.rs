

mod fm;
mod ssg;

use fm::FmCore;
use ssg::Ssg;

/// Chip mode selected by register 0x27 bits 6-7.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ChipMode {
    Normal,
    Csm,
    Special3Ch,
}

/// A complete YM2203 chip instance.
pub struct Ym2203 {
    fm: FmCore,
    ssg: Ssg,

    clock: u32,
    sample_rate: u32,
    prescaler_div: u32, // 2, 3, or 6 (default 6)

    // --- timers ---
    timer_a_reg: u16, // 10-bit reload value
    timer_b_reg: u8,  // 8-bit reload value
    timer_a_running: bool,
    timer_b_running: bool,
    timer_a_irq_enable: bool,
    timer_b_irq_enable: bool,
    timer_a_acc: f64, // samples accumulated toward next overflow
    timer_b_acc: f64,

    mode: ChipMode,
    status: u8,

    addr_latch: u8,

    /// Relative SSG/FM mix (SSG amplitude scale). 0.0..=2.0ish, default 1.0.
    pub ssg_mix: f32,
    /// Relative FM mix (FM amplitude scale). 0.0..=2.0ish, default 1.0.
    pub fm_mix: f32,
}

impl Ym2203 {
    /// Create a new chip instance.
    ///
    /// `clock` is the chip's input oscillator frequency in Hz (common
    /// real-world values: 3,993,600 Hz or 4,000,000 Hz for PC-88/PC-98
    /// era hardware; 1,500,000 Hz-3,000,000 Hz on various arcade boards).
    /// `sample_rate` is the desired output PCM sample rate in Hz.
    pub fn new(clock: u32, sample_rate: u32) -> Self {
        Ym2203 {
            fm: FmCore::new(clock),
            ssg: Ssg::new(),
            clock,
            sample_rate,
            prescaler_div: 6,
            timer_a_reg: 0,
            timer_b_reg: 0,
            timer_a_running: false,
            timer_b_running: false,
            timer_a_irq_enable: false,
            timer_b_irq_enable: false,
            timer_a_acc: 0.0,
            timer_b_acc: 0.0,
            mode: ChipMode::Normal,
            status: 0,
            addr_latch: 0,
            ssg_mix: 1.0,
            fm_mix: 1.0,
        }
    }

    fn effective_clock(&self) -> u32 {
        // Formula already assumes the default /6 prescaler; scale
        // relative to that reference when a faster prescaler is selected.
        ((self.clock as u64 * 6) / self.prescaler_div as u64) as u32
    }

    /// Two-step register write, as on the real bus: latch the address,
    /// then write data to it. Equivalent to `write(addr, data)`.
    pub fn write_addr(&mut self, addr: u8) {
        self.addr_latch = addr;
    }

    /// Two-step register write (data half). See [`Ym2203::write_addr`].
    pub fn write_data(&mut self, data: u8) {
        let addr = self.addr_latch;
        self.write(addr, data);
    }

    /// Write a single YM2203 register directly.
    pub fn write(&mut self, addr: u8, data: u8) {
        self.addr_latch = addr;
        match addr {
            0x00..=0x0D => self.ssg.write(addr, data),
            0x24 => {
                self.timer_a_reg = (self.timer_a_reg & 0x0003) | ((data as u16) << 2);
            }
            0x25 => {
                self.timer_a_reg = (self.timer_a_reg & !0x0003) | (data as u16 & 0x03);
            }
            0x26 => {
                self.timer_b_reg = data;
            }
            0x27 => {
                self.timer_a_running = data & 0x01 != 0;
                self.timer_b_running = data & 0x02 != 0;
                self.timer_a_irq_enable = data & 0x04 != 0;
                self.timer_b_irq_enable = data & 0x08 != 0;
                if data & 0x10 != 0 {
                    self.status &= !0x01; // reset Timer A flag
                }
                if data & 0x20 != 0 {
                    self.status &= !0x02; // reset Timer B flag
                }
                self.mode = match (data >> 6) & 0x03 {
                    0b10 => ChipMode::Csm,
                    0b11 => ChipMode::Special3Ch,
                    _ => ChipMode::Normal,
                };
                self.fm.set_special_mode(self.mode == ChipMode::Special3Ch);
            }
            0x28 => self.fm.key_control(data),
            0x2D => {
                self.prescaler_div = 2;
                self.fm.set_clock(self.effective_clock());
            }
            0x2E => {
                self.prescaler_div = 3;
                self.fm.set_clock(self.effective_clock());
            }
            0x2F => {
                self.prescaler_div = 6;
                self.fm.set_clock(self.effective_clock());
            }
            0x30..=0xB7 => self.fm.write(addr, data),
            _ => { /* unused/reserved on YM2203 */ }
        }
    }

    /// Read the chip status register (Timer A/B overflow flags in bits
    /// 0/1; busy flag in bit 7, always 0 in this emulator since register
    /// writes are applied instantaneously).
    pub fn read_status(&self) -> u8 {
        self.status
    }

    /// Read back the value of an SSG register (0x00-0x0D), as real
    /// YM2203 hardware allows. Other addresses return 0.
    pub fn read_ssg(&self, addr: u8) -> u8 {
        if addr <= 0x0D {
            self.ssg.read(addr)
        } else {
            0
        }
    }

    fn tick_timers(&mut self) {
    let fm_clock = self.effective_clock() as f64;

    if self.timer_a_running {
        let period_sec = (1024.0 - self.timer_a_reg as f64) / (fm_clock / 72.0);
        let period_samples = (period_sec * self.sample_rate as f64).max(1.0);
        self.timer_a_acc += 1.0;
        if self.timer_a_acc >= period_samples {
            self.timer_a_acc -= period_samples;
            if self.timer_a_irq_enable {
                self.status |= 0x01;
            }
            if self.mode == ChipMode::Csm {
                self.fm.csm_trigger();
            }
        }
    }

    if self.timer_b_running {
        let period_sec = (256.0 - self.timer_b_reg as f64) / (fm_clock / 288.0);
        let period_samples = (period_sec * self.sample_rate as f64).max(1.0);
        self.timer_b_acc += 1.0;
        if self.timer_b_acc >= period_samples {
            self.timer_b_acc -= period_samples;
            if self.timer_b_irq_enable {
                self.status |= 0x02;
            }
        }
    }
}

    pub fn generate_sample_f32(&mut self) -> f32 {
        self.tick_timers();
        let fm_out = self.fm.render(self.sample_rate as f64) * self.fm_mix as f64;
        let ssg_out = self.ssg.render(self.clock, self.sample_rate) * self.ssg_mix as f64;
        (fm_out * 0.66 + ssg_out * 0.55).clamp(-1.0, 1.0) as f32
    }

    /// Render `n` samples as mono 16-bit signed PCM.
    pub fn generate(&mut self, n: usize) -> Vec<i16> {
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            let s = self.generate_sample_f32();
            out.push((s * 32767.0) as i16);
        }
        out
    }

    /// Render `n` samples as interleaved stereo 16-bit PCM (the YM2203
    /// itself is mono; both channels receive the same signal).
    pub fn generate_stereo(&mut self, n: usize) -> Vec<i16> {
        let mut out = Vec::with_capacity(n * 2);
        for _ in 0..n {
            let s = (self.generate_sample_f32() * 32767.0) as i16;
            out.push(s);
            out.push(s);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn silent_when_idle() {
        let mut chip = Ym2203::new(3_993_600, 44_100);
        let samples = chip.generate(1000);
        assert!(samples.iter().all(|&s| s == 0), "chip should be silent with no notes/tone enabled and no key-on");
    }

    #[test]
    fn produces_finite_nonsilent_output_on_key_on() {
        // Algorithm 7 (all operators are independent carriers): every
        // slot must be configured since all 4 are audible outputs.
        let mut chip = Ym2203::new(3_993_600, 44_100);
        for slot_off in [0u8, 8, 4, 12] {
            chip.write(0x30 + slot_off, 0x01); // MUL=1
            chip.write(0x40 + slot_off, 0x00); // TL=0 (loudest)
            chip.write(0x50 + slot_off, 0x1F); // AR=31 (fastest attack)
            chip.write(0x80 + slot_off, 0x00); // SL=0 RR=0
        }
        chip.write(0xB0, 0x07); // algorithm 7, feedback 0
        chip.write(0xA4, 0x22);
        chip.write(0xA0, 0x69);
        chip.write(0x28, 0xF0);
        let samples = chip.generate(4410);
        assert!(samples.iter().any(|&s| s != 0));
    }

    #[test]
    fn ssg_tone_produces_output() {
        let mut chip = Ym2203::new(3_993_600, 44_100);
        chip.write(0x07, 0b111110); // tone A enabled only
        chip.write(0x08, 0x0F);
        chip.write(0x00, 0x37);
        chip.write(0x01, 0x02);
        let samples = chip.generate(4410);
        assert!(samples.iter().any(|&s| s != 0));
    }

    #[test]
    fn key_off_eventually_decays_to_silence() {
        let mut chip = Ym2203::new(3_993_600, 44_100);
        chip.write(0x30, 0x01);
        chip.write(0x40, 0x00);
        chip.write(0x50, 0x1F);
        chip.write(0x80, 0x0F); // RR = 15, fast release
        chip.write(0xB0, 0x00);
        chip.write(0xA4, 0x22);
        chip.write(0xA0, 0x69);
        chip.write(0x28, 0xF0);
        let _ = chip.generate(2000);
        chip.write(0x28, 0x00); // key off
        let tail = chip.generate(44_100 * 3);
        let last_1000: i32 = tail[tail.len() - 1000..].iter().map(|&s| s.unsigned_abs() as i32).sum();
        assert!(last_1000 < 1000, "expected near-silence after release, got sum={}", last_1000);
    }

    #[test]
    fn timer_a_overflow_sets_status_flag() {
        let mut chip = Ym2203::new(3_993_600, 44_100);
        chip.write(0x24, 0xFF); // Timer A high bits -> near-max reload (short period)
        chip.write(0x25, 0x03);
        chip.write(0x27, 0b0000_0101); // start Timer A + enable IRQ
        let _ = chip.generate(1000);
        assert_eq!(chip.read_status() & 0x01, 0x01, "Timer A should have overflowed");
    }

    #[test]
    fn no_nan_or_inf_in_output() {
        let mut chip = Ym2203::new(3_993_600, 44_100);
        chip.write(0x30, 0x01);
        chip.write(0x40, 0x00);
        chip.write(0x50, 0x1F);
        chip.write(0xB0, 0x07);
        chip.write(0xA4, 0x22);
        chip.write(0xA0, 0x69);
        chip.write(0x28, 0xF0);
        for _ in 0..20 {
            let v = chip.generate_sample_f32();
            assert!(v.is_finite());
        }
    }
}
