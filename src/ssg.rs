//! SSG (Software-Controlled Sound Generator) core: the AY-3-8910
//! compatible 3-channel square wave + noise + hardware-envelope PSG
//! embedded in the YM2203, mapped to register addresses 0x00-0x0D.

const NUM_CH: usize = 3;

pub(crate) struct Ssg {
    regs: [u8; 14],

    // Tone generators: simple phase accumulators running at
    // clock / (16 * period).
    tone_phase: [f64; NUM_CH],

    // Noise generator: 17-bit LFSR, stepped by its own phase accumulator.
    noise_phase: f64,
    noise_lfsr: u32,

    // Hardware envelope generator: 0..=15 position, stepped by its own
    // phase accumulator.
    env_phase: f64,
    env_pos: i32, // 0..=15
    env_dir: i32, // +1 / -1
    env_holding: bool,
}

impl Ssg {
    pub fn new() -> Self {
        Ssg {
            regs: [0; 14],
            tone_phase: [0.0; NUM_CH],
            noise_phase: 0.0,
            noise_lfsr: 1,
            env_phase: 0.0,
            env_pos: 0,
            env_dir: 1,
            env_holding: false,
        }
    }

    pub fn write(&mut self, addr: u8, data: u8) {
        let idx = addr as usize;
        if idx >= self.regs.len() {
            return;
        }
        self.regs[idx] = data;
        if idx == 13 {
            // Writing the envelope shape register always restarts the
            // envelope from the beginning, as on real hardware.
            let attack = data & 0x04 != 0;
            self.env_pos = if attack { 0 } else { 15 };
            self.env_dir = if attack { 1 } else { -1 };
            self.env_holding = false;
            self.env_phase = 0.0;
        }
    }

    pub fn read(&self, addr: u8) -> u8 {
        self.regs.get(addr as usize).copied().unwrap_or(0)
    }

    fn tone_period(&self, ch: usize) -> u32 {
        let fine = self.regs[ch * 2] as u32;
        let coarse = (self.regs[ch * 2 + 1] & 0x0F) as u32;
        let p = fine | (coarse << 8);
        p.max(1)
    }

    fn noise_period(&self) -> u32 {
        (self.regs[6] & 0x1F).max(1) as u32
    }

    fn envelope_period(&self) -> u32 {
        let fine = self.regs[11] as u32;
        let coarse = self.regs[12] as u32;
        (fine | (coarse << 8)).max(1)
    }

    fn mixer(&self) -> u8 {
        self.regs[7]
    }

    fn channel_volume_reg(&self, ch: usize) -> u8 {
        self.regs[8 + ch]
    }

    fn tick_envelope_step(&mut self) {
        let shape = self.regs[13];
        let hold = shape & 0x01 != 0;
        let alternate = shape & 0x02 != 0;
        let attack = shape & 0x04 != 0;
        let continue_ = shape & 0x08 != 0;

        if self.env_holding {
            return;
        }

        self.env_pos += self.env_dir;
        if self.env_pos > 15 || self.env_pos < 0 {
            if !continue_ {
                self.env_pos = 0;
                self.env_holding = true;
            } else if hold {
                self.env_pos = if attack ^ alternate { 15 } else { 0 };
                self.env_holding = true;
            } else if alternate {
                self.env_dir = -self.env_dir;
                self.env_pos = if self.env_dir == 1 { 0 } else { 15 };
            } else {
                self.env_pos = if attack { 0 } else { 15 };
            }
        }
    }

    fn envelope_level(&self) -> u8 {
        self.env_pos.clamp(0, 15) as u8
    }

    /// Logarithmic 16-step DAC curve approximation (~consistent with the
    /// AY-3-8910's roughly-3dB-per-two-steps volume table).
    fn level_to_amplitude(level: u8) -> f64 {
        if level == 0 {
            0.0
        } else {
            2f64.powf((level as f64 - 15.0) / 2.0)
        }
    }

    pub fn render(&mut self, clock: u32, sample_rate: u32) -> f64 {
        let sample_rate = sample_rate as f64;
        let clock = clock as f64;

        // --- advance tone generators ---
        let mut tone_bit = [false; NUM_CH];
        for ch in 0..NUM_CH {
            let period = self.tone_period(ch) as f64;
            let freq = clock / (16.0 * period);
            self.tone_phase[ch] += freq / sample_rate;
            self.tone_phase[ch] -= self.tone_phase[ch].floor();
            tone_bit[ch] = self.tone_phase[ch] < 0.5;
        }

        // --- advance noise generator ---
        {
            let period = self.noise_period() as f64;
            let freq = clock / (16.0 * period);
            let inc = freq / sample_rate;
            self.noise_phase += inc;
            while self.noise_phase >= 1.0 {
                self.noise_phase -= 1.0;
                let bit = (self.noise_lfsr ^ (self.noise_lfsr >> 3)) & 1;
                self.noise_lfsr = (self.noise_lfsr >> 1) | (bit << 16);
            }
        }
        let noise_bit = self.noise_lfsr & 1 != 0;

        // --- advance envelope generator ---
        {
            let period = self.envelope_period() as f64;
            let freq = clock / (16.0 * period);
            let inc = freq / sample_rate;
            self.env_phase += inc;
            while self.env_phase >= 1.0 {
                self.env_phase -= 1.0;
                self.tick_envelope_step();
            }
        }
        let env_amp = Self::level_to_amplitude(self.envelope_level());

        // --- mix the 3 channels ---
        let mixer = self.mixer();
        let mut out = 0.0;
        for ch in 0..NUM_CH {
            let tone_disabled = mixer & (1 << ch) != 0;
            let noise_disabled = mixer & (1 << (ch + 3)) != 0;
            let gate = (tone_disabled || tone_bit[ch]) && (noise_disabled || noise_bit);

            let vreg = self.channel_volume_reg(ch);
            let use_envelope = vreg & 0x10 != 0;
            let amp = if use_envelope {
                env_amp
            } else {
                Self::level_to_amplitude(vreg & 0x0F)
            };

            out += if gate { amp } else { -amp };
        }

        out / NUM_CH as f64
    }
}
