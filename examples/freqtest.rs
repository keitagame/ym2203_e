//! Isolated pitch-accuracy test: plays a single sine-ish carrier (algorithm
//! 7, only S4 audible) at a known note on channel 0, and also a known SSG
//! tone on channel A, each in its own WAV file for easy FFT verification.

use ym2203::Ym2203;

const CLOCK: u32 = 3_993_600;
const SAMPLE_RATE: u32 = 44_100;

fn note_to_fnum_block(note: f64, clock: u32) -> (u16, u8) {
    let freq = 440.0 * 2f64.powf((note - 69.0) / 12.0);
    let mut best = (0u16, 0u8, f64::MAX);
    for block in 0..8u8 {
        let fnum = freq * 144.0 * (1u64 << 20) as f64 / (clock as f64 * (1u64 << block) as f64);
        if (1.0..=2047.0).contains(&fnum) {
            let dist = (fnum - 1024.0).abs();
            if dist < best.2 {
                best = (fnum.round() as u16, block, dist);
            }
        }
    }
    (best.0, best.1)
}

fn write_wav(name: &str, samples: &[i16]) {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut w = hound::WavWriter::create(name, spec).unwrap();
    for &s in samples {
        w.write_sample(s).unwrap();
    }
    w.finalize().unwrap();
}

fn main() {
    // --- FM pure tone test: A4 = 440Hz on channel 0, single carrier ---
    let mut chip = Ym2203::new(CLOCK, SAMPLE_RATE);
    for (slot_off, tl) in [(0u8, 0x7Fu8), (8, 0x7F), (4, 0x7F), (12, 0x00)] {
        chip.write(0x30 + slot_off, 0x01); // MUL=1 DT=0
        chip.write(0x40 + slot_off, tl);
        chip.write(0x50 + slot_off, 0x1F); // AR=31 fastest
        chip.write(0x60 + slot_off, 0x00);
        chip.write(0x70 + slot_off, 0x00);
        chip.write(0x80 + slot_off, 0x00); // SL=0 RR=0 (sustain forever at 0dB decay... RR0->min rate applies on release only)
    }
    chip.write(0xB0, 0x07); // algorithm 7, feedback 0

    let (fnum, block) = note_to_fnum_block(69.0, CLOCK); // A4 = 440Hz
    println!("A4 -> fnum={} block={}", fnum, block);
    chip.write(0xA4, (block << 3) | ((fnum >> 8) as u8 & 7));
    chip.write(0xA0, (fnum & 0xFF) as u8);
    chip.write(0x28, 0xF0); // key on ch0 all slots

    let fm_samples = chip.generate(SAMPLE_RATE as usize);
    write_wav("freqtest_fm_440hz.wav", &fm_samples);

    // --- SSG pure tone test: A4 = 440Hz on SSG channel A ---
    let mut chip2 = Ym2203::new(CLOCK, SAMPLE_RATE);
    chip2.write(0x07, 0b111110); // tone A enabled, everything else disabled
    chip2.write(0x08, 0x0F); // vol A = max
    let period = (CLOCK as f64 / (16.0 * 440.0)).round() as u16;
    println!("SSG A4 -> period={}", period);
    chip2.write(0x00, (period & 0xFF) as u8);
    chip2.write(0x01, ((period >> 8) & 0x0F) as u8);
    let ssg_samples = chip2.generate(SAMPLE_RATE as usize);
    write_wav("freqtest_ssg_440hz.wav", &ssg_samples);

    println!("wrote freqtest_fm_440hz.wav and freqtest_ssg_440hz.wav");
}
