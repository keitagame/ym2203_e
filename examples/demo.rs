//! Demo: plays a short arpeggiated FM lead on channel 0, a simple bass
//! drone on channel 1 (algorithm 7, 4 unison carriers), and an SSG
//! square-wave + noise hi-hat pattern on the PSG side, then writes the
//! result to demo_output.wav.
//!
//! Run with: cargo run --release --example demo

use std::f64::consts::LOG2_10;
use ym2203::Ym2203;

const CLOCK: u32 = 3_993_600; // typical PC-88/PC-98 era YM2203 clock
const SAMPLE_RATE: u32 = 44_100;

/// Convert a MIDI-ish note number (A4=69 -> 440Hz) to YM2203 F-Number/Block.
fn note_to_fnum_block(note: f64, clock: u32) -> (u16, u8) {
    let freq = 440.0 * 2f64.powf((note - 69.0) / 12.0);
    // Try blocks 0..=7 and pick the one that keeps fnum in 11-bit range
    // closest to the middle for good resolution.
    let mut best = (0u16, 0u8, f64::MAX);
    for block in 0..8u8 {
        let fnum = freq * 144.0 * (1u64 << 20) as f64 / (clock as f64 * (1u64 << block) as f64);
        if fnum >= 1.0 && fnum <= 2047.0 {
            let dist = (fnum - 1024.0).abs();
            if dist < best.2 {
                best = (fnum.round() as u16, block, dist);
            }
        }
    }
    let _ = LOG2_10; // silence unused-import if optimizer removes usage above
    (best.0, best.1)
}

fn set_channel_freq(chip: &mut Ym2203, ch: u8, note: f64) {
    let (fnum, block) = note_to_fnum_block(note, CLOCK);
    chip.write(0xA4 + ch, ((block & 7) << 3) | ((fnum >> 8) as u8 & 7));
    chip.write(0xA0 + ch, (fnum & 0xFF) as u8);
}

fn setup_lead_voice(chip: &mut Ym2203) {
    // Channel 0: classic 2-op-ish bright bell/lead using algorithm 2,
    // feedback on op1.
    let ch = 0u8;
    // Op S1 (base 0x30/0x40/0x50/0x60/0x70/0x80 + ch)
    chip.write(0x30 + ch, 0x01); // DT=0 MUL=1
    chip.write(0x40 + ch, 0x23); // TL (some attenuation, it's a modulator)
    chip.write(0x50 + ch, 0x1F); // KS=0 AR=31
    chip.write(0x60 + ch, 0x05); // D1R=5
    chip.write(0x70 + ch, 0x02); // D2R=2
    chip.write(0x80 + ch, 0x11); // SL=1 RR=1

    // Op S2 (+8)
    chip.write(0x30 + 8 + ch, 0x07); // MUL=7 (bell-like inharmonic ratio)
    chip.write(0x40 + 8 + ch, 0x1E);
    chip.write(0x50 + 8 + ch, 0x1F);
    chip.write(0x60 + 8 + ch, 0x07);
    chip.write(0x70 + 8 + ch, 0x02);
    chip.write(0x80 + 8 + ch, 0x15);

    // Op S3 (+4)
    chip.write(0x30 + 4 + ch, 0x02); // MUL=2
    chip.write(0x40 + 4 + ch, 0x20);
    chip.write(0x50 + 4 + ch, 0x1F);
    chip.write(0x60 + 4 + ch, 0x06);
    chip.write(0x70 + 4 + ch, 0x02);
    chip.write(0x80 + 4 + ch, 0x14);

    // Op S4 (+12) -- the final carrier
    chip.write(0x30 + 12 + ch, 0x01); // MUL=1
    chip.write(0x40 + 12 + ch, 0x00); // TL=0 loudest (carrier)
    chip.write(0x50 + 12 + ch, 0x1F);
    chip.write(0x60 + 12 + ch, 0x08);
    chip.write(0x70 + 12 + ch, 0x03);
    chip.write(0x80 + 12 + ch, 0x17);

    chip.write(0xB0 + ch, (3 << 3) | 4); // feedback=3, algorithm=4
}

fn setup_bass_voice(chip: &mut Ym2203) {
    // Channel 1: simple single-op-style bass, algorithm 7 (all parallel),
    // only S4 audible-ish via low TL on others.
    let ch = 1u8;
    for (slot_off, mul) in [(0u8, 1u8), (8, 1), (4, 1), (12, 1)] {
        chip.write(0x30 + slot_off + ch, mul);
        chip.write(0x40 + slot_off + ch, if slot_off == 12 { 0x02 } else { 0x7F });
        chip.write(0x50 + slot_off + ch, 0x1A);
        chip.write(0x60 + slot_off + ch, 0x08);
        chip.write(0x70 + slot_off + ch, 0x04);
        chip.write(0x80 + slot_off + ch, 0x28);
    }
    chip.write(0xB0 + ch, 0x07); // algorithm 7, feedback 0
}

fn setup_ssg(chip: &mut Ym2203) {
    // Channel A: square lead doubling, Channel C: noise hi-hat.
    chip.write(0x07, 0b111_011); // tone A,B enabled, noise C enabled (bits: noiseC=1<<5)
    chip.write(0x08, 0x0A); // vol A
    chip.write(0x09, 0x00); // vol B off
    chip.write(0x0A, 0x08); // vol C (hi-hat)
    chip.write(0x06, 0x08); // noise period
}

fn set_ssg_tone_a(chip: &mut Ym2203, note: f64) {
    let freq = 440.0 * 2f64.powf((note - 69.0) / 12.0);
    let period = (CLOCK as f64 / (16.0 * freq)).round().clamp(1.0, 4095.0) as u16;
    chip.write(0x00, (period & 0xFF) as u8);
    chip.write(0x01, ((period >> 8) & 0x0F) as u8);
}

fn main() {
    let mut chip = Ym2203::new(CLOCK, SAMPLE_RATE);
    setup_lead_voice(&mut chip);
    setup_bass_voice(&mut chip);
    setup_ssg(&mut chip);

    let mut out: Vec<i16> = Vec::new();

    // Bass drone: C2 held throughout.
    set_channel_freq(&mut chip, 1, 36.0);
    chip.write(0x28, 0xF1); // key on ch1, all slots

    let melody = [60.0, 64.0, 67.0, 72.0, 67.0, 64.0, 71.0, 72.0]; // C major-ish arpeggio, MIDI notes
    let note_samples = SAMPLE_RATE as usize / 4; // 250ms per note

    for (i, &note) in melody.iter().cycle().take(melody.len() * 2).enumerate() {
        set_channel_freq(&mut chip, 0, note);
        set_ssg_tone_a(&mut chip, note + 12.0);
        chip.write(0x28, 0xF0); // key on ch0
        if i % 2 == 0 {
            chip.write(0x0A, 0x0F); // hi-hat accent
        } else {
            chip.write(0x0A, 0x06);
        }

        let hold = (note_samples as f64 * 0.8) as usize;
        let release = note_samples - hold;
        out.extend(chip.generate(hold));
        chip.write(0x28, 0x00); // key off ch0 (mask=0 -> release all slots off)
        out.extend(chip.generate(release));
    }

    chip.write(0x28, 0x01); // key off ch1
    out.extend(chip.generate(SAMPLE_RATE as usize)); // 1s tail for release/reverb-less decay

    // Sanity check: make sure we actually produced non-silent, finite audio.
    let peak = out.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
    let nonzero = out.iter().filter(|&&s| s != 0).count();
    eprintln!(
        "generated {} samples, peak={}, nonzero_samples={} ({:.1}%)",
        out.len(),
        peak,
        nonzero,
        100.0 * nonzero as f64 / out.len() as f64
    );
    assert!(peak > 1000, "output seems suspiciously quiet/silent");

    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer = hound::WavWriter::create("demo_output.wav", spec).unwrap();
    for s in out {
        writer.write_sample(s).unwrap();
    }
    writer.finalize().unwrap();
    println!("Wrote demo_output.wav");
}
