use std::fs::File;
use std::io::Read;
use flate2::read::GzDecoder;
use ym2203::Ym2203;
fn load_vgm_or_vgz(path: &str) -> Vec<u8> {
    let mut f = File::open(path).unwrap();

    // 拡張子で判定（vgz → gzip）
    if path.ends_with(".vgz") {
        let mut gz = GzDecoder::new(f);
        let mut data = Vec::new();
        gz.read_to_end(&mut data).unwrap();
        data
    } else {
        let mut data = Vec::new();
        f.read_to_end(&mut data).unwrap();
        data
    }
}

const CLOCK: u32 = 3_993_600;
const SAMPLE_RATE: u32 = 44_100;

fn main() {
    // VGM ファイル読み込み
    
    let vgm = load_vgm_or_vgz("/workspaces/ym2203_e/test.vgz");
    

    // データオフセット取得（ヘッダ 0x34 に書かれている）
    let data_offset = 0x34 + u32::from_le_bytes(vgm[0x34..0x38].try_into().unwrap()) as usize;

    let mut chip = Ym2203::new(CLOCK, SAMPLE_RATE);
    let mut out: Vec<i16> = Vec::new();

    let mut pos = data_offset;

    loop {
        let cmd = vgm[pos];
        pos += 1;

        match cmd {
            0x55 => {
                // YM2203 write: 55 aa dd
                let addr = vgm[pos];
                let data = vgm[pos + 1];
                pos += 2;
                chip.write(addr, data);
            }

            0x61 => {
                // wait n samples
                let n = u16::from_le_bytes(vgm[pos..pos + 2].try_into().unwrap());
                pos += 2;
                out.extend(chip.generate(n as usize));
            }

            0x62 => {
                // wait 1/60 sec
                let n = SAMPLE_RATE / 60;
                out.extend(chip.generate(n as usize));
            }

            0x63 => {
                // wait 1/50 sec
                let n = SAMPLE_RATE / 50;
                out.extend(chip.generate(n as usize));
            }

            0x66 => {
                // end of data
                break;
            }

            _ => {
                // 未対応コマンドは無視
            }
        }
    }

    // WAV 出力
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer = hound::WavWriter::create("vgm_output.wav", spec).unwrap();
    for s in out {
        writer.write_sample(s).unwrap();
    }
    writer.finalize().unwrap();

    println!("Wrote vgm_output.wav");
}
