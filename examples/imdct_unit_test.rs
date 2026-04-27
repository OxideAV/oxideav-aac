//! Compare our IMDCT output against the spec formula for a single-bin
//! unit input. Should match (modulo our 2x scale convention).
fn main() {
    let mut spec = [0.0f32; 1024];
    spec[20] = 1.0;
    let mut out = vec![0.0f32; 2048];
    oxideav_aac::imdct::imdct_long(&spec, &mut out);
    for &n in &[0usize, 1, 100, 512, 1024, 1500, 2047] {
        println!("ours IMDCT[n={n}] = {:.6e}", out[n]);
    }
    let pk = out.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
    println!("ours IMDCT peak (unit bin 20) = {pk:.6e}");
    println!(
        "spec IMDCT peak (unit bin 20) = 2/N = 2/2048 = {:.6e}",
        2.0f64 / 2048.0
    );
    println!("ratio (ours/spec) = {:.4}", pk as f64 / (2.0 / 2048.0));
}
