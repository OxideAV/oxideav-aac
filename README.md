# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status (round 263)

Round 263 adds the [`tns_coef`](src/tns_coef.rs) module — the
ISO/IEC 14496-3 §4.6.9.3 `tns_decode_coef` inverse-quantisation and
conversion-to-LPC step-up surface, plus the §C.6 encoder-side
companion `tns_encode_coef`. This is the missing bridge between the
existing [`tns_data`](src/tns_data.rs) wire parser (which surfaces
the unsigned-magnitude `coef[i]` field as transmitted, packed into
`coef_res2 = coef_res_bits − coef_compress ∈ {2, 3, 4}` bits) and
the eventual `tns_ar_filter()` all-pole IIR pass that operates on
the reconstructed MDCT spectrum. The decode path follows the
§4.6.9.3 pseudocode literally: the wire `coef[i]` is sign-extended
via the §4.6.9.3 `sgn_mask` / `neg_mask` pair (two's-complement
extension of the truncated `coef_res2`-bit field, validated by
walking every legal wire pattern across all three field widths),
then inverse-quantised via `sin(tmp[i] / iqfac_branch)` where the
divisor branches on the sign of the sign-extended index: `iqfac =
((1 << (coef_res_bits-1)) - 0.5) / (π/2)` for non-negative,
`iqfac_m = ((1 << (coef_res_bits-1)) + 0.5) / (π/2)` for negative.
The half-bit offset matches the §C.6 encoder's rounded
`NINT(arcsin(r) * iqfac_branch)` quantisation so every legal wire
value round-trips through decode → encode bit-exactly (a full
enumeration across all four `(coef_res_bits, coef_compress)`
combinations confirms the invariant: 16 patterns at `(4, 0)`, 8 at
both `(3, 0)` and `(4, 1)`, 4 at `(3, 1)`). The §4.6.9.3
conversion-to-LPC step-up loop is implemented as `lpc_step_up`: it
takes the inverse-quantised PARCOR array and produces the
`order + 1` direct-form LPC `a[]` vector with `a[0] = 1.0` and
`a[order] = parcor[order-1]`, intermediate slots derived by the
spec's `b[i] = a[i] + k * a[m-i]` cross-term recurrence (validated
against hand-arithmetic at orders 1, 2, and 3). The combined
`tns_decode_coef_to_lpc` wrapper runs both passes in one call for
the eventual `tns_decode_frame()` orchestration. Encoder-side
saturation lands at the field extrema: `r = +1.0` rounds to
`NINT(π/2 · iqfac(4)) = NINT(7.5) = 8` clamped to the 4-bit signed
max `+7` (wire `0b0111`), and `r = -1.0` rounds to
`NINT(-π/2 · iqfac_m(4)) = -9` clamped to `-8` (wire `0b1000`).
Argument validation: invalid `coef_res_bits` (anything outside
`{3, 4}`), `coef_compress > 1`, wire `coef[i]` that overflows the
`coef_res2`-bit field, encode PARCOR values `|r| > 1.0` (or NaN /
±∞ — `arcsin` undefined), or `pack_coef` values outside the
field-representable signed range all surface as the new
`Error::TnsCoefOutOfRange`. The §4.6.9.3 `tns_ar_filter()` all-pole
IIR pass and the `tns_decode_frame()` orchestration that chains
`tns_decode_coef_to_lpc` / `tns_ar_filter` per `(window, filter)`
are deferred until the IMDCT back-end lands. The §4.6.17.3.4 ER
AAC LD `int_tns_decode_coef()` integer variant is deferred until
the LD reconstruction path is wired. 36 new unit tests
(`src/tns_coef.rs`) plus 11 new integration tests
(`tests/tns_coef.rs`) lift the total green test count to **1050**.

## Status (round 259)

Round 259 lands **Table 4.A.12** (Spectrum Huffman Codebook 11) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 / 234 / 238 / 241 / 244 / 250 / 253 / 255
bootstrapped for Codebooks 1 through 10. Codebook 11 is the only
**ESC** spectrum book — Table 4.95 row 11 declares `unsigned_cb = 1`,
`dim = 2`, `LAV = 16` with an ESC threshold of `8191` (the §4.6.1.3
`x_quant` ceiling). The §4.6.3.3 in-band universe widens to a
`17 × 17 = 289`-entry lattice indexed `0..=288` with each `(y, z)`
coefficient in `0..=16`; a coefficient value of `16` in either slot
is **not** a literal 16 — it is the §4.6.3.3 `escape_flag` whose
actual magnitude is reconstructed from the `escape_sequence`
(`escape_prefix` of N `1`s, a `0` `escape_separator`, and an
`(N + 4)`-bit `escape_word`) bridged by the round-213
[`spectral_codebook::decode_esc_value`](src/spectral_codebook.rs) /
`encode_esc_value` helpers — outside the Huffman codeword carried
by this module. The §4.6.3.3 unsigned polynomial `idx = y * 17 + z`
parks the zero-tuple `(0, 0)` at index 0 with the shortest 4-bit
codeword `0b0000`, shares that 4-bit floor with the interior `(1, 1)`
pair at index 18 (codeword `0b0001`), pins the half-ESC tuples
`(0, 16)` and `(16, 0)` to a 10-bit `0x38e` at index 16 and a 9-bit
`0x1c2` at index 272, and parks the full-ESC corner `(16, 16)` at
index 288 with the 5-bit `0b00100` (`0x04`) — the wire layout
extends with two sign bits and two escape sequences for that corner,
so the in-band Huffman codeword stays short. The codeword ceiling
matches Codebook 10's **12 bits** — exactly six rows reach that
ceiling (indices 12, 14, 15, 255, 269, 270 with codewords `0xffb`,
`0xffa`, `0xffe`, `0xffd`, `0xffc`, `0xfff`) — because Codebook 11
pushes its tail distribution out of the Huffman table and into the
§4.6.3 ESC sequence. The table is a **complete** 12-bit prefix code
(Kraft equality `Σ 2^(12 − L) = 4096 = 2^12`), exhaustively verified
by walking every 12-bit prefix and asserting each maps to exactly
one entry. Because Codebook 11 is unsigned, the §4.6.3.3 sign-bit
suffix follows the Huffman codeword on the wire — one sign bit per
non-zero coefficient, delivered low-frequency-first — and is exposed
by the round-213 `apply_sign_bits` / `derive_sign_bits` helpers (the
suffix lives outside the Huffman codeword carried by this module, an
invariant verified across every index by an explicit
`derive_sign_bits(11, &tuple)` cross-check that confirms a `(16, 16)`
tuple emits two sign bits, a `(0, 0)` tuple emits zero sign bits,
and the suffix length matches the non-zero-coefficient count for
every index in between). Public API: `HCOD11_NUM_ENTRIES = 289`,
`HCOD11_MAX_LEN = 12`, `hcod11_encode(idx) -> (u8, u16)`,
`hcod11_decode(reader) -> u32`, and the convenience
`hcod11_write(writer, idx)`. Out-of-range indices surface as
`Error::SpectralCodebookIndexOutOfRange(11)`; reader underflow
surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised across every index:
`decode_index_to_tuple(11, idx)` →
`encode_tuple_to_index(11, &tuple)` round-trips back to the same
index for the entire `17 × 17` unsigned pair lattice, and a
bijection cross-check builds the `289`-entry `HashSet` of legal
pairs and confirms every `(y, z)` maps to a distinct index. A
disjoint-set partition cross-check splits the 289 indices into the
`256` in-band pairs (`y, z ∈ 0..=15`) and the `33` ESC-border pairs
(`y == 16 || z == 16`) and confirms the two sets cover the universe
without overlap. 20 new unit tests (in `src/spectrum_huffman.rs`:
289-entry shape / 12-bit max / 4-bit min at indices 0 and 18 /
zero-tuple-at-index-0 / interior-`(1, 1)`-at-index-18 / far-corner-
at-index-288 / codewords fit / Kraft equality 4096 / complete-
prefix walk over all `2^12` prefixes / six-12-bit-ceiling
enumeration at indices 12 / 14 / 15 / 255 / 269 / 270 / half-ESC
rows at indices 16 and 272 / encode and write rejection / 4-bit
codeword decode at index 0 / full 12-bit codeword decode at
index 270 / writer round-trip / truncation rejection /
`HCOD11_MAX_LEN` constant consistency / 12-bit ceiling parity with
Codebook 10 / `289 − 169 = 120` universe expansion) plus 30 new
integration tests in `tests/spectrum_huffman.rs` (twelve per-row PDF
spot checks at indices 0 / 1 / 12 / 14 / 15 / 16 / 18 / 255 / 269 /
270 / 272 / 288; Table 4.95 row 11 ↔ Table 4.A.12 size cross-check
including ESC threshold = `8191` and `has_esc()`; full
writer→reader round-trip with bit-consumption invariant; the
§4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check; zero-tuple ↔
index-0-with-4-bit-codeword invariant; interior `(1, 1)` ↔ index-18
invariant; far-corner `(16, 16)` ↔ index-288-with-5-bit-codeword
invariant; half-ESC tuple ↔ index 16 and 272 cross-check; per-index
sign-bit-count invariant against `derive_sign_bits(11, …)`;
zero-tuple zero-sign-bits invariant; `(16, 16)` two-sign-bits
invariant; two hand-pinned byte sequences (`[0x00]` for index 0
padded; `[0xff, 0xf0]` for index 270 packed); out-of-range and
truncation rejections; `HCOD11_MAX_LEN` constant consistency;
Codebook-11-shares-12-bit-ceiling-with-Codebook-10 cross-check;
Codebook-11-is-the-only-ESC-spectrum-book cross-check confirming
`row11.has_esc() == true` and `row10.has_esc() == false`; 289-pair
bijection sweep that builds every legal `(y, z)` pair, asserts
`encode_tuple_to_index(11, …)` produces a distinct index for each,
and confirms `decode_index_to_tuple(11, idx)` round-trips back to
the same `(y, z)`; in-band/ESC-border disjoint-set partition that
splits the 289 indices into `256` in-band pairs and `33` ESC-border
pairs with `0` overlap). Completes the AAC spectrum Huffman
Codebook 1..=11 table set; the next step is the §4.4.6
`spectral_data()` wire walker that loops over scalefactor bands and
dispatches per-band onto the codebook chosen by `section_data()`.

## Status (round 255)

Round 255 lands **Table 4.A.11** (Spectrum Huffman Codebook 10) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 / 234 / 238 / 241 / 244 / 250 / 253 bootstrapped
for Codebooks 1 through 9. Codebook 10 is the second **expanded-LAV
unsigned pair** spectrum book — Table 4.95 row 10 mirrors row 9
column-for-column (`unsigned_cb = 1`, `dim = 2`, `LAV = 12`) so the
§4.6.3.3 universe is the same `13 × 13 = 169`-entry lattice indexed
`0..=168` with each `(y, z)` coefficient in `0..=12`. The two books
differ in their codeword-length distribution: where Codebook 9 parks
the zero-tuple `(0, 0)` at index 0 with the single-bit `0` codeword
and lets the four rarest pair magnitudes climb to a 15-bit ceiling,
Codebook 10 lifts the zero-tuple at index 0 to a 6-bit `0b100010`
(`0x22`), migrates the shortest 4-bit slot onto the interior `(1, 1)`
tuple at index 14 with codeword `0b0000`, and pulls the codeword
ceiling down to **12 bits** — the same head-displacement pattern
Codebook 8 uses relative to Codebook 7 but at the wider `LAV = 12`
universe. Exactly three rows reach the 4-bit floor (indices 14, 15,
27 with codewords `0x0`, `0x1`, `0x2`) and exactly eight rows reach
the 12-bit ceiling (indices 12, 129, 142, 155, 165, 166, 167, 168
with codewords `0xffd`, `0xffa`, `0xff9`, `0xffb`, `0xff8`, `0xffe`,
`0xffc`, `0xfff`). The table is a **complete** 12-bit prefix code
(Kraft equality `Σ 2^(12 − L) = 4096 = 2^12`), exhaustively verified
by walking every 12-bit prefix and asserting each maps to exactly
one entry. Because Codebook 10 is unsigned, the §4.6.3.3 sign-bit
suffix follows the Huffman codeword on the wire — one sign bit per
non-zero coefficient, delivered low-frequency-first — and is exposed
by the round-213 `apply_sign_bits` / `derive_sign_bits` helpers (the
suffix lives outside the Huffman codeword carried by this module, an
invariant verified across every index by an explicit
`derive_sign_bits(10, &tuple)` cross-check that confirms a `(12, 12)`
tuple emits two sign bits, a `(0, 0)` tuple emits zero sign bits,
and the suffix length matches the non-zero-coefficient count for
every index in between). Public API: `HCOD10_NUM_ENTRIES = 169`,
`HCOD10_MAX_LEN = 12`, `hcod10_encode(idx) -> (u8, u16)`,
`hcod10_decode(reader) -> u32`, and the convenience
`hcod10_write(writer, idx)`. Out-of-range indices surface as
`Error::SpectralCodebookIndexOutOfRange(10)`; reader underflow
surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised across every index: `decode_index_to_tuple
(10, idx)` → `encode_tuple_to_index(10, &tuple)` round-trips back to
the same index for the entire `13 × 13` unsigned pair lattice, and a
bijection cross-check builds the `169`-entry `HashSet` of legal
pairs and confirms every `(y, z)` maps to a distinct index. 17 new
unit tests (in `src/spectrum_huffman.rs`: 169-entry shape / 12-bit
max / 4-bit min at indices 14, 15, 27 / zero-tuple-at-index-0-with-
6-bit-codeword / codewords fit / Kraft equality 4096 / complete-
prefix walk over all `2^12` prefixes / per-row PDF spot checks at
indices 0 / 14 / 15 / 27 / 168 / eight-12-bit-ceiling enumeration
at indices 12 / 129 / 142 / 155 / 165 / 166 / 167 / 168 / encode +
write rejection / 4-bit codeword decode at index 14 / full 12-bit
codeword decode at index 168 / writer round-trip / unsigned-pair
LAV-12 universe parity with Codebook 9 / 3-bit ceiling pull-down vs
Codebook 9 / zero-tuple head-displacement vs Codebook 9) plus 22 new
integration tests in `tests/spectrum_huffman.rs` (ten per-row PDF
spot checks at indices 0 / 1 / 12 / 14 / 15 / 27 / 129 / 142 / 155 /
168; Table 4.95 row 10 ↔ Table 4.A.11 size cross-check; full
writer→reader round-trip; the §4.6.3.3 wire-index ↔ tuple ↔
wire-index cross-check; zero-tuple ↔ index-0 carries 6-bit `0x22`
invariant; shortest 4-bit codeword sits on interior `(1, 1)` tuple
at index 14 invariant; far-corner `(12, 12)` ↔ index-168 invariant;
per-index sign-bit-count invariant against `derive_sign_bits(10,
…)`; zero-tuple zero-sign-bits invariant; `(12, 12)` two-sign-bits
invariant; two hand-pinned byte sequences (`[0x00]` for index 14
padded; `[0xff, 0xf0]` for index 168 packed); exact bit-consumption
invariant across every index; out-of-range and truncation rejections;
`HCOD10_MAX_LEN` constant consistency; Codebook-9-and-10-share-the-
LAV-12-unsigned-pair-universe cross-check confirming both rows
declare `unsigned, dim = 2, LAV = 12` but route to different
Annex 4.A tables; 169-pair bijection sweep that builds every legal
`(y, z)` pair, asserts `encode_tuple_to_index(10, …)` produces a
distinct index for each, and confirms `decode_index_to_tuple(10,
idx)` round-trips back to the same `(y, z)`; ceiling pull-down
invariant `HCOD9_MAX_LEN - HCOD10_MAX_LEN = 3`; head-displacement
vs Codebook 9 contrasted against Codebook 9's single-bit zero-tuple
slot). Codebook 11 (Table 4.A.12) reuses the same module shape and
will land in a future round; Codebook 11 is the **ESC** book — its
16 + ESC shape (Table 4.95 row 11: `unsigned = 1`, `dim = 2`,
`LAV = 16` with the §4.6.3 ESC-sequence reconstruction already landed
in round 213's `spectral_codebook` module) exercises the in-band
universe-extension path that Codebooks 1..=10 do not.

## Status (round 253)

Round 253 lands **Table 4.A.10** (Spectrum Huffman Codebook 9) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 / 234 / 238 / 241 / 244 / 250 bootstrapped for
Codebooks 1 through 8. Codebook 9 is the first **expanded-LAV
unsigned pair** spectrum book — Table 4.95 row 9 declares
`unsigned_cb = 1`, `dim = 2`, `LAV = 12`, so the §4.6.3.3 universe
shifts from Codebooks 7 / 8's shared `8 × 8 = 64`-entry `LAV = 7`
lattice to a `(12 + 1)^2 = 13^2 = 169`-entry lattice indexed
`0..=168` with each `(y, z)` coefficient in `0..=12`. The §4.6.3.3
unsigned polynomial `idx = y * (LAV + 1) + z = y * 13 + z` parks the
zero-tuple `(0, 0)` at index 0 with the single-bit `0` codeword —
matching the head placement Codebook 7 uses — and pins the maximum
tuple `(12, 12)` at index 168 (`12 * 13 + 12 = 168`) with a 15-bit
`0x7fff`. The maximum codeword length is **15 bits** — a 5-bit jump
over Codebook 8's 10-bit ceiling and the widest non-ESC codeword in
the entire Annex 4.A book set — reflecting the `169 / 64 ≈ 2.6×`
universe expansion that widens the distribution's tail. Exactly four
rows reach the 15-bit ceiling: indices 142 (`0x7ffc`), 154
(`0x7ffd`), 155 (`0x7ffe`), and 168 (`0x7fff`) — the rarest pair
magnitudes near the `LAV = 12` cap. The table is a **complete**
15-bit prefix code (Kraft equality `Σ 2^(15 − L) = 32768 = 2^15`),
exhaustively verified by walking every 15-bit prefix and asserting
each maps to exactly one entry. Because Codebook 9 is unsigned, the
§4.6.3.3 sign-bit suffix follows the Huffman codeword on the wire —
one sign bit per non-zero coefficient, delivered low-frequency-
first — and is exposed by the round-213 `apply_sign_bits` /
`derive_sign_bits` helpers (the suffix lives outside the Huffman
codeword carried by this module, an invariant verified across every
index by an explicit `derive_sign_bits(9, &tuple)` cross-check that
confirms a `(12, 12)` tuple emits two sign bits, a `(0, 0)` tuple
emits zero sign bits, and the suffix length matches the non-zero-
coefficient count for every index in between). Public API:
`HCOD9_NUM_ENTRIES = 169`, `HCOD9_MAX_LEN = 15`, `hcod9_encode(idx)
-> (u8, u16)`, `hcod9_decode(reader) -> u32`, and the convenience
`hcod9_write(writer, idx)`. Out-of-range indices surface as
`Error::SpectralCodebookIndexOutOfRange(9)`; reader underflow
surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised across every index: `decode_index_to_tuple
(9, idx)` → `encode_tuple_to_index(9, &tuple)` round-trips back to
the same index for the entire `13 × 13` unsigned pair lattice, and a
bijection cross-check builds the `169`-entry `HashSet` of legal
pairs and confirms every `(y, z)` maps to a distinct index. 16 new
unit tests (in `src/spectrum_huffman.rs`: 169-entry shape / 15-bit
max / 1-bit min at index 0 / codewords fit / Kraft equality 32768 /
complete-prefix walk over all `2^15` prefixes / per-row PDF spot
checks at indices 0 / 1 / 13 / 14 / 168 / four-15-bit-ceiling
enumeration at indices 142 / 154 / 155 / 168 / encode + write
rejection / single-zero-bit decode at index 0 / full 15-bit codeword
decode at index 168 / writer round-trip / unsigned-pair universe
expansion from Codebook 8 / shared head-placement with Codebook 7 /
5-bit ceiling delta vs Codebook 8) plus 25 new integration tests in
`tests/spectrum_huffman.rs` (eight per-row PDF spot checks at
indices 0 / 1 / 13 / 14 / 142 / 154 / 155 / 168; Table 4.95 row 9 ↔
Table 4.A.10 size cross-check; full writer→reader round-trip; the
§4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check; zero-tuple ↔
index-0 carries 1-bit `0` invariant; far-corner `(12, 12)` ↔
index-168 invariant; per-index sign-bit-count invariant against
`derive_sign_bits(9, …)`; zero-tuple zero-sign-bits invariant;
`(12, 12)` two-sign-bits invariant; two hand-pinned byte sequences
(`[0x00]` for index 0 padded; `[0xff, 0xfe]` for index 168 packed);
exact bit-consumption invariant across every index; out-of-range
and truncation rejections; `HCOD9_MAX_LEN` constant consistency;
Codebook-9-is-the-first-expanded-LAV-unsigned-pair-book cross-check
confirming Codebooks 7 / 8 / 9 all share the `unsigned`, `dim = 2`
columns but only Codebook 9 widens `LAV` from `7` to `12`; 169-pair
bijection sweep that builds every legal `(y, z)` pair, asserts
`encode_tuple_to_index(9, …)` produces a distinct index for each,
and confirms `decode_index_to_tuple(9, idx)` round-trips back to the
same `(y, z)`; ceiling-jump invariant `HCOD9_MAX_LEN -
HCOD8_MAX_LEN = 5`; head-placement parity with Codebook 7 contrasted
against Codebook 8's lifted zero-tuple). Codebooks 10..=11 (Tables
4.A.11 … 4.A.12) reuse the same module shape and will land one per
future round; Codebook 11 is the **ESC** book — its 16 + ESC shape
(Table 4.95 row 11: `unsigned = 1`, `dim = 2`, `LAV = 16` with the
§4.6.3 ESC-sequence reconstruction already landed in round 213's
`spectral_codebook` module) exercises the in-band universe-extension
path that Codebooks 1..=10 do not.

## Status (round 250)

Round 250 lands **Table 4.A.9** (Spectrum Huffman Codebook 8) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 / 234 / 238 / 241 / 244 bootstrapped for
Codebooks 1 through 7. Codebook 8 is the second **unsigned pair**
spectrum book — Table 4.95 row 8 declares the same `unsigned_cb = 1`,
`dim = 2`, `LAV = 7` shape as row 7 (Codebook 7), so both books share
the `(7 + 1)^2 = 8^2 = 64`-entry unsigned pair lattice indexed
`0..=63` with each `(y, z)` coefficient in `0..=7`. Where Codebook 7
pins the §4.6.3.3 zero-tuple `(0, 0)` at index 0 with the single-bit
`0` codeword and lets the upper-right quadrant of the lattice climb
to a 12-bit ceiling, Codebook 8 lifts the zero-tuple at index 0 to a
5-bit `0xe` and migrates the shortest codeword (3 bits `0b000`) to
**index 9** — the unsigned-polynomial position of the interior tuple
`(y, z) = (1, 1)` (`idx = 1 * 8 + 1 = 9`). The maximum codeword
length is **10 bits**; exactly four rows reach that ceiling:
indices 7 (`0x3fe`), 47 (`0x3fc`), 56 (`0x3fd`), and 63 (`0x3ff`) —
the rarest pair magnitudes (one or two coefficients at the `LAV`
cap). The flatter, lower-ceiling codeword distribution makes
Codebook 8 a better fit for sections whose magnitude statistics put
weight on the `(1, 1)` interior rather than the `(0, 0)` zero-tuple
corner Codebook 7 optimises. The table is a **complete** 10-bit
prefix code (Kraft equality `Σ 2^(10 − L) = 1024 = 2^10`),
exhaustively verified by walking every 10-bit prefix and asserting
each maps to exactly one entry. Because Codebook 8 is unsigned, the
§4.6.3.3 sign-bit suffix follows the Huffman codeword on the wire —
one sign bit per non-zero coefficient, delivered low-frequency-
first — and is exposed by the round-213 `apply_sign_bits` /
`derive_sign_bits` helpers (the suffix lives outside the Huffman
codeword carried by this module, an invariant verified across every
index by an explicit `derive_sign_bits(8, &tuple)` cross-check that
confirms a `(7, 7)` tuple emits two sign bits, a `(0, 0)` tuple emits
zero sign bits, and the suffix length matches the non-zero-
coefficient count for every index in between). Public API:
`HCOD8_NUM_ENTRIES = 64`, `HCOD8_MAX_LEN = 10`, `hcod8_encode(idx)
-> (u8, u16)`, `hcod8_decode(reader) -> u32`, and the convenience
`hcod8_write(writer, idx)`. Out-of-range indices surface as
`Error::SpectralCodebookIndexOutOfRange(8)`; reader underflow
surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised across every index: `decode_index_to_tuple
(8, idx)` → `encode_tuple_to_index(8, &tuple)` round-trips back to
the same index for the entire `8 × 8` unsigned pair lattice, and a
bijection cross-check builds the `64`-entry `HashSet` of legal pairs
and confirms every `(y, z)` maps to a distinct index. 17 new unit
tests (in `src/spectrum_huffman.rs`: 64-entry shape / 10-bit max /
3-bit min at index 9 / codewords fit / Kraft equality 1024 /
complete-prefix walk over all `2^10` prefixes / per-row PDF spot
checks at indices 0 / 8 / 9 / 63 / four-10-bit-ceiling enumeration
at indices 7 / 47 / 56 / 63 / encode + write rejection / three-zero-
bits decode at index 9 / full 10-bit codeword decode at index 63 /
writer round-trip / Codebooks 7 + 8 shortest-slot disagreement /
Codebooks 7 + 8 shared far-corner placement) plus 25 new integration
tests in `tests/spectrum_huffman.rs` (eight per-row PDF spot checks
at indices 0 / 1 / 7 / 8 / 9 / 40 / 55 / 63; Table 4.95 row 8 ↔
Table 4.A.9 size cross-check; full writer→reader round-trip; the
§4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check; zero-tuple ↔
index-0 carries 5-bit `0xe` invariant; interior `(1, 1)` ↔ index-9
holds the 3-bit shortest codeword invariant; far-corner ↔ index-63
invariant; per-index sign-bit-count invariant against
`derive_sign_bits(8, …)`; zero-tuple zero-sign-bits invariant;
`(7, 7)` two-sign-bits invariant; three hand-pinned byte sequences
(`[0x00]` for index 9 padded; `[0xff, 0xc0]` for index 63 padded;
`[0x06, 0xe0]` for indices 9 + 8 + 0 packed); exact bit-consumption
invariant across every index; out-of-range and truncation rejections;
`HCOD8_MAX_LEN` constant consistency; Codebook 8-is-the-second-
unsigned-pair-book cross-check confirming Codebooks 7 and 8 share
the same `(unsigned, dim, LAV)` columns but pick distinct table
pointers; zero-tuple-codeword disagreement between Codebooks 7 and
8; shared `(7, 7)` far-corner index placement; and a 64-pair
bijection sweep that builds every legal `(y, z)` pair, asserts
`encode_tuple_to_index(8, …)` produces a distinct index for each,
and confirms `decode_index_to_tuple(8, idx)` round-trips back to the
same `(y, z)`). Suite grows 812 → 859 tests. Codebooks 9..=11
(Tables 4.A.10 … 4.A.12) reuse the same module shape and will land
one per future round; Codebook 9 is the first **expanded-LAV pair**
book (Table 4.95 row 9: `unsigned = 1`, `dim = 2`, `LAV = 12` → 169
entries) and is the largest of the non-ESC books, exercising the
§4.6.3.3 universe expansion to a `13 × 13` lattice without ESC
extension.

## Status (round 244)

Round 244 lands **Table 4.A.8** (Spectrum Huffman Codebook 7) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 / 234 / 238 / 241 bootstrapped for Codebooks 1
through 6. Codebook 7 is the first **unsigned pair** spectrum book —
Table 4.95 row 7 declares `unsigned_cb = 1`, `dim = 2`, `LAV = 7`, so
the §4.6.3.3 universe shifts to `(7 + 1)^2 = 8^2 = 64` entries indexed
`0..=63` with each `(y, z)` coefficient in `0..=7`. That is a notable
shape change from the 81-entry universe shared by Codebooks 1..=6:
Codebooks 1..=4 used dim-4 with `LAV ∈ {1, 2}` (`3^4 = 81`); Codebooks
5 and 6 used dim-2 with `LAV = 4` (`(2 * 4 + 1)^2 = 9^2 = 81`); Codebook
7 swaps signedness for range, narrowing the universe to 64 entries
while widening the per-coefficient ceiling from `4` to `7`. The
§4.6.3.3 unsigned polynomial `idx = y * (LAV + 1) + z = y * 8 + z`
parks the zero-tuple `(0, 0)` at index 0 (the origin of the unsigned
dim-2 lattice) and pins the far corner `(7, 7)` to index 63 — the same
"zero-tuple at index 0 with the single-bit `0` codeword" head
placement Codebook 3 uses for its unsigned dim-4 universe. The maximum
codeword length is **12 bits**; exactly four rows reach that ceiling:
indices 54 (`0xffd`), 55 (`0xffe`), 62 (`0xffc`), and 63 (`0xfff`) —
the upper-right `{(6, 7), (6, 7+), (7, 6), (7, 7)}` quadrant of the
`8 × 8` unsigned pair lattice (the rarest pair magnitudes). The table
is a **complete** 12-bit prefix code (Kraft equality
`Σ 2^(12 − L) = 4096 = 2^12`), exhaustively verified by walking every
12-bit prefix and asserting each maps to exactly one entry. Because
Codebook 7 is unsigned, the §4.6.3.3 sign-bit suffix follows the
Huffman codeword on the wire — one sign bit per non-zero coefficient,
delivered low-frequency-first — and is exposed by the round-213
`apply_sign_bits` / `derive_sign_bits` helpers (the suffix lives
outside the Huffman codeword carried by this module, an invariant
verified across every index by an explicit `derive_sign_bits(7,
&tuple)` cross-check that confirms a `(7, 7)` tuple emits two sign
bits, a `(0, 0)` tuple emits zero sign bits, and the suffix length
matches the non-zero-coefficient count for every index in between).
Public API: `HCOD7_NUM_ENTRIES = 64`, `HCOD7_MAX_LEN = 12`,
`hcod7_encode(idx) -> (u8, u16)`, `hcod7_decode(reader) -> u32`, and
the convenience `hcod7_write(writer, idx)`. Out-of-range indices
surface as `Error::SpectralCodebookIndexOutOfRange(7)`; reader
underflow surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised across every index: `decode_index_to_tuple(7,
idx)` → `encode_tuple_to_index(7, &tuple)` round-trips back to the
same index for the entire `8 × 8` unsigned pair lattice, and a
bijection cross-check builds the `64`-entry `HashSet` of legal pairs
and confirms every `(y, z)` maps to a distinct index. 16 new unit
tests (in `src/spectrum_huffman.rs`: 64-entry shape / 12-bit max /
1-bit min at index 0 / codewords fit / Kraft equality 4096 /
complete-prefix walk over all `2^12` prefixes / per-row PDF spot
checks at indices 0 / 8 / 63 / four-12-bit-ceiling enumeration at
indices 54 / 55 / 62 / 63 / encode + write rejection / single-zero-bit
decode at index 0 / full 12-bit codeword decode at index 63 / writer
round-trip / writer rejection / Codebook 3 vs Codebook 7 zero-tuple
placement agreement / Codebook 7 entry-count = 64 vs Codebook 3/4 = 81
shape contrast) plus 24 new integration tests in
`tests/spectrum_huffman.rs` (eight per-row PDF spot checks at indices
0 / 1 / 8 / 9 / 16 / 40 / 55 / 63; Table 4.95 row 7 ↔ Table 4.A.8 size
cross-check; full writer→reader round-trip; the §4.6.3.3 wire-index ↔
tuple ↔ wire-index cross-check; zero-tuple ↔ index-0 invariant;
far-corner ↔ index-63 invariant; per-index sign-bit-count invariant
against `derive_sign_bits(7, …)`; zero-tuple zero-sign-bits invariant;
`(7, 7)` two-sign-bits invariant; three hand-pinned byte sequences
(`[0x00]` for index 0 padded; `[0xff, 0xf0]` for index 63 padded;
`[0x4c]` for indices 0 + 8 + 9 packed); exact bit-consumption
invariant across every index; out-of-range and truncation rejections;
`HCOD7_MAX_LEN` constant consistency; Codebook 7-is-the-first-unsigned-
pair-book cross-check confirming Codebooks 6 and 7 share dim-2 but pick
distinct signedness / LAV columns; shared-zero-tuple-placement cross-
check confirming Codebooks 3 and 7 both park the zero-tuple at index 0
with a single-bit `0` codeword; and a 64-pair bijection sweep that
builds every legal `(y, z)` pair, asserts `encode_tuple_to_index(7, …)`
produces a distinct index for each, and confirms `decode_index_to_tuple
(7, idx)` round-trips back to the same `(y, z)`). Suite grows 768 →
808 tests. Codebooks 8..=11 (Tables 4.A.9 … 4.A.12) reuse the same
module shape and will land one per future round; Codebook 8 is the
second **unsigned pair** book (Table 4.95 row 8: `unsigned = 1`,
`dim = 2`, `LAV = 7` → again `64` entries) sharing Codebook 7's
shape but with a different Huffman-length distribution column.

## Status (round 241)

Round 241 lands **Table 4.A.7** (Spectrum Huffman Codebook 6) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 / 234 / 238 bootstrapped for Codebooks 1
through 5. Codebook 6 is the second **signed pair** spectrum book —
Table 4.95 row 6 declares the same `unsigned_cb = 0`, `dim = 2`,
`LAV = 4` shape as row 5 (Codebook 5), so both books share the
`(2 * 4 + 1)^2 = 9^2 = 81`-entry signed pair lattice indexed
`0..=80` with each `(y, z)` coefficient in `-4..=+4`. Where Codebook
5 parks the single-bit `0` codeword at index 40 (the §4.6.3.3
zero-tuple `(0, 0)`) and lets the four lattice corners stretch out
to a 13-bit ceiling, Codebook 6 lifts the zero-tuple to a 4-bit
`0b0000` and pulls the maximum codeword length back to **11 bits**.
Exactly four rows occupy that 11-bit ceiling: indices 0 (`0x7fe`),
8 (`0x7fd`), 72 (`0x7ff`), and 80 (`0x7fc`) — the same four
`(±4, ±4)` lattice-corner positions Codebook 5 also pinned to its
ceiling, confirming both books reserve their longest codewords for
the rarest pair magnitudes. The shorter, flatter codeword
distribution makes Codebook 6 a better fit for sections whose
magnitude statistics put more weight in the `(±1, ±1) .. (±3, ±3)`
interior than Codebook 5's zero-tuple-heavy target. Because Codebook
6 is signed, no §4.6.3.3 sign-bit suffix follows the codeword on
the wire — every coefficient's sign is already baked into the index
via the `offset = LAV = 4` shift, an invariant verified across every
index by an explicit `derive_sign_bits(6, &tuple)` cross-check. The
table is a **complete** 11-bit prefix code (Kraft equality
`Σ 2^(11 − L) = 2048 = 2^11`), exhaustively verified by walking
every 11-bit prefix and asserting each maps to exactly one entry.
Public API: `HCOD6_NUM_ENTRIES = 81`, `HCOD6_MAX_LEN = 11`,
`hcod6_encode(idx) -> (u8, u16)`, `hcod6_decode(reader) -> u32`, and
the convenience `hcod6_write(writer, idx)`. Out-of-range indices
surface as `Error::SpectralCodebookIndexOutOfRange(6)`; reader
underflow surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised across every index: `decode_index_to_tuple(6,
idx)` → `encode_tuple_to_index(6, &tuple)` round-trips back to the
same index for the entire `9 × 9` signed pair lattice. 16 new unit
tests (in `src/spectrum_huffman.rs`: 81-entry shape / 11-bit max /
4-bit min at index 40 / codewords fit / Kraft equality 2048 /
complete-prefix walk over all `2^11` prefixes / per-row PDF spot
checks at indices 0 / 40 / 80 / four-11-bit-corner enumeration /
encode + decode rejection / four-zero-bits decode at index 40 / full
11-bit codeword decode at index 72 / writer round-trip / writer
rejection / Codebook 5 vs Codebook 6 zero-tuple disagreement at
index 40 / shared lattice-corner index positions) plus 24 new
integration tests in `tests/spectrum_huffman.rs` (eight per-row PDF
spot checks at indices 0 / 8 / 13 / 31 / 40 / 41 / 72 / 80; Table
4.95 row 6 ↔ Table 4.A.7 size cross-check; full writer→reader
round-trip; the §4.6.3.3 wire-index ↔ tuple ↔ wire-index
cross-check; zero-tuple ↔ index-40 invariant; per-index
zero-sign-bit invariant against `derive_sign_bits(6, …)`; three
hand-pinned byte sequences (`[0x00]` for index 40 padded;
`[0xff, 0xe0]` for index 72 padded; `[0x03]` for indices 40 + 41
packed); exact bit-consumption invariant across every index;
out-of-range and truncation rejections; `HCOD6_MAX_LEN` constant
consistency; Codebook 6-is-the-second-signed-pair-book cross-check
confirming Codebooks 5 and 6 share the signed-pair Table 4.95 row
but pick a different `huffman_table` column; and a shared
lattice-corner-positions cross-check confirming both books pin the
four `(±4, ±4)` corners to their respective maximum-length
codewords at indices 0 / 8 / 72 / 80). Suite grows 728 → 768
tests. Codebooks 7..=11 (Tables 4.A.8 … 4.A.12) reuse the same
module shape and will land one per future round; Codebook 7 is the
first **unsigned** pair book (Table 4.95 row 7: `unsigned = 1`,
`dim = 2`, `LAV = 7` → `(7 + 1)^2 = 64` entries) — a shape change
from the 81-entry universe shared by Codebooks 1..=6 because
unsigned pair books use `modulus = LAV + 1` rather than the signed
`2 * LAV + 1`.

## Status (round 238)

Round 238 lands **Table 4.A.6** (Spectrum Huffman Codebook 5) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 / 234 bootstrapped for Codebooks 1, 2, 3, and 4.
Codebook 5 is the first **pair** spectrum book — Table 4.95 row 5
declares `unsigned_cb = 0`, `dim = 2`, `LAV = 4` so each Huffman
codeword conveys a signed pair `(y, z)` with each coefficient in
`-4..=+4` rather than the dim-4 magnitude tuple of Codebooks 1..=4.
The pair universe stays at 81 entries because `(2 * 4 + 1)^2 = 9^2 = 81`
coincides with the dim-4 LAV-1 / LAV-2 universes of the earlier
books. Index 40 carries the §4.6.3.3 zero-tuple `(0, 0)` — the
`(modulus = 9, offset = LAV = 4)` polynomial evaluation puts the
origin at the centre of the index range, not at the edges as in the
unsigned Codebooks 3 and 4 (which placed `(0, 0, 0, 0)` at index 0)
— and the shortest codeword (1 bit `0`) parks at index 40 (the same
zero-tuple position as Codebook 1, since both books share the signed
polynomial offset). The maximum codeword length is **13 bits** — one
more than Codebook 4's 12-bit ceiling and three less than Codebook
3's 16-bit reach — and exactly four rows occupy the 13-bit ceiling:
indices 0 (`0x1fff`), 8 (`0x1ffd`), 72 (`0x1ffc`), and 80 (`0x1ffe`)
— the four `(±4, ±4)` corners of the signed `9 × 9` pair lattice.
Because Codebook 5 is signed, no §4.6.3.3 sign-bit suffix follows the
codeword on the wire — every coefficient's sign is already baked
into the index via the `offset = 4` shift, an invariant verified
across every index by an explicit `derive_sign_bits(5, &tuple)`
cross-check. The table is a **complete** 13-bit prefix code (Kraft
equality `Σ 2^(13 − L) = 8192 = 2^13`), exhaustively verified by
walking every 13-bit prefix and asserting each maps to exactly one
entry. Public API: `HCOD5_NUM_ENTRIES = 81`, `HCOD5_MAX_LEN = 13`,
`hcod5_encode(idx) -> (u8, u16)`, `hcod5_decode(reader) -> u32`, and
the convenience `hcod5_write(writer, idx)`. Out-of-range indices
surface as `Error::SpectralCodebookIndexOutOfRange(5)`; reader
underflow surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised across every index: `decode_index_to_tuple(5,
idx)` → `encode_tuple_to_index(5, &tuple)` round-trips back to the
same index for the entire `9 × 9` signed pair lattice. 16 new unit
tests (in `src/spectrum_huffman.rs`: 81-entry shape / 13-bit max /
1-bit min at index 40 / codewords fit / Kraft equality 8192 /
complete-prefix walk over all `2^13` prefixes / per-row PDF spot
checks at indices 0 / 40 / 80 / four-13-bit-corner enumeration /
encode + decode rejection / single-zero-bit decode at index 40 / full
13-bit codeword decode at index 0 / writer round-trip / writer
rejection) plus 21 new integration tests in
`tests/spectrum_huffman.rs` (eight per-row PDF spot checks at indices
0 / 8 / 13 / 31 / 40 / 41 / 72 / 80; Table 4.95 row 5 ↔ Table 4.A.6
size cross-check; full writer→reader round-trip; the §4.6.3.3
wire-index ↔ tuple ↔ wire-index cross-check; zero-tuple ↔ index-40
invariant plus four-corner `(±4, ±4)` invariant; per-index zero-sign-
bit invariant against `derive_sign_bits(5, …)`; three hand-pinned
byte sequences (`[0x00]` for index 40 padded; `[0xff, 0xf8]` for
index 0 padded; `[0x50]` for index 40 + index 41 packed); exact bit-
consumption invariant across every index; out-of-range and truncation
rejections; `HCOD5_MAX_LEN` constant consistency; and a Codebook
5-is-the-first-pair-book cross-check confirming Codebooks 1..=4 are
all dim-4 while Codebook 5 is dim-2 but all five books happen to share
the 81-entry universe). Suite grows 691 → 728 tests. Codebooks 6..=11
(Tables 4.A.7 … 4.A.12) reuse the same module shape and will land
one per future round; Codebook 6 is the second signed pair
(`unsigned = 0`, `dim = 2`, `LAV = 4`) — the same Table 4.95 row
shape as Codebook 5.

## Status (round 234)

Round 234 lands **Table 4.A.5** (Spectrum Huffman Codebook 4) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 / 231 bootstrapped for Codebooks 1, 2, and 3.
Codebook 4 shares Codebook 3's unsigned dim-4 LAV-2 tuple universe
(Table 4.95 row 4 is identical to row 3 except for the source-table
column: `unsigned_cb = 1`, `dim = 4`, `LAV = 2` → `3^4 = 81` entries
indexed `0..=80`, each tuple coefficient in `0..=2`, with the §4.6.3.3
sign-bit suffix carrying the sign of every non-zero coefficient
outside the Huffman codeword) but uses a different per-row Huffman
length tuning. Where Codebook 3 puts the zero-tuple at index 0 with a
single-bit codeword and lets the magnitude-2 tuples climb to a 16-bit
maximum, Codebook 4 parks the **shortest codeword** (4 bits `0b0000`)
at **index 40** while lifting index 0 (still the §4.6.3.3
zero-tuple `(0, 0, 0, 0)`) to a 4-bit `0b0111`. The maximum codeword
length is **12 bits** (vs 16 for Codebook 3), and exactly two rows
reach that length: index 62 (`0xfff`) and index 74 (`0xffe`). The
shorter, flatter distribution makes Codebook 4 a better fit for
sections whose magnitude statistics are uniformly spread across the
`(0, 0, 0, 0) .. (2, 2, 2, 2)` range than Codebook 3's zero-heavy
target. The table is a **complete** 12-bit prefix code (Kraft equality
`Σ 2^(12 − L) = 4096 = 2^12`), exhaustively verified by walking every
12-bit prefix and asserting each maps to exactly one entry. Public
API: `HCOD4_NUM_ENTRIES = 81`, `HCOD4_MAX_LEN = 12`,
`hcod4_encode(idx) -> (u8, u16)`, `hcod4_decode(reader) -> u32`, and
the convenience `hcod4_write(writer, idx)`. Out-of-range indices
surface as `Error::SpectralCodebookIndexOutOfRange(4)`; reader
underflow surfaces as `Error::UnexpectedEnd`. The round-213 §4.6.3.3
translation is exercised as a cross-check: every Codebook 4 index
round-trips through `decode_index_to_tuple(4, idx)` →
`encode_tuple_to_index(4, &tuple)` back to the same index; the
§4.6.3.3 sign-bit count (`derive_sign_bits(4, &tuple)`) matches the
non-zero-coefficient count for every index; and a Codebook 3 ↔
Codebook 4 cross-check confirms that the two books map every index
to the *same* `(w, x, y, z)` magnitude tuple — only the codeword
assignment differs, since the §4.6.3.3 translation depends on the
Table 4.95 row shape (which is identical for rows 3 and 4) and not on
the codeword bit pattern. 16 new unit tests (in
`src/spectrum_huffman.rs`: 81-entry shape / 12-bit max / 4-bit min at
index 40 / codewords fit / Kraft equality 4096 / complete-prefix walk
over all `2^12` prefixes / per-row PDF spot checks at indices 0 / 40 /
62 / 74 / 80 / encode + decode rejection / four-zero-bit decode at
index 40 / full 12-bit codeword decode at index 62 / writer
round-trip / writer rejection / Codebook 3 ↔ Codebook 4
zero-tuple-codeword disagreement) plus 24 new integration tests in
`tests/spectrum_huffman.rs` (seven per-row PDF spot checks at
indices 0 / 13 / 27 / 40 / 62 / 74 / 80; Table 4.95 row 4 ↔ Table
4.A.5 size cross-check; full writer→reader round-trip; the §4.6.3.3
wire-index ↔ tuple ↔ wire-index cross-check; Codebook 3 / Codebook 4
tuple-universe equivalence cross-check; zero-tuple ↔ index-0 invariant;
full magnitude tuple ↔ index 80 invariant; zero-sign-bit /
four-sign-bit / per-index-sign-count cross-checks against
`derive_sign_bits`; three hand-pinned byte sequences (`[0x00]` for
index 40 padded; `[0xff, 0xf0]` for index 62 padded; `[0x07]` for
index 40 + index 0 packed); exact bit-consumption invariant across
every index; out-of-range and truncation rejections; `HCOD4_MAX_LEN`
constant consistency; and a Codebook 3 / Codebook 4 max-codeword-length
cross-check). Suite grows 651 → 691 tests. Codebooks 5..=11
(Tables 4.A.6 … 4.A.12) reuse the same module shape and will land
one per future round; Codebooks 5 and 6 are the first **pair**
(`dim = 2`) books in Table 4.95 with `LAV = 4`, opening a new
codebook geometry (`9^2 = 81` entries per book).

## Status (round 231)

Round 231 lands **Table 4.A.4** (Spectrum Huffman Codebook 3) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module that
rounds 219 / 226 bootstrapped for Codebooks 1 and 2. Codebook 3 is the
first **unsigned** spectrum book: Table 4.95 row 3 declares
`unsigned_cb = 1`, `dim = 4`, `LAV = 2` so the Huffman codeword now
conveys magnitude n-tuples whose every coefficient lies in `0..=2`
(rather than the signed `(-1, 0, +1)` of Codebooks 1 and 2), and the
§4.6.3.3 sign-bit suffix carries the sign of each non-zero coefficient
outside the codeword. The index space is still `3^4 = 81` (the
`(LAV + 1)^dim` polynomial for unsigned books coincides with the
`(2*LAV + 1)^dim` of the `LAV = 1` signed books) but the zero-tuple
`(0, 0, 0, 0)` now lives at **index 0** (not 40) because the unsigned
polynomial puts all-zero at the origin; it carries the single bit
codeword `0`. The maximum codeword length is **16 bits** (vs 11 for
Codebook 1 and 9 for Codebook 2) — the longer codes are a direct
consequence of unsigned Codebook 3's smaller-effective-alphabet target
statistics, where the magnitude-2 tuples deep in the table earn their
length budget back via the sign bits' presence-conditioned emission.
Two distinct rows carry the full 16-bit codewords: index 62
(`0xffff`) and index 74 (`0xfffe`). The table is a **complete** 16-bit
prefix code (Kraft equality `Σ 2^(16 − L) = 65536 = 2^16`),
exhaustively verified by walking every 16-bit prefix and asserting
each maps to exactly one entry. Public API: `HCOD3_NUM_ENTRIES = 81`,
`HCOD3_MAX_LEN = 16`, `hcod3_encode(idx) -> (u8, u16)`,
`hcod3_decode(reader) -> u32`, and the convenience
`hcod3_write(writer, idx)`. Out-of-range indices surface as
`Error::SpectralCodebookIndexOutOfRange(3)`; reader underflow surfaces
as `Error::UnexpectedEnd`. The round-213 §4.6.3.3 translation is
exercised as a cross-check: every Codebook 3 index round-trips through
`decode_index_to_tuple(3, idx)` → `encode_tuple_to_index(3, &tuple)`
back to the same index; the §4.6.3.3 sign-bit count
(`derive_sign_bits(3, &tuple)`) matches the non-zero-coefficient count
for every index; and an explicit cross-check confirms Codebook 3 has
a *disjoint* tuple universe from Codebooks 1 and 2 (a negative-entry
tuple cannot encode under Codebook 3; a magnitude-2 tuple cannot
encode under Codebook 1). 17 new unit tests (in
`src/spectrum_huffman.rs`: 81-entry shape / 16-bit max / 1-bit min at
index 0 / codewords fit / Kraft equality 65536 / complete-prefix walk
over all `2^16` prefixes / per-row PDF spot checks at indices 0 / 62 /
80 / encode + decode rejection / single-zero-bit decode / full 16-bit
codeword decode / writer round-trip / writer rejection / cross-book
zero-tuple-position contrast) plus 21 new integration tests in
`tests/spectrum_huffman.rs` (seven per-row PDF spot checks at indices
0 / 1 / 27 / 40 / 62 / 74 / 80; Table 4.95 row 3 ↔ Table 4.A.4 size
cross-check; full writer→reader round-trip; the §4.6.3.3 wire-index ↔
tuple ↔ wire-index cross-check; zero-tuple ↔ index-0 invariant; full
magnitude tuple ↔ index 80 invariant; zero-sign-bit / four-sign-bit /
per-index-sign-count cross-checks against `derive_sign_bits`;
three hand-pinned byte sequences (`[0x00]` for index 0 padded;
`[0xff, 0xff]` for index 62; `[0x48]` for index 0 + index 1 packed);
exact bit-consumption invariant across every index; out-of-range and
truncation rejections; `HCOD3_MAX_LEN` constant consistency;
cross-book tuple-universe disjointness). Suite grows 613 → 651 tests.
Codebooks 4..=11 (Tables 4.A.5 … 4.A.12) reuse the same module shape
and will land one per future round; the `spectral_data()` driver that
dispatches per-band onto the chosen codebook arrives once all eleven
spectrum books are in place.

Round 226 lands **Table 4.A.3** (Spectrum Huffman Codebook 2) inside
the existing [`spectrum_huffman`](src/spectrum_huffman.rs) module
that round 219 bootstrapped against Table 4.A.2. Codebook 2 shares
Codebook 1's signed 4-tuple universe (`unsigned_cb = 0`, `dim = 4`,
`LAV = 1` → `3^4 = 81` entries indexed `0..=80`) but uses a different
per-row Huffman length tuning to fit a different encoder
target-statistics: maximum codeword length is **9 bits** (vs 11 for
Codebook 1); the zero-tuple at index 40 carries the **3-bit
codeword `0b000`** (vs the single bit `0` in Codebook 1); the
shortest non-zero-tuple codeword is **4 bits** at index 67 (`0b0010`,
the spec-PDF's most likely non-zero 4-tuple for this book's target
statistics). The table is a **complete** 9-bit prefix code (Kraft
equality `Σ 2^(9 − L) = 512 = 2^9`), exhaustively verified by
walking every 9-bit prefix and asserting each maps to exactly one
entry. Public API: `HCOD2_NUM_ENTRIES = 81`, `HCOD2_MAX_LEN = 9`,
`hcod2_encode(idx) -> (u8, u16)`, `hcod2_decode(reader) -> u32`, and
the convenience `hcod2_write(writer, idx)`. Out-of-range indices
surface as `Error::SpectralCodebookIndexOutOfRange(2)`; reader
underflow surfaces as `Error::UnexpectedEnd`. The round-213
§4.6.3.3 translation is exercised as a cross-check: every Codebook 2
index round-trips through `decode_index_to_tuple(2, idx)` →
`encode_tuple_to_index(2, &tuple)` back to the same index, and an
explicit tuple-equivalence test confirms Codebook 1 and Codebook 2
map every shared index to the same `(w, x, y, z)` spectral tuple —
only the Huffman codewords differ, since the §4.6.3.3 translation
depends on the Table 4.95 row shape (identical for both books) and
not on the codeword assignment. 33 new tests (16 unit +
17 integration); suite grows 580 → 613 tests. Codebooks 3..=11
(Tables 4.A.4 … 4.A.12) reuse the same module shape and will land
one per future round; the `spectral_data()` driver that dispatches
per-band onto the chosen codebook arrives once all eleven spectrum
books are in place.

Round 219 had landed the bootstrap of `spectrum_huffman` — the
**wire layer** for the §4.6.3 / Annex 4.A Huffman codebooks that the
round-213 [`spectral_codebook`](src/spectral_codebook.rs) §4.6.3.3
index↔tuple translation already consumes — by transcribing
**Table 4.A.2** (Spectrum Huffman Codebook 1, signed 4-tuple,
`LAV = 1`, 81 entries indexed `0..=80`) verbatim from ISO/IEC
14496-3:2001(E) §4.A.1 (page 193). Each entry stores
`(length, codeword)` with the codeword right-aligned in a `u16`
(MSB at bit `length − 1`); maximum codeword length is 11 bits; the
zero-tuple `(0, 0, 0, 0)` at index 40 carries the single bit `0`.
The codebook is a **complete** prefix code (Kraft equality
`Σ 2^(11 − L) = 2048 = 2^11`), exhaustively verified by walking
every 11-bit prefix and asserting each maps to exactly one entry.
Public API: `HCOD1_NUM_ENTRIES = 81`, `HCOD1_MAX_LEN = 11`,
`hcod1_encode(idx) -> (u8, u16)`, `hcod1_decode(reader) -> u32`,
and the convenience `hcod1_write(writer, idx)`. Out-of-range
indices surface as `Error::SpectralCodebookIndexOutOfRange(1)`;
reader underflow surfaces as `Error::UnexpectedEnd`. The §4.6.3.3
translation is exercised as a cross-check: every Codebook 1 index
round-trips through `decode_index_to_tuple(1, idx)` →
`encode_tuple_to_index(1, &tuple)` back to the same index, with
every tuple element verified to lie in the `±LAV = ±1` range.

**Phase 1 complete + Phase 2 in progress + seven tool-level encoder
primitives + the §4.6.2.3.2 / §4.6.8.1.4 / §4.6.13 DPCM accumulator
pair + the §4.4.2.1 `raw_data_block()` frame assembler + the §4.4.2.7
`extension_payload()` body parser/writer for EXT_FILL / EXT_FILL_DATA
/ EXT_DYNAMIC_RANGE (Table 4.52 `dynamic_range_info()` + Table 4.53
`excluded_channels()`) + the §4.4.1.1
`program_config_element()` writer + the §4.4.1 / Table 4.1
`extensionFlag` body and the Table 1.15 `epConfig` field for every
General Audio ER object type + the §4.4.6.5 / Table 4.12
`gain_control_data()` SSR tool + the §1.6.5 / Table 1.15 trailing
`syncExtensionType == 0x2b7` implicit-SBR / PS probe + the §4.5.4.1
`swb_offset_long_window[]` / `swb_offset_short_window[]` lookup
tables and the §4.6.13 pulse-escape reconstruction loop + the
§4.6.9.4 `TNS_MAX_ORDER` / `TNS_MAX_BANDS` clamp tables and
§4.6.17.2.5 LD-specific `TNS_MAX_BANDS` tables + the §4.4.6 /
Table 4.50 `individual_channel_stream()` body walker + the
§4.6.3 / Table 4.95 Spectrum Huffman codebook parameters and the
§4.6.3.3 index → tuple translation (including the sign-bit fix-up
and the codebook-11 ESC sequence) + the §4.A.1 Tables 4.A.2 / 4.A.3 /
4.A.4 Spectrum Huffman Codebook 1 (signed, 11-bit max) / 2 (signed,
9-bit max) / 3 (unsigned, 16-bit max) wire-layer encode / decode
primitives.** Round 213 lands the
`spectral_codebook` module — the foundational decoder layer for
the upcoming `spectral_data()` parser. `TABLE_4_95: [Table495Row;
32]` covers every codebook number `0..=31` row-by-row: the
ZERO_HCB row (no dim/lav), the QUAD spectrum books 1..=4 (signed
LAV 1 / unsigned LAV 2), the PAIR spectrum books 5..=10 (signed
LAV 4 / unsigned LAV 7 / unsigned LAV 12), the ESC book 11
(unsigned LAV 16 + ESC threshold 8191), the four non-spectral
slots 12..=15 (reserved / PNS / intensity stereo), and the
sixteen extension books 16..=31 with their per-row ESC thresholds
(15, 31, 47, 63, 95, 127, 159, 191, 223, 255, 319, 383, 511, 767,
1023, 2047). `decode_index_to_tuple(cb, idx)` implements the
§4.6.3.3 pseudocode (`mod = lav + 1` unsigned or `2*lav + 1`
signed; `dim == 4` slice via `mod^3 / mod^2 / mod^1`); the
inverse `encode_tuple_to_index(cb, tuple)` round-trips every
tuple in 3^4 / 9^2 / 8^2 cells under integration tests. The
sign-bit fix-up `apply_sign_bits` / `derive_sign_bits` covers the
unsigned-codebook low-frequency-first sign sequence; the ESC
sequence `decode_esc_value` / `encode_esc_value` covers the
§4.6.3.3 `2^(N+4) + escape_word` magnitude expansion for codebook
11 (and the extension books). Public `MAX_QUANT = 8191`
(§4.6.1.3 maximum absolute amplitude for `x_quant`); per-row
helpers `is_unsigned` and `has_esc`; `Error` extends with
`SpectralCodebookOutOfRange`, `SpectralCodebookHasNoTuple`,
`SpectralCodebookIndexOutOfRange`,
`SpectralCodebookTupleOutOfRange`,
`SpectralCodebookSignBitsMismatch`, and
`SpectralCodebookEscOutOfRange`. 46 new integration tests in
`tests/spectral_codebook.rs` (row-by-row Table 4.95 layout,
§4.6.3.3 round-trip every legal tuple for codebooks 1 / 3 / 5 /
7, every legal index for codebook 11, ESC round-trip every
prefix length `0..=8`, every boundary at the LAV cap and at
MAX_QUANT, and every rejection branch). Suite grows 503 → 549
tests. Round 207
lands the channel-element body walker that composes the existing
per-tool parsers / writers (`global_gain`, `ics_info`,
`section_data`, `scale_factor_data`, optional `pulse_data` /
`tns_data` / `gain_control_data`) into a single
`individual_channel_stream()` parse / write cycle, **up to but not
including** `spectral_data()`. The new `ics_body` module exposes
`IcsBody` with the four entry points
`IcsBody::parse(reader, audio_object_type, sampling_frequency_index,
scale_flag)` (SCE / LFE / non-shared CPE form — reads inline
`ics_info()`),
`IcsBody::parse_with_ics_info(reader, &ics_info,
audio_object_type, scale_flag)` (CPE `common_window == 1`
shared-info form), and the symmetric writers
`IcsBody::write(...)` / `IcsBody::write_with_ics_info(...)`. The
parsed struct surfaces every wire field plus the
`spectral_data_bit_offset` (the bit position of the first
`spectral_data()` bit relative to the start of the body) so the
caller can hand off the trailing spectrum block to a future
parser. The Table 4.50 Note 1 "pulse_data is illegal on
`EIGHT_SHORT_SEQUENCE`" constraint and the §4.6.12
"gain_control_data is AOT-3 (SSR) only" normative constraint are
enforced on the writer side
(`Error::PulseDataEncodeInvalid` / `Error::GainControlDataEncodeInvalid`);
the parser surfaces the literal bits so hostile streams that violate
the constraints do not panic. `scale_flag == true` (scalable AAC,
AOT 6) is rejected on both sides with `Error::NotImplemented`
because the scalable extension's `aac_scalable_main_header()`
dispatch is not yet wired up. New public constants
`GLOBAL_GAIN_BITS = 8` and `AOT_AAC_SSR = 3`. 15 new integration
tests in `tests/ics_body.rs` cover: the minimal AAC-LC long body
(no tools), the one-active-band variant, every dispatch branch
(`pulse_data` / `tns_data` / `gain_control_data` independently
and jointly), the AAC-LC eight-short tns_data round-trip, the
`EIGHT_SHORT_SEQUENCE` pulse-data rejection, the populated-slot-
without-dispatch-bit rejection, the AOT-3-only gain_control_data
rejection, the CPE shared-`ics_info` round-trip, the missing-
inline-`ics_info` writer rejection, the `scale_flag == true`
rejection on both sides, and the
`spectral_data_bit_offset == bits_written` invariant (the parser
stops exactly at the writer's emitted position). Suite size grows
488 → 503 tests. The `raw_data_block::Walker` still emits
`Element::ChannelElement` for the header alone — wiring the
walker to consume the body via `ics_body::IcsBody::parse` is a
follow-up round.

Round 200 lands
the ISO/IEC 14496-3 Table 4.102 `TNS_MAX_ORDER` and Table 4.103
`TNS_MAX_BANDS` decoder-side clamps that bound the `order` and
band-index fields of every parsed `tns_data()` filter at
reconstruction time, the §4.6.17.2.5 Tables 4.119 / 4.120
LD-specific `TNS_MAX_BANDS` lookups (480- and 512-sample AAC LD
frames), and the §4.6.9.3 three-way `min(band, TNS_MAX_BANDS,
max_sfb)` and `min(order, TNS_MAX_ORDER)` clamp helpers. New
module `tns_max` exposes the public AOT constants
(`AOT_AAC_MAIN = 1`, `AOT_AAC_LC = 2`, `AOT_AAC_SSR = 3`,
`AOT_AAC_LTP = 4`, `AOT_ER_AAC_LD = 23`); the per-rate caps
`TNS_MAX_BANDS_LD_480: [Option<u8>; 12]` and
`TNS_MAX_BANDS_LD_512: [Option<u8>; 12]` (slots 3/4/5/6/7 are
`Some`, every other rate is `None` reflecting the LD-tables-only-cover-5-rates fact); and the accessors
`tns_max_order(aot, window_sequence, fs_index)`,
`tns_max_bands(aot, window_sequence, fs_index)`,
`tns_max_bands_ld_480(fs_index)` /
`tns_max_bands_ld_512(fs_index)`,
`clamp_tns_order(order, aot, ws, fs_index)`, and
`clamp_tns_band(band, max_sfb, aot, ws, fs_index)`. The Table
4.102 dispatch splits AOT 1 / 2 / 3 from "other AOT using TNS"
and partitions the long-window column at the > 32 kHz / ≤ 32 kHz
threshold (fs ≤ 4 vs ≥ 5); the Table 4.103 dispatch routes
AOT 3 (AAC SSR) through the PQF-filterbank columns and every
other AOT through the non-PQF columns. `clamp_tns_band` folds
the three-way `min(band, TNS_MAX_BANDS, max_sfb)` from the
§4.6.9.3 pseudocode into one call so the eventual TNS
reconstruction layer can consume it without re-deriving the AOT
dispatch. Field-validity sanity: every non-LD cap is ≤ the
corresponding `num_swb_long_window` / `num_swb_short_window`
(verified by an integration test that walks every fs × AOT
combination), and every `TNS_MAX_ORDER` cap is within the
Table 4.155 `order` field width (5 bits for long, 3 bits for
short). Rejection: `fs_index >= 13` (the Table 1.18 reserved
slot) surfaces as `Error::IcsInfoUnsupportedSampleRateIndex` on
every accessor; fs 12 (7350 Hz) rejects on the Table 4.103
accessor (not listed in the table) but is accepted on the
Table 4.102 `TNS_MAX_ORDER` accessor; the LD accessors reject
the 7 uncovered rates (fs 0/1/2/8/9/10/11). 34 new tests
(24 unit in `src/tns_max.rs` + 10 integration in
`tests/tns_max.rs`): row-by-row exact-match of every
12-entry table column (Table 4.103 long-non-PQF, short-non-PQF,
long-PQF, short-PQF; Tables 4.119 / 4.120 row-by-row), the
> 32 kHz / ≤ 32 kHz partition for the "other AOT" row of Table
4.102, the short-windows-always-7 row collapse for every AOT,
the AOT 3 PQF-column dispatch, the LongStart / LongStop = long
column verification, the LD 480 vs 512 divergence at 24 / 22.05
kHz (30 vs 31), every `Error::IcsInfoUnsupportedSampleRateIndex`
branch on every accessor, three-way-`min` correctness for
`clamp_tns_band` under each of the three operands dominating
(band wins, max_sfb wins, TNS_MAX_BANDS wins), and the
`tns_max_bands` ≤ `num_swb` integration invariant via the
sibling [`ics_info`](src/ics_info.rs) constants. Suite size
grows 454 → 488 tests.

Round 194
lands the ISO/IEC 14496-3 Tables 4.129–4.141 scalefactor-band offset
tables for all 12 standard `samplingFrequencyIndex` values, the
first spectrum-reconstruction-layer entry point in the crate, and
the §4.6.13 pulse-escape fix-up that consumes one. Public
constants: `SWB_OFFSET_LONG_WINDOW[13]` and
`SWB_OFFSET_SHORT_WINDOW[13]` (slot `12`, 7350 Hz, is an empty
slice — no SWB table is defined for that rate); each non-empty
slot is `num_swb + 1` entries long (the trailing entry is the
spectrum-length sentinel `1024` or `128`). Public constants
`LONG_WINDOW_LEN = 1024` / `SHORT_WINDOW_LEN = 128` pin the
sentinel values. Safe accessors `long_window_offsets(fs_index)` /
`short_window_offsets(fs_index)` reject out-of-range
`fs_index` (`>= 13`) and the 7350 Hz slot via
`Error::IcsInfoUnsupportedSampleRateIndex`. The new
`swb_offset::apply_pulse_data(x_quant, fs_index, pulse_data)`
walks the §4.6.13 pseudocode:
`k = swb_offset_long[fs][pulse_start_sfb]; for each pulse: k +=
pulse_offset[i]; if x_quant[k] > 0 then x_quant[k] += pulse_amp[i]
else x_quant[k] -= pulse_amp[i]`. Rejects empty / oversized pulse
lists, a `pulse_start_sfb` past the last real scalefactor band,
and a `k` accumulation that runs off the 1024-coefficient
spectrum via `Error::PulseDataEncodeInvalid`. The 960-line frame
variant (Tables 4.142–4.147, `frameLengthFlag == 1`) and the
24-bit explicit-rate escape (`samplingFrequencyIndex == 0xf`) are
NOT covered — both demand caller resolution before the accessor
is invoked. 45 new tests (33 unit in `src/swb_offset.rs` + 12
integration in `tests/swb_offset.rs`): per-table band-count
verification, Table-row-by-row spot checks against the spec PDF
(Tables 4.129 / 4.130 / 4.131 / 4.132 / 4.133 / 4.134 / 4.135 /
4.136 / 4.137 / 4.138 / 4.139 / 4.140 / 4.141), strict-monotonic
+ sum-to-spectrum-length invariants for every table, same-table
aliasing (fs 0 / 1 share Table 4.140; 3 / 4 share 4.129; 3 / 4 /
5 share 4.130; 6 / 7 share 4.136 + 4.137; 8 / 9 / 10 share 4.134
+ 4.135), 7350 Hz / out-of-range / explicit-rate rejection, the
§4.6.13 sign-dependent reconstruction (`> 0` positive-add branch,
`<= 0` negative-subtract branch, exact-zero falling into the
else), the multi-pulse `k` chaining (one-pulse, two-pulse,
four-pulse with `(offset=31, amp=15)` saturation), the
zero-`pulse_offset` corner case (two pulses on the same
coefficient), reconstruction at 8 kHz (Table 4.132) and 96 kHz
(Table 4.140), unrelated-coefficient untouched assertion, and
every error-rejection branch (empty pulses, > 4 pulses, oversized
`pulse_start_sfb`, k-overrun, unsupported `fs_index`). Suite size
grows 409 → 454 tests.

Round 192
lands the Table 1.15 trailing-bits backward-compatible probe used
to carry HE-AAC v1 / v2 / ER BSAC extension data **inside the
AudioSpecificConfig** when the outer `audioObjectType` is not the
explicit SBR (5) or PS (29) wrapper. Driven from
`AudioSpecificConfig::parse(&[u8])` (which knows the byte-slice
bound) and the new carrier-bounded entry point
`AudioSpecificConfig::parse_bits_bounded(reader,
origin_bit_offset, asc_bit_length)`. After the per-AOT body
(plus optional `epConfig`), when at least 16 bits remain, the
parser reads an 11-bit `syncExtensionType`; on a `0x2b7` match it
consumes a nested `GetAudioObjectType()` and dispatches the
extension AOT (`5` = HE-AAC SBR or `22` = ER BSAC; any other
value surfaces `Error::UnsupportedTrailingExtensionAot`). The
SBR branch reads `sbrPresentFlag`, an optional
`extensionSamplingFrequencyIndex` (with the 24-bit explicit-rate
escape mirroring the outer ASC), and, when at least 12 further
bits remain, a second 11-bit `syncExtensionType` gating a 1-bit
`psPresentFlag` on the `0x548` match. The ER BSAC branch reads
`sbrPresentFlag`, the optional `extensionSamplingFrequencyIndex`,
and a mandatory 4-bit `extensionChannelConfiguration`. Probe
results land in the new `AudioSpecificConfig::trailing_sbr_probe:
Option<SbrExtensionProbe>` carrier; when the probe resolves SBR
or PS, `asc.sbr_present` / `asc.ps_present` /
`asc.extension_sampling_frequency_index` /
`asc.extension_sample_rate` / `asc.extension_channel_configuration`
are also promoted so callers reading the top-level fields see the
implicit signalling. The existing bit-level
`AudioSpecificConfig::parse_bits(reader, origin_bit_offset)` keeps
its no-probe semantics so callers holding a `BitReader` carrying
trailing carrier bytes are not surprised by a stray 11-bit match.
20 new integration tests cover the constant-pin, the four
"probe-skipped" branches (no trailing bits, fewer than 16 bits
remaining, 11-bit non-match, outer SBR / PS wrapper guard), the
SBR-branch round-trips (sbrPresentFlag clear and set, 24-bit
escape `extensionSamplingFrequencyIndex == 0xf`, PS sub-probe
present, PS marker mismatch, inner-guard under 12 bits), the ER
BSAC-branch round-trips (sbrPresentFlag clear and set), the
`UnsupportedTrailingExtensionAot` reject for AOT 3, the
`parse_bits` no-probe contract, the `parse_bits_bounded`
explicit-length call shape (including a truncated-bound assertion
that the probe does not over-read), a mid-escape truncation
(`UnexpectedEnd`), and a hand-pinned 7-byte HE-AAC v2 implicit
chain (AOT-2/16kHz/mono + 0x2b7 + ext_aot=5 + sbr=1 + sfi=5 +
0x548 + ps=1). Suite size grows 389 → 409. A new
`Error::UnsupportedTrailingExtensionAot(u8)` variant surfaces
non-{5, 22} extension AOTs returned by `GetAudioObjectType()`
inside the probe; new public constants `SYNC_EXTENSION_TYPE_SBR`
(0x2b7), `SYNC_EXTENSION_TYPE_PS` (0x548),
`SYNC_EXTENSION_TYPE_BITS` (11), `TRAILING_EXTENSION_AOT_SBR`
(5), `TRAILING_EXTENSION_AOT_BSAC` (22) pin the field-width and
marker constants.

Round 187 lands the §4.4.2.7 /
Table 4.51 `extension_payload()` parser **and** encoder primitive,
the seventh tool-level encode-side syntax writer in the crate. The
new `extension_payload` module covers the three Table 4.51
`extension_type` branches whose body layouts are fully specified
by fixed-width fields: `EXT_FILL` (`0b0000`) — the default branch
surfacing `8 * (cnt - 1) + 4` `other_bits` as a packed byte buffer;
`EXT_FILL_DATA` (`0b0001`) — the normative-pattern filler with
`fill_nibble == 0b0000` and `fill_byte == 0b1010_0101`; and
`EXT_DYNAMIC_RANGE` (`0b1011`) — the Table 4.52
`dynamic_range_info()` block with optional `pce_instance_tag`
(4 + 4 bits), optional Table 4.53 `excluded_channels()` exclude-mask
list (7 mask bits + 1 continuation bit per 8-bit group), optional
per-band partitioning (4-bit `drc_band_incr`, 4-bit
`drc_bands_reserved_bits`, then `1 + drc_band_incr` × 8-bit
`drc_band_top[]`), optional `prog_ref_level` (7 + 1 bits), and
per-band `(1-bit dyn_rng_sgn, 7-bit dyn_rng_ctl)` records. The two
SBR-data extension types from ISO/IEC 13818-7 Table 40
(`EXT_SBR_DATA` `0b1101` and `EXT_SBR_DATA_CRC` `0b1110`) surface
as `Error::UnsupportedExtensionSbr` — their bodies are the
`sbr_extension_data()` syntax which needs the QMF / patching
back-end this crate does not yet provide. All other 4-bit values
surface as `Error::UnsupportedExtensionType` (Table 4.59 / Table 40
list them as "reserved"). A new `Error::ExtensionPayloadInvalid`
variant surfaces caller-side wire-field violations: `cnt == 0` (no
room for the extension_type nibble); EXT_FILL `other_bits.len()` not
matching the `8 * (cnt - 1) + 4` body-bits ceiling; EXT_FILL_DATA
non-`0b0000` `fill_nibble` or non-`0b1010_0101` `fill_byte`; DRC
Table 4.52 derived `n` disagreeing with the dispatching FIL `cnt`;
DRC field overflows (`pce_instance_tag > 0x0f`,
`drc_tag_reserved_bits > 0x0f`, `drc_band_incr > 0x0f`,
`drc_bands_reserved_bits > 0x0f`, `prog_ref_level > 0x7f`,
`dyn_rng_ctl > 0x7f`); internal shape mismatches
(`band_top.len() != 1 + band_incr`, `bands.len() != drc_num_bands`);
and an `excluded_channels.exclude_mask.len()` that is not a positive
multiple of 7 (Table 4.53 emits exclusion bits in fixed groups of
7). 36 new integration tests cover Table 4.59 / Table 40 dispatch
(known values, round-trip, SBR rejection, reserved rejection), the
EXT_FILL `cnt == 1` collapsed body (4 unused trailing bits), an
EXT_FILL `cnt == 3` round-trip with a partial trailing byte, the
EXT_FILL_DATA `cnt == 1` and `cnt == 3` cases with hand-pinned byte
sequences (`[0x10]` and `[0x10, 0xA5, 0xA5]`) and the two normative
rejections (non-zero nibble, non-`0xA5` byte), the DRC minimal body
with all four optionals absent (hand-pinned bytes
`[0xB0, 0x42]`), single-optional round-trips for pce_tag /
excluded_channels / drc_bands / prog_ref_level, an
every-optional-present round-trip (`n == 9`), the
continuation-bit positioning across a two-group exclude_mask
(hand-pinned bytes `[0xB6, 0x04, 0x08, 0x00]`), six encoder-side
overflow rejections, a parser-side `cnt`-vs-`n` mismatch detection,
three truncation tests, an `excluded_group_count` lookup-table
check, the `byte_length` / `num_bands` accessor checks, and a
trailing-bit non-consumption assertion confirming the parser stops
exactly at bit-position `8 * n`. Suite size grows 353 → 389. The
§4.5.2.13 DRC companding-curve application is **not** performed
here — the raw `(dyn_rng_sgn, dyn_rng_ctl)` records are surfaced
verbatim for a later round.

Round 183 closed the SSR-side
gap by landing the §4.4.6.5 / Table 4.12 `gain_control_data()`
parser + encoder primitive (`GainControlData::parse` /
`GainControlData::write`) — the sixth tool-level encode-side syntax
writer in the crate. The wire layout is a 2-bit `max_band` followed
by, for each `bd ∈ 1..=max_band` and each `wd ∈ 0..N(window_sequence)`,
a 3-bit `adjust_num` then `adjust_num` `(4-bit alevcode, W(seq,
wd)-bit aloccode)` records. The per-`window_sequence` window count
`N` (1 / 2 / 8 / 2 for `OnlyLong` / `LongStart` / `EightShort` /
`LongStop`) and the per-`(seq, wd)` `aloccode` width table
(5 / 4-2 / 2 / 4-5) are exposed as the public helpers
`gain_control_data::num_windows` and `aloccode_bits` so downstream
callers can compute the body size without re-deriving Table 4.12.
A new `Error::GainControlDataEncodeInvalid` variant surfaces
caller-side wire-field violations: `max_band > 3`, `bands.len() !=
max_band`, a per-`bd` `windows.len()` that disagrees with the
surrounding `window_sequence`, a per-`(bd, wd)` `adjustments.len()
> 7`, an `alevcode > 0x0f`, or an `aloccode` exceeding the per-slot
width cap. 20 new integration tests cover the `num_windows` /
`aloccode_bits` lookup tables, the `max_band == 0` body byte
layout, a hand-pinned `OnlyLong` wire-bit assertion, self-roundtrip
across the four window sequences (including a max-everything
`EightShort` with `max_band = 3` and every per-slot adjustment
maxed out), six encoder-side overflow rejections (max-band,
band-count mismatch, window-count mismatch, adjust-num overflow,
alevcode overflow, aloccode overflow at the 5-bit / 2-bit slots),
an `UnexpectedEnd` mid-record truncation test, and a trailing-bit
non-consumption check confirming the parser does not over-read
past the Table 4.12 body. Suite size grows 333 → 353. The §4.6.12
ladder-application loop that reconstructs sample-domain
attenuation factors from the `(alevcode, aloccode)` pairs is
**not** performed here — it needs the SSR PQF / IMDCT back-end,
which is not part of Phase 2.

Round 177 closes the two
configuration-layer Phase 1 gaps that the previous rounds had left as
explicit deferrals: the `GASpecificConfig` `extensionFlag == 1`
subtree (Table 4.1: AOT 22's 5-bit `numOfSubFrame` + 11-bit
`layer_length`; the AOT-17 / 19 / 20 / 23 `aacSection /
Scalefactor / Spectral DataResilienceFlag` triplet; the
always-present `extensionFlag3` tail bit, whose body is reserved by
the spec as "tbd in version 3" and is rejected via the new
`Error::UnsupportedAscExtensionFlag3`) and the Table 1.15 trailing
2-bit `epConfig` field for ER object types in {17, 19, 20, 21, 22,
23, 24, 25, 26, 27, 39}. `epConfig ∈ {0, 1}` are accepted and
surfaced as `AudioSpecificConfig::ep_config = Some(_)`; `epConfig ==
2` or `epConfig == 3` (which mandate the inline
`ErrorProtectionSpecificConfig()` body that Phase 1 does not parse)
surface as `Error::UnsupportedEpConfig(u8)`. New carrier types in
the public API: `GaExtensionBody`, `BsacLayerSpec`,
`AacResilienceFlags`. 13 new integration tests in `tests/asc.rs`
cover the resilience triplet for ER AAC LC (AOT 17), epConfig 0
through 3 round-trip / rejection, ER BSAC (AOT 22) with the
`numOfSubFrame` pair and an inline PCE, ER AAC LD (AOT 23) with
the resilience triplet only, ER AAC scalable (AOT 20) hitting both
the `layerNr` branch AND the resilience triplet in the correct
Table 4.1 order, ER TwinVQ (AOT 21) where the extension body
collapses to `extensionFlag3` + `epConfig` only, the
`extensionFlag3 == 1` rejection, truncation inside the resilience
body and at the `epConfig` field, and a hand-pinned bit-position
assertion (a 22-bit ER AAC LC ASC). Suite size grows 320 → 333.

Round 165 had landed
[`pce::Pce::write`] — the encoder-side counterpart of the round-126
`Pce::parse`, emitting the full Table 4.2 wire layout (4-bit
`element_instance_tag` + 2-bit `object_type` + 4-bit
`sampling_frequency_index`; the six element-count fields with widths
4 / 4 / 4 / 2 / 3 / 4; three mix-down presence bits and their
optional 4 / 4 / (2+1)-bit bodies; per-element select lists for
front / side / back as `(is_cpe, tag_select)` pairs plus bare
4-bit `tag_select` lists for LFE / assoc and `(is_ind_sw,
tag_select)` pairs for `valid_cc`; a §4.4.1.1 Note 1 origin-relative
`byte_alignment()`; then the 8-bit `comment_field_bytes` length
prefix and payload bytes) — and the matching
[`raw_data_block::FrameAssembler::push_pce`] that emits the 3-bit PCE
`id_syn_ele` (`0b101`) before the body and passes
`origin_bit_offset = 0` so the byte-alignment collapses to the
absolute byte boundary inside the assembler. The origin handling
mirrors the parser exactly: pass `0` for a standalone PCE inside a
`raw_data_block()`, or the ASC origin bit-position for a PCE inline
in [`AudioSpecificConfig`](crate::asc::AudioSpecificConfig). A new
`Error::PceEncodeInvalid` variant surfaces every caller-side
wire-field violation: `element_instance_tag > 0x0f`, `object_type >
0x03`, `sampling_frequency_index > 0x0f`, `front`/`side`/`back`
element-list lengths > 15, LFE list length > 3, assoc list length >
7, CC list length > 15, any per-element `tag_select > 0x0f`,
`matrix_mixdown.idx > 0x03`, mono / stereo mix-down element numbers >
0x0f, or `comment_field.len() > 0xff`. 26 new integration tests cover
self-roundtrip (`write` → `parse`) across the empty PCE, a 5.1 layout
with mixed SCE / CPE / LFE / CC and a comment field, the
all-mix-downs-set permutation, the maximum-fields ladder
(`tag = 0x0f`, `object_type = 3`, `sfi = 0x0f`, 255-byte comment),
and a non-zero-origin parser / writer agreement test; six
`FrameAssembler::push_pce` Walker round-trip tests (PCE+END, PCE
after SCE header, PCE+FIL+END, full-mix-downs PCE) plus two error-
propagation tests; and a hand-pinned wire-layout assertion that the
all-zero PCE produces 6 all-zero bytes (header + counts + flags +
6-bit pad + comment length). Suite size grows 294 → 320 tests.

Round 160 had landed [`raw_data_block::FrameAssembler`] — the encoder-side
composition primitive that splices the existing typed writers
(`IcsInfo::write`, `SectionData::write`, `ScaleFactorData::write`,
`PulseData::write`, `TnsData::write`) into a complete bit-exact
`raw_data_block()` byte stream, the inverse of the round-121
`Walker`. The push-API covers every element type with a Phase-1 / 2
parser counterpart: `push_channel_header` for SCE / CPE / CCE / LFE
(3-bit `id_syn_ele` + 4-bit `element_instance_tag`),
`push_channel_body_bits` for splicing a pre-serialised body bit-slice
behind a header, `push_fill` for §4.4.2.7 FIL (with the 8-bit
`esc_count` escape inverting the parser's `cnt = esc_count + 15 − 1`
to `esc_count = payload_bytes − 14`, single-element fill capped at
269 bytes), `push_data` for §4.4.2.5 DSE (with the 8-bit `esc_count`
escape inverting `cnt = count + esc_count` to `esc_count =
payload_bytes − 255`, single-element data capped at 510 bytes, plus
the optional `data_byte_align_flag` honoured **before** the payload),
and `push_end` which emits the 3-bit `END (0b111)`, byte-aligns per
§4.4.2.1, and consumes the assembler to return the finished
`Vec<u8>`. END is mandatory — `push_end` is a destructor-style
terminator, so post-END pushes are a compile-time error rather than a
runtime check. **Round 165** now closes the PCE-writer gap with
[`raw_data_block::FrameAssembler::push_pce`] (3-bit `id_syn_ele`
prefix + the round-165 `Pce::write` body). The new
`Error::RawDataBlockEncodeInvalid` surfaces every caller-side wire-
field violation: non-channel `IdSynEle` on `push_channel_header`,
over-4-bit `element_instance_tag` on channel or DSE elements, FIL
payload above 269 bytes, DSE payload above 510 bytes, and
`push_channel_body_bits` `bit_count` exceeding `bits.len() × 8`. 30
new integration tests in `tests/raw_data_block_encode.rs` exercise
every entry point against the round-121 `Walker`, including
hand-pinned wire-byte assertions (`[0xE0]` END-only, `[0x15, 0xC0]`
SCE(tag=0x0a)+END, `[0xC1, 0xC0]` empty FIL+END, `[0x01, 0x57,
0x9C]` SCE(tag=0)+0xABC(12-bit body)+END), escape-boundary roundtrips
at FIL(15) / FIL(16) / FIL(269) / DSE(255) / DSE(510), and a
composite CPE+13-bit-body+DSE+FIL+END frame that walks every push
entry-point's bit-position handoff. Suite size grows 264 → 294
tests.

Round 152 had landed `accumulate()` / `differentiate()` — the
symmetric decoder / encoder pair that converts between transmitted
DPCM deltas and absolute per-band quantities. Three independent
tracks run in
lockstep: spectrum scalefactors (seed `last_sf = global_gain`, range
`0..=255`), intensity stereo positions (seed `last_is = 0`), and PNS
noise energies (seed `last_nrg = global_gain - NOISE_OFFSET - 256`
with `NOISE_OFFSET = 90`; the first PNS band of the frame carries
the 9-bit `uimsbf` literal directly into `last_nrg`, every
subsequent PNS band a `-60..=+60` Huffman delta). The §4.6.2.3.2
illustrative pseudocode that would single-track everything is
deliberately not implemented — the surrounding §4.6.8.1.4 / §4.6.13
prose "differential decoding is done separately between
scalefactors, intensity stereo positions and noise energies" takes
precedence (the §4.6.2.3.2 listing predates MPEG-4 PNS and is
identical in 13818-7 §11.3.2 where PNS does not exist). The
encoder-side `differentiate()` checks every spectrum / intensity /
PNS-subsequent delta against Table 4.150's `-60..=+60`, the first
PNS band's literal against the 9-bit `uimsbf` field (0..=511), and
the §4.6.2.3.2 Note's `sf ∈ 0..=255` range — rejections surface as
`Error::ScaleFactorAccumulatorInvalid`. Round 149 had landed the
`scale_factor_data()` parser **and** `ScaleFactorData::write`
encoder primitive (ISO/IEC 14496-3 §4.4.6 / Table 4.53,
non-resilient branch, plus the §4.6.3 / Table 4.A.1 "scalefactor
Huffman codebook" — codebook 12) — the fifth encode-side
syntax-element writer in the crate. Table 4.A.1 is transcribed
verbatim from the spec (121 entries indexed `0..=120`, max length
19 bits, codeword for index 60 / delta 0 is the single bit `0`)
and is a *complete* prefix code (Kraft equality, exhaustively
verified by walking all `2^19` 19-bit prefixes). Public helpers
`hcod_sf_encode(dpcm: i8) -> (length, codeword)` and
`hcod_sf_decode(reader) -> i8` expose the Table 4.A.1 codebook
directly; `SF_INDEX_OFFSET = -60` and `NOISE_PCM_BITS = 9`
constants pin the §4.6.3 / Table 4.150 dispatch. The parser /
writer walk the per-`(g, sfb)` non-`ZERO_HCB` subsequence driven
by `SectionData::sfb_cb`, dispatching between `hcod_sf[]` (ordinary
spectrum / PNS-after-first / both intensity codebooks 14 + 15)
and the 9-bit `dpcm_noise_nrg` PCM seed (first PNS band of the
frame; `noise_pcm_flag` is *frame*-scoped, not group-scoped). The
writer's structural sieve rejects every illegal in-memory shape
(`Error::ScaleFactorDataEncodeInvalid`): outer length mismatch
vs `sfb_cb`, missing or surplus entries, variant ↔ codebook
mismatch, `NoisePcm` re-used after `noise_pcm_flag` clears,
`NoiseDpcm` on the first PNS band, DPCM delta outside `-60..=+60`,
`NoisePcm` magnitude > 0x1FF. The §4.6.2.3.2 `last_sf =
global_gain` accumulator (DPCM → absolute `sf[g][sfb] ∈ 0..=255`)
is **not** performed; that's a per-AOT decoder step. The §4.4.6
error-resilient branch (`aacScalefactorDataResilienceFlag == 1`,
RVLC + `rev_global_gain` + `sf_concealment`) is **not** implemented;
ER AAC-LD / scalable profiles will need a sibling
`scale_factor_data_rvlc()` module. Round 146 had landed the
`tns_data()` parser **and** `TnsData::write` encoder primitive
(ISO/IEC 14496-3 §4.4.6 / Table 4.54, with the §4.6.9.2 Table 4.155
size switch) — the fourth encode-side syntax-element writer in the
crate, covering every transform window of the surrounding
`window_sequence` (`num_windows` = 8 for `EIGHT_SHORT_SEQUENCE`, 1
otherwise), with `n_filt` (1 or 2 bits), the optional `coef_res`
(when `n_filt > 0`), per-filter `length` (4 or 6 bits), `order`
(3 or 5 bits), and — when `order > 0` — `direction`,
`coef_compress`, and `order` × `coef[i]` magnitudes whose width is
`coef_bits = (3 + coef_res) − coef_compress ∈ {2, 3, 4}` per
§4.6.9.3. Public helpers `field_widths`, `num_windows`, and
`coef_bits` expose the Table 4.155 / §4.5.2.3.4 / §4.6.9.3
dispatch. Round 142 had landed `PulseData::parse` /
`PulseData::write` (§4.4.6.3 / Table 4.7) — the third encode-side
primitive, covering the 2-bit `number_pulse` + 6-bit
`pulse_start_sfb` + 1..=4 `(5-bit offset, 4-bit amp)` records.
Round 140 had landed `IcsInfo::write` (and a public `write_ltp_data`
helper) — the inverse of the round-129 `ics_info()` parser and the
crate's second encode-side syntax-element writer. The new primitive
covers the full Table 4.6 surface (`ics_reserved_bit` 1 +
`window_sequence` 2 + `window_shape` 1; `max_sfb` 4 + 7-bit grouping
mask on `EIGHT_SHORT_SEQUENCE`; `max_sfb` 6 + `predictor_data_present`
1 plus the per-AOT predictor / LTP body on every other window
sequence) and the Table 4.55 `ltp_data()` body for both the non-LD
11-bit-lag form and the ER-AAC-LD delta-coded form, with the second
`ltp_data_present` (+ optional paired body) for CPE common-window
streams. 35 new self-roundtrip integration tests exercise the long
branch, EIGHT_SHORT grouping, Main predictor (with and without reset,
including the `PRED_SFB_MAX[fs_index]` cap), non-LD LTP, ER-AAC-LD LTP
with and without `lag_update`, CPE common-window paired LTP, every
encoder-rejection branch (`Error::IcsInfoEncodeInvalid` for field
overflow, wrong AOT vs body slot, missing flag matching, stale data
on the predictor-absent branch, …), plus a hand-pinned wire-layout
assertion (`buf[0] == 0x06`, `buf[1] == 0x40` for a 25-band LC long
sequence). Round 137 had landed the `section_data()` writer; round
133 the `section_data()` parser; round 129 the `ics_info()` parser;
earlier rounds (121 / 126) the ADTS framing, out-of-band
`AudioSpecificConfig`, the `raw_data_block()` walker, and the
`program_config_element()` parser. The current capability set:

- **ADTS fixed + variable header parser** (ISO/IEC 13818-7
  §1.A.2.2.1–.2): syncword, ID, layer, `protection_absent`, profile,
  `sampling_frequency_index`, `channel_configuration`,
  `aac_frame_length`, `adts_buffer_fullness`, and
  `number_of_raw_data_blocks_in_frame` (decoded as `N`, not the wire
  `N − 1`). Sample-rate index is resolved against ISO/IEC 14496-3
  Table 1.18; reserved indices (13, 14, 15) are rejected. CRC bytes
  are confirmed present when `protection_absent == 0` but the CRC
  value itself is **not** validated yet.
- **`AudioSpecificConfig` parser** (ISO/IEC 14496-3 §1.6.2.1 +
  §4.4.1, **new in round 126**): full `GetAudioObjectType()`
  escape (5-bit base + 6-bit extension), `samplingFrequencyIndex`
  (4-bit + 24-bit explicit-rate escape at `0xf`),
  `channelConfiguration` (Table 1.19), and `GASpecificConfig`
  (Table 4.1) body for every GA `audioObjectType` (1, 2, 3, 4, 6,
  7, 17, 19, 20, 21, 22, 23) including `frameLengthFlag`
  (1024 vs 960 lines), `dependsOnCoreCoder` (+ optional 14-bit
  `coreCoderDelay`), `extensionFlag`, the AOT-6 / 20 `layerNr`,
  and an inline `program_config_element()` when
  `channelConfiguration == 0`. The hierarchical SBR (AOT 5) and
  PS (AOT 29) outer wrappers are unwrapped — `sbr_present` /
  `ps_present` flags + extension sampling-frequency index +
  inner AOT are all surfaced.
- **`program_config_element()` parser** (ISO/IEC 14496-3 §4.4.1.1
  / ISO/IEC 13818-7 §8.5 Table 25, **new in round 126**): all
  element counts (`num_front`, `num_side`, `num_back`, `num_lfe`,
  `num_assoc`, `num_valid_cc`); every per-element select
  (`*_element_is_cpe`, `*_element_tag_select`); mono / stereo /
  matrix mix-down hints; `byte_alignment()` honouring the Table 4.2
  Note 1 ASC-origin relative-alignment requirement; full
  `comment_field` round-trip. Both standalone-in-`raw_data_block`
  and inline-in-ASC call sites are supported via a single
  `Pce::parse(reader, origin_bit_offset)`.
- **`raw_data_block()` syntactic walker** (ISO/IEC 14496-3
  §4.4.2.1): iterates `id_syn_ele` and recognises the eight element
  types (SCE, CPE, CCE, LFE, DSE, PCE, FIL, END). Per-element body
  handling:
  - SCE / CPE / CCE / LFE: emit `ChannelElement { kind, tag }`
    after consuming the 3-bit id and 4-bit `element_instance_tag`.
    The channel-element body itself is not parsed yet.
  - FIL (§4.4.2.7): full count + `esc_count` parse; payload bytes
    are skipped.
  - DSE (§4.4.2.5): full header parse including
    `data_byte_align_flag`; payload bytes are skipped.
  - PCE (§4.4.1.1, **round 126**): full parse via the `pce` module;
    walker emits `Element::ProgramConfig(Pce)`.
  - END: byte-aligns the bit-reader and stops cleanly; further calls
    return `None`.
- **`ics_info()` parser** (ISO/IEC 14496-3 §4.4.6 Table 4.6, **new
  in round 129**): full Table 4.6 surface — `ics_reserved_bit`,
  `window_sequence` (Table 4.128), `window_shape`, `max_sfb` (4-bit
  EIGHT_SHORT / 6-bit long), `scale_factor_grouping` mask plus the
  Main AOT's `predictor_data()` body (`predictor_reset` +
  `reset_group_number` + `prediction_used[]` capped at the
  sample-rate-dependent `PRED_SFB_MAX` from ISO/IEC 13818-7
  Table 62) and every non-Main AOT's `ltp_data_present` chain.
  `ltp_data()` (Table 4.55) is fully parsed for both the ER-AAC-LD
  delta-coded form (AOT 23) and the unconditional 11-bit-lag form
  (every other GA AOT), including the second `ltp_data_present` (+
  optional second `ltp_data()`) for CPE common-window streams. The
  parser exposes the §4.5.2.3.4 derivations (`num_windows`,
  `num_window_groups`, `window_group_length[]`, `num_swb`)
  alongside the literal wire fields. Sample-rate count tables for
  long (Tables 4.129 / 4.131 / 4.132 / 4.134 / 4.136 / 4.138 /
  4.140) and short windows (Tables 4.130 / 4.133 / 4.135 / 4.137 /
  4.139 / 4.141) are encoded as `NUM_SWB_LONG_WINDOW[12]` /
  `NUM_SWB_SHORT_WINDOW[12]` constants.
- **`section_data()` parser** (ISO/IEC 14496-3 §4.4.6 / ISO/IEC
  13818-7 §6.3 Table 17, **new in round 133**): run-length section
  assignment with the §6.3 escape mechanism — per window group it
  reads `sect_cb` (4 bits) and accumulates `sect_len` from
  `sect_len_incr` fields (3-bit / `sect_esc_val == 7` for
  EIGHT_SHORT, 5-bit / `sect_esc_val == 31` otherwise), repeating
  the increment read while it equals the escape value. Produces the
  ordered per-group `Section { codebook, start, end }` lists plus
  the flattened `sfb_cb[g][sfb]` map that `scale_factor_data()`
  consumes. The codebook constants (`ZERO_HCB`, `FIRST_PAIR_HCB`,
  `ESC_HCB`, `NOISE_HCB`, `INTENSITY_HCB2`, `INTENSITY_HCB`) and a
  `Codebook` classifier (QUAD / PAIR signedness per Table 59,
  intensity / PNS / zero predicates) are exposed for the downstream
  tools. Every field is fixed-width — no Huffman decode yet.
- **`ics_info()` encoder primitive** (ISO/IEC 14496-3 §4.4.6
  Table 4.6 / Table 4.55, **new in round 140**):
  `IcsInfo::write(writer, audio_object_type, sampling_frequency_index,
  common_window)` — the inverse of the round-129 parser. Emits the
  bit-exact Table 4.6 stream: 1-bit `ics_reserved_bit`, 2-bit
  `window_sequence`, 1-bit `window_shape`, then either the 4-bit
  `max_sfb` + 7-bit `scale_factor_grouping` short-window pair, or the
  6-bit `max_sfb` + 1-bit `predictor_data_present` long-window pair
  with the per-AOT predictor body (Main: 1-bit `reset` + optional
  5-bit `reset_group_number` + `min(max_sfb, PRED_SFB_MAX[fs_index])`
  `prediction_used` bits; LTP / other GA: `write_ltp_data` for the
  primary channel + an optional second `ltp_data_present` + paired
  body when `common_window == true`). The companion
  `write_ltp_data(writer, ltp, aot, window_sequence, max_sfb)` covers
  the Table 4.55 body for both the non-LD 11-bit-lag form and the
  ER-AAC-LD (AOT 23) delta-coded form, including the `EIGHT_SHORT`
  omission of `ltp_long_used[]` in the non-LD branch. Caller-side
  structural bugs surface as `Error::IcsInfoEncodeInvalid`
  (`max_sfb` field-width overflow, wrong AOT vs predictor / LTP slot,
  missing or extra pair flag, `prediction_used[]` / `long_used[]`
  length mismatch, `coef`/`lag`/`reset_group_number` field overflow,
  populated predictor or LTP slot on the `EIGHT_SHORT` or
  predictor-absent branch).
- **`section_data()` encoder primitive** (ISO/IEC 14496-3 §4.4.6 /
  ISO/IEC 13818-7 §6.3 Table 17, **new in round 137**):
  `SectionData::write(writer, window_sequence, max_sfb)` — the
  inverse of the parser. Walks each window group's section list and
  emits `sect_cb` (4 bits) + the inverse-§6.3 `sect_len_incr`
  sequence: while remaining `sect_len >= sect_esc_val`, emit the
  escape value and subtract; emit the residual (in
  `[0, sect_esc_val)`) once at the end. The "greater than or equal
  to" boundary forces a trailing non-escape `sect_len_incr == 0`
  when the run length is an exact multiple of `sect_esc_val`,
  preserving the parser's loop-termination invariant. Validates
  caller-provided structure: per-group sections must be contiguous
  `[0, max_sfb)`, `sect_cb` must fit the 4-bit field, and
  zero-length sections are rejected (`Error::SectionDataEncodeInvalid`).
  No Huffman tables of its own — purely the fixed-width side of
  Table 17.
- **`pulse_data()` parser + encoder primitive** (ISO/IEC 14496-3
  §4.4.6.3 / Table 4.7, **new in round 142**):
  `PulseData::parse(reader)` and `PulseData::write(writer)` cover the
  full pulse-escape tool — 2-bit `number_pulse` (wire value
  `pulses.len() - 1`, so 1..=4 pulses), 6-bit `pulse_start_sfb`, then
  `number_pulse + 1` `(5-bit pulse_offset, 4-bit pulse_amp)` records.
  Every field is fixed-width; no Huffman tables, no `swb_offset`
  dependence, no surrounding-element state. Public constants
  (`PULSE_OFFSET_BITS`, `PULSE_AMP_BITS`, `MAX_PULSES`) pin the
  Table 4.7 widths for downstream call sites. The §4.6.13
  reconstruction loop (`k += swb_offset[pulse_start_sfb] +
  pulse_offset[j]; x_quant[…] ±= pulse_amp[j]`) is **not** performed
  here — it needs `swb_offset_long_window[]` and the post-Huffman
  `x_quant` array that arrive with `spectral_data()`. Caller-side
  structural bugs surface as `Error::PulseDataEncodeInvalid` (empty
  pulse list, `> 4` pulses, `pulse_start_sfb > 63`, `pulse_offset > 31`,
  or `pulse_amp > 15`).
- **`tns_data()` parser + encoder primitive** (ISO/IEC 14496-3
  §4.4.6 / Table 4.54 with the §4.6.9.2 Table 4.155 size switch,
  **new in round 146**): `TnsData::parse(reader, window_sequence)`
  walks every transform window (`num_windows` = 8 for
  `EIGHT_SHORT_SEQUENCE`, 1 otherwise) and reads `n_filt[w]` with
  the field-width switch (1 bit short / 2 bits long), an optional
  `coef_res[w]` (when `n_filt[w] > 0`), then per-filter
  `length[w][filt]` (4 / 6 bits), `order[w][filt]` (3 / 5 bits),
  and — when `order > 0` — `direction`, `coef_compress`, and
  `order` × `coef[i]` unsigned magnitudes whose width is
  `coef_bits = (3 + coef_res) − coef_compress ∈ {2, 3, 4}` per
  §4.6.9.3 `tns_decode_coef`. `TnsData::write(writer,
  window_sequence)` is the bit-exact inverse. Public helpers
  `field_widths(window_sequence)`, `num_windows(window_sequence)`,
  and `coef_bits(coef_res, coef_compress)` plus the
  `N_FILT_BITS_*` / `LENGTH_BITS_*` / `ORDER_BITS_*` /
  `COEF_RES_BITS` / `DIRECTION_BITS` / `COEF_COMPRESS_BITS`
  constants pin the Table 4.155 / §4.5.2.3.4 / §4.6.9.3 dispatch.
  The §4.6.9.3 `tns_decode_coef` LPC reconstruction (signed
  conversion, `iqfac` arcsine inverse-quantisation, Levinson-style
  conversion to LPC coefficients) and the §4.6.9.3 `tns_ar_filter`
  all-pole pass over the spectrum are **not** performed here — they
  need the spectral context that arrives with the per-AOT IMDCT
  back-end. The §4.6.9.4 `TNS_MAX_ORDER` / `TNS_MAX_BANDS` clamp
  tables (Tables 4.156 / 4.157) are similarly the decoder
  reconstruction layer's responsibility. Caller-side structural
  bugs surface as `Error::TnsDataEncodeInvalid` (wrong
  `windows.len()` for the surrounding `window_sequence`,
  `n_filt` / `length` / `order` field overflow, `coef[]` length
  mismatch with `order`, coef-value overflow at any
  `coef_bits` tier, or zero-`order` filter carrying a non-default
  `direction` / `coef_compress` that would be silently dropped on
  the wire).
- **`swb_offset` module** (ISO/IEC 14496-3 §4.5.4.1 / Tables
  4.129–4.141, **new in round 194**): the per-sample-rate
  `swb_offset_long_window[]` and `swb_offset_short_window[]`
  lookup tables, plus the §4.6.13 pulse-escape reconstruction
  loop. Constants `SWB_OFFSET_LONG_WINDOW[13]` /
  `SWB_OFFSET_SHORT_WINDOW[13]` map every standard
  `samplingFrequencyIndex` (Table 1.18 slots 0..=11) to a
  per-band offset slice; slot `12` (7350 Hz) is an empty slice —
  no SWB table is defined. Each non-empty slot is `num_swb + 1`
  entries long (the trailing entry is the spectrum-length
  sentinel `1024` for long, `128` for short). Safe accessors
  `long_window_offsets(fs_index)` /
  `short_window_offsets(fs_index)` reject out-of-range
  `fs_index` and 7350 Hz via
  `Error::IcsInfoUnsupportedSampleRateIndex`. Public constants
  `LONG_WINDOW_LEN = 1024` / `SHORT_WINDOW_LEN = 128` pin the
  sentinel values. `apply_pulse_data(x_quant, fs_index,
  pulse_data)` is the first spectrum-reconstruction-layer entry
  point in the crate — it walks the §4.6.13 pseudocode `k =
  swb_offset_long[fs][pulse_start_sfb]; for i in 0..number_pulse
  + 1: k += pulse_offset[i]; if x_quant[k] > 0 then x_quant[k] +=
  pulse_amp[i] else x_quant[k] -= pulse_amp[i]`, with structural
  rejections (empty / oversized pulse list, `pulse_start_sfb`
  past the last real band, `k` overrun past `LONG_WINDOW_LEN`,
  unsupported `fs_index`) surfaced via the existing
  `Error::PulseDataEncodeInvalid` and
  `Error::IcsInfoUnsupportedSampleRateIndex` variants. The
  960-line frame variant (Tables 4.142–4.147, `frameLengthFlag
  == 1`) is **not** covered.

## What Phase 2 still omits

- The remaining channel-stream tool after `scale_factor_data()`:
  `spectral_data()`. The walker therefore still cannot iterate past
  a single channel-element body. Round 219 lands the first, round
  226 the second, and round 231 the third of the eleven spectrum
  Huffman codebooks needed to drive that tool — Codebook 1
  (Table 4.A.2), Codebook 2 (Table 4.A.3), and Codebook 3
  (Table 4.A.4). Codebooks 4..=11 (Tables 4.A.5 … 4.A.12) reuse the
  same module shape and are owed in subsequent rounds; the
  `spectral_data()` driver that dispatches per-band onto the chosen
  codebook arrives once all eleven are in place.
- The §4.6.12 `gain_control_data()` ladder-application loop. The
  Table 4.12 record is parseable / writable bit-for-bit, but
  reconstructing sample-domain attenuation factors from the
  `(alevcode, aloccode)` pairs requires the SSR PQF / IMDCT back-end
  (still owed).
- The `scale_factor_data()` error-resilient branch
  (`aacScalefactorDataResilienceFlag == 1` → RVLC with
  `rev_global_gain`, `length_of_rvlc_sf`, `sf_concealment`,
  `length_of_rvlc_escapes`); ER AAC-LD / scalable profiles need
  a sibling `scale_factor_data_rvlc()` module.
- The §4.6.13 pulse-escape reconstruction loop's *upstream
  dependency*. Round 194 lands
  `swb_offset::apply_pulse_data(x_quant, fs_index, pulse_data)`,
  which folds the per-pulse `±pulse_amp` fix-up into a long-window
  `x_quant` slice at the running coefficient index `k =
  swb_offset_long[fs][pulse_start_sfb] + Σ pulse_offset[i]`. The
  fix-up itself is complete; what's still owed is the
  *already-Huffman-decoded* spectrum that it operates on, which
  arrives with `spectral_data()`.
- The §4.6.9.3 `tns_decode_coef` LPC reconstruction and the
  §4.6.9.3 `tns_ar_filter` all-pole pass. The `tns_data()` block
  is parseable / writable bit-for-bit, but applying the all-pole
  filter to the dequantised MDCT spectrum requires both the LPC
  reconstruction (signed conversion + `iqfac` arcsine inverse-
  quantisation + Levinson-style conversion to LPC coefficients)
  and a real spectral context.
- The full per-AOT IMDCT + windowing back-end (filterbank, TNS
  reconstruction, PNS / IS pair routing).
- The decoder-side channel-element body walker that drives
  `ics_info` → `section_data` → `scale_factor_data` → optional
  `pulse_data` / `tns_data` / `gain_control_data` → `spectral_data`
  inside the per-channel `Element::ChannelElement` branch — the
  Phase 1 walker still stops after the 4-bit `element_instance_tag`
  and the caller must advance the bit-reader past the body manually.
- All decode / encode runtime wiring. The `register()` entry point
  installs no `Decoder` or `Encoder` into the runtime context yet,
  and the factory functions return `Error::NotImplemented`.
- The `extensionFlag3 == 1` body inside `GASpecificConfig` (Table 4.1
  reserves it as "tbd in version 3"; round 177 surfaces the bit but
  rejects the body via `Error::UnsupportedAscExtensionFlag3`).
- The `ErrorProtectionSpecificConfig()` body that follows when
  `epConfig == 2` or `epConfig == 3` (round 177 rejects via
  `Error::UnsupportedEpConfig(u8)` rather than silently returning a
  partial ASC).
- The implicit-SBR / PS signalling path **via the FIL
  `extension_payload`** carried inside the raw_data_block stream.
  Round 192 already covers the ASC-side Table 1.15 trailing
  `syncExtensionType == 0x2b7` probe; the FIL-side detection
  (ADTS HE-AAC's dominant form) still needs the SBR QMF / patching
  back-end this crate does not yet provide.
- LATM/LOAS transport (ISO/IEC 14496-3 §1.7.3).
- ADTS CRC-16 validation.
- The `sect_sfb_offset` table — derived per-frame from
  `swb_offset_long_window[]` / `swb_offset_short_window[]` (which
  are now in place as of round 194) and the runtime grouping mask
  from `ics_info()`. The derivation belongs in the per-frame
  channel-element body walker that consumes `ics_info()` +
  `section_data()` + `spectral_data()` together; the building-
  block tables themselves are landed.

## Provenance

No external implementation (FFmpeg's `libav*` family, FDK-AAC, FAAD,
Nero AAC, libfaac, …) was consulted at any stage. Every numeric
constant, bit-layout decision, and clause reference in this crate
is sourced from the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 PDFs
under `docs/audio/aac/`.

## License

MIT — see [LICENSE](./LICENSE).
