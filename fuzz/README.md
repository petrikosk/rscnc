# Fuzz targets

[cargo-fuzz](https://github.com/rust-fuzz/cargo-fuzz) targets for the text
front end and the offline pipeline. Every target only checks that arbitrary
input is either processed or rejected with an error, never a panic.

| Target     | Input → stages                                                                 |
|------------|--------------------------------------------------------------------------------|
| `lex`      | text → `rscnc::gcode::lex`                                                     |
| `decode`   | text → `Decoder::decode_source` (default config, and with auxiliary axes)      |
| `pipeline` | text → decode → `CornerRounding` / `CornerSmoothing` / `ToolRadiusCorrection` → `LookAhead::plan` |

The crate is a workspace of its own and is not built with the main workspace.

## Running

cargo-fuzz needs a nightly toolchain:

```sh
cargo install cargo-fuzz
cargo +nightly fuzz run decode
```

Run from the repository root (or from `fuzz/`). A run goes on until it finds
a crash or is interrupted; bound it with libFuzzer options after `--`:

```sh
cargo +nightly fuzz run pipeline -- -max_total_time=60
```

To start from real programs, list a seed directory after the corpus
directory (the first directory is the one libFuzzer writes to):

```sh
mkdir -p fuzz/corpus/decode
cargo +nightly fuzz run decode fuzz/corpus/decode samples/gcode
```

## Crashes

A crashing input is saved under `fuzz/artifacts/<target>/`. Reproduce and
shrink it with:

```sh
cargo +nightly fuzz run decode fuzz/artifacts/decode/crash-<hash>
cargo +nightly fuzz tmin decode fuzz/artifacts/decode/crash-<hash>
```

The corpus, the artifacts and the build directory are not checked in.
