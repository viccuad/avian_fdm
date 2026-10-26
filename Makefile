.PHONY: check
check:
	# Default features (parry-f32, debug-plugin) workspace-wide. Then parry-f64
	# with and without debug-plugin. parry-f32 and parry-f64 are mutually exclusive.
	cargo check --workspace
	cargo check --workspace --no-default-features --features "parry-f64,debug-plugin"
	cargo check -p avian_fdm --no-default-features --features "parry-f64"

.PHONY: clippy
clippy:
	cargo clippy --workspace
	cargo clippy --workspace --no-default-features --features "parry-f64,debug-plugin"
	cargo clippy -p avian_fdm --no-default-features --features "parry-f64"

.PHONY: test
test:
	# Default features workspace-wide, then parry-f64 core-only.
	cargo test --workspace
	cargo test -p avian_fdm --no-default-features --features "parry-f64"
	
.PHONY: fmt
fmt:
	cargo fmt

.PHONY: doc
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc -p avian_fdm --no-deps
	RUSTDOCFLAGS="-D warnings" cargo doc -p avian_fdm_j3cub_jsbsim --no-deps

.PHONY: build
build:
	cargo build --workspace --release

.PHONY: jsbsim
jsbsim:
	cargo test -p avian_fdm_j3cub_jsbsim -- jsbsim --nocapture
	
.PHONY: j3cub-jsbsim
j3cub-jsbsim:
	cargo run -p avian_fdm_j3cub_jsbsim --example j3cub_visual --release

.PHONY: bench
bench:
	cargo test -p avian_fdm --test perf_limits -- --nocapture

.PHONY: ci
ci: check clippy test doc build
