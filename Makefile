.PHONY: check
check:
	# f32 and f64 are mutually exclusive avian3d backends; check each separately.
	cargo check --workspace --features "f32"
	cargo check --workspace --features "f64"
	cargo check --workspace --features "f32,debug-plugin"

.PHONY: clippy
clippy:
	cargo clippy --workspace --features "f32"
	cargo clippy --workspace --features "f64"

.PHONY: test
test:
	# f32 and f64 are mutually exclusive avian3d backends; test each separately.
	cargo test --workspace --features "f32"
	cargo test --workspace --features "f64"
	cargo test --workspace --features "f32,debug-plugin"

.PHONY: doc
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc -p avian_fdm --no-deps --features "f32,debug-plugin"
	RUSTDOCFLAGS="-D warnings" cargo doc -p avian_fdm_j3cub_jsbsim --no-deps --features "f32"

.PHONY: build
build:
	cargo build --workspace --release --features "f32"

.PHONY: jsbsim
jsbsim:
	cargo test -p avian_fdm_j3cub_jsbsim --features "f32" -- jsbsim --nocapture

.PHONY: bench
bench:
	cargo test -p avian_fdm --features "f32" --test perf_limits -- --nocapture

.PHONY: ci
ci: check clippy test doc build
