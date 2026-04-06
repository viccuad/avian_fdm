.PHONY: check
check:
	cargo check --all-features

.PHONY: clippy
clippy:
	cargo clippy --all-features

.PHONY: test
test:
	# avian3d requires an explicit f32/f64 backend (we set default-features = false),
	# so bare `cargo test` and `--no-default-features` do not compile.
	# Test with the two meaningful feature combinations instead.
	cargo test --features "presets"
	cargo test --all-features

.PHONY: doc
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features

.PHONY: build
build:
	cargo build --release --all-features

.PHONY: jsbsim
jsbsim:
	cargo test --features "presets" -- jsbsim --nocapture

.PHONY: bench
bench:
	cargo test --features "presets" --test perf_limits -- --nocapture

.PHONY: ci
ci: check clippy test doc build
