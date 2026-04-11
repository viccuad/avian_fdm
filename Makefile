.PHONY: check
check:
	cargo check --all-features --workspace

.PHONY: clippy
clippy:
	cargo clippy --all-features --workspace

.PHONY: test
test:
	# avian3d requires an explicit f32/f64 backend (we set default-features = false),
	# so bare `cargo test` and `--no-default-features` do not compile.
	cargo test -p avian_fdm --features "f32"
	cargo test -p avian_fdm --all-features
	cargo test -p avian_fdm_j3cub_jsbsim

.PHONY: doc
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features --workspace

.PHONY: build
build:
	cargo build --release --all-features --workspace

.PHONY: jsbsim
jsbsim:
	cargo test -p avian_fdm --features "f32" -- jsbsim --nocapture

.PHONY: bench
bench:
	cargo test -p avian_fdm --features "f32" --test perf_limits -- --nocapture

.PHONY: ci
ci: check clippy test doc build
