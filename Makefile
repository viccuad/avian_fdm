CARGO_FLAGS ?= --locked

.PHONY: check
check:
	# Default features (parry-f32, debug-plugin) workspace-wide. Then parry-f64
	# with and without debug-plugin. parry-f32 and parry-f64 are mutually exclusive.
	cargo check $(CARGO_FLAGS) --workspace
	cargo check $(CARGO_FLAGS) --workspace --no-default-features --features "parry-f64,debug-plugin"
	cargo check $(CARGO_FLAGS) -p avian_fdm --no-default-features --features "parry-f64"

.PHONY: clippy
clippy:
	cargo clippy $(CARGO_FLAGS) --workspace
	cargo clippy $(CARGO_FLAGS) --workspace --no-default-features --features "parry-f64,debug-plugin"
	cargo clippy $(CARGO_FLAGS) -p avian_fdm --no-default-features --features "parry-f64"

.PHONY: test
test:
	# Default features workspace-wide, then parry-f64 core-only.
	cargo test $(CARGO_FLAGS) --workspace
	cargo test $(CARGO_FLAGS) -p avian_fdm --no-default-features --features "parry-f64"
	
.PHONY: fmt
fmt:
	cargo fmt

.PHONY: doc
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc $(CARGO_FLAGS) -p avian_fdm --no-deps --document-private-items
	RUSTDOCFLAGS="-D warnings" cargo doc $(CARGO_FLAGS) -p avian_fdm_j3cub_jsbsim --no-deps

.PHONY: build
build:
	cargo build $(CARGO_FLAGS) --workspace --release

.PHONY: jsbsim
jsbsim:
	cargo test $(CARGO_FLAGS) -p avian_fdm_j3cub_jsbsim -- jsbsim --nocapture
	
.PHONY: j3cub-jsbsim
j3cub-jsbsim:
	cargo run $(CARGO_FLAGS) -p avian_fdm_j3cub_jsbsim --example j3cub_visual --release

.PHONY: bench
bench:
	cargo test $(CARGO_FLAGS) -p avian_fdm --test perf_limits -- --nocapture
	
.PHONY: coverage
coverage:
	cargo $(CARGO_FLAGS) llvm-cov --workspace --ignore-run-fail --html --output-dir coverage/

.PHONY: ci
ci: check clippy test doc build
