## Code layout

- `src/`: The `avian_fdm` crate library.
- `tests/`: Integration tests.
- `avian_fdm_j3cub_jsbsim/`: Examples used as e2e test, providing a comparison
  with the J3Cub model of JSBSim, and a visual example of such J3Cub. It acts
  as a validating target, and the heavy JSBSim test fixtures stay out of the
  published `avian_fdm` crate. Its content is derived from JSBSim, and thus has
  a different license (GPL-3.0 vs L-GPL-3.0-or-later).
  This crate has extra dependencies as needed.
- `migration-guides/`: Provided for breaking changes between versions.

## Code conventions

The majority of code conventions are enforced by gating CI jobs (format,
clippy, test coverage, enforce docstrings, pub(crate), etc). Please use
the provided `make` targets locally before submitting a PR.
The project is configured to emit compiler warnings for an easy development
loop (see provided make targets), and all those warnings are treated as errors
in CI by passing `RUSTFLAGS="-D warnings".

Most questions can be answered by matching the conventions and style of nearby
code.

For Bevy related patterns, see
https://github.com/tbillington/bevy_best_practices.

## Documentation

The project favors crate and inline documentation as close as makes sense to
[ASD-STE100 aeronautical Simplified Technical
English](https://www.asd-ste100.org/).

## Commit, PRs, issues conventions

The project doesn't enforce commit conventions much as we feel that stiffles
drive-by contributions and newcomers. Commit messages should primarly explain
["why" there's a change](https://chris.beams.io/git-commit), not only "what"
change they bring. We favor [conventional
commits](https://www.conventionalcommits.org/en/v1.0.0/).

PR titles are used by
[release-drafter](https://github.com/release-drafter/release-drafter) to come
up with GitHub release drafts, as a changelog of the next yet-to-be released
version. If you can, use conventional commit style for the PR title,
particularly if your changes contain breaking changes (e.g: `fix!: foo` instead
of `fix: foo`). The PR title is always editable by maintainers, even after a
merge.

PRs from branches on the repo are autolabeled by autolabeler, depending on their
"conventional commit" title. This is not enabled for branches from forks for
security reasons.

For issues for features, please list the acceptance criteria that the issue
must fulfill for it to be closed.

## Testing

The project aims to have a code coverage at ~80%. This is judged per PR
changes as needed, and the code coverage is shown in PRs.

## Releasing

1. Copy the corresponding ./migration-guides/0.X-to-main.md into its own
   version, and remove the first 2 paragraphs.
2. Update "Version compat" table in README.md
3. Bump version, merge PR.
4. Edit GH release draft as needed (version, content).
5. Tag.
6. Approve release workflow run for trusted publishing.
