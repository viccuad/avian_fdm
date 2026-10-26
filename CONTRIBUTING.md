## Releasing

1. Copy the corresponding ./migration-guides/0.X-to-main.md into its
   own version, and remove the first 2 paragraphs.
2. Bump version, merge PR.
3. Edit GH release draft as needed (version, content).
4. Tag.
5. `cargo publish --dry-run`.
