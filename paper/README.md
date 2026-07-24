# Paper: AI-Assisted Aircraft Conceptualization

LaTeX sources for the manuscript *"AI-Assisted Aircraft
Conceptualization: Preliminary Design of a Tail-Sitter UAV"*
(Tuncer, Güneş, Uçler).

## Build

```bash
latexmk -pdf main.tex          # or: make
```

CI (`.github/workflows/paper.yml`) rebuilds the PDF on every push/PR
touching `paper/` and uploads it as the `paper-pdf` workflow artifact;
tagged releases attach `vbat-paper-<version>.pdf` to the GitHub
Release (`release.yml`).

Requires a standard TeX Live or MiKTeX distribution (pdflatex, natbib,
tikz, booktabs, tabularx — all in the default install). No LaTeX
toolchain was available on the authoring machine, so run the first
build locally and fix any residual typos it flags.

## Layout

- `main.tex` — preamble, title, abstract, acknowledgements
- `sections/` — one file per section
- `references.bib` — bibliography (from the draft's reference list +
  the Claude Code tool citation)
- `figures/` — **copied from the pipeline outputs** (`assets/`,
  `out/`); regenerate there and re-copy after a design change, do not
  edit here

## Editorial state

Open items are marked in red in the PDF via `\todopaper{...}`:

- affiliations for Ahmet Güneş and Çağlar Uçler
- final check of process metrics if more commits land before submission
  (commit counts, AI co-authorship share, dates were extracted from the
  git history on 2026-07-24)

Numbers in the case-study tables come from the v0.5.2 design snapshot
(`CHANGELOG.md`, `out/` handoffs) and the git history; update them if
the design point moves.
