# DuDe: origin and permission to modify

This note records why the local copy of DuDe in `Third_Party/dude_final`
is modified (`patches/dude/`). It is a record, not a licence grant.

## Origin

- DuDe 2D, dual-space decomposition of 2D shapes: Liu, Xi and Lien,
  "Dual-Space Decomposition of 2D Complex Shapes", CVPR 2014.
- Upstream page: http://masc.cs.gmu.edu/wiki/Dude2D (not downloaded here;
  the copy in this repository is the reference).
- Earlier local edits are in the git history of this repository:
  `c7c5817` "modifying third party", `241b399` "working on bugs",
  `30906db` "format files".

## Licence situation

- The DuDe file headers say "Copyright <years> by Jyh-Ming Lien and
  George Mason University" (years from 2007-2008 to 2010-2015) and
  "See the file "LICENSE" for more information" (19 files).
- One bundled file, `src/simple_svg_1.0.0.h` (Mark Turney, 2010), has
  its own licence text in its header (New BSD).
- No LICENSE file is in the tree or in the git history.
- The package licence in `package.xml` is `TODO` (L-09, Phase 6; not
  changed here).

## Permission

On 2026-10-15 the owner of this repository confirmed that DuDe may be
modified for this project. Every copyright header is kept, and each
change carries a `PATCH(dude-<n>)` marker (see `PATCHES.md`).

This confirmation is the only basis for the changes. It does not grant a
licence to DuDe, and it does not replace one from the copyright holders.

## Recommendation

Ask the upstream authors (Jyh-Ming Lien, George Mason University) for an
explicit licence for DuDe, or for the missing LICENSE file, before the
code or the patches are published or distributed.
