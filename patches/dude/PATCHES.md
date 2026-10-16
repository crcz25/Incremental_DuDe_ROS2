# Local patches to DuDe

**Do not apply the leak patches without dude-0 and dude-0b.** Without them,
DuDe's output depends on the heap layout.

- **Origin:** DuDe 2D (dual-space decomposition), `Third_Party/dude_final`.
  Liu, Xi and Lien, "Dual-Space Decomposition of 2D Complex Shapes",
  CVPR 2014. Upstream page: http://masc.cs.gmu.edu/wiki/Dude2D (not
  downloaded here; the copy in this repository is the reference).
- **Licence:** the files say "Copyright 2007-2014 by Jyh-Ming Lien and
  George Mason University; see the file LICENSE". No LICENSE file is in the
  tree or in the git history. The project owner confirmed on 2026-10-15
  that DuDe may be modified. Every copyright header is kept.
- **Root cause of dude-0 and dude-0b:** DuDe's result depended on two
  orders that come from the heap layout:
  - the order of pointers, in DuDe's pointer-keyed sets and in a pair sort
    that broke ties by pointer;
  - the order in which CGAL's constrained Delaunay triangulation returns
    its edges (vendored CGAL is not changed).

  Both were stable only while DuDe leaked: the heap only grew, so it
  followed the creation order. Once memory is released and reused, a
  second decomposition in the same process could choose other cuts.
  dude-0 and dude-0b make the result independent of both orders. The
  geometry of the reference maps equals that of unpatched DuDe; the order
  of some lists and the orientation of some segments differ (by design).
- **Expected difference from unpatched DuDe (DP-13, accepted 2026-10-15):**
  DuDe's geometry depends on the orientation of the CDT edges on some
  maps. On the evaluation maps:
  - at τ 54 px, 5 of 40 inputs differ from every unpatched layout
    (`Freiburg101_scan` and its furniture version, `lab_a_scan`,
    `lab_b_scan_furnitures`, `lab_d_scan_furnitures`);
  - at τ 50 px, none differs.

  The changes are a few cuts or diagonals and small region changes (see
  `docs/fixes/dude_round2_report.md`, E4). The project owner chose to keep
  dude-0b: the result is the same in every process.
- **Marker:** each change carries a comment `PATCH(dude-<n>)`.

## How to apply

The patches are in `git format-patch` form, with paths relative to the
DuDe root (`dude_final/`). They are already applied in this repository,
and no build step applies them. To apply them to a new, unmodified copy of
DuDe, in this order:

```sh
cd dude_final        # outside any other git repository
for p in /path/to/patches/dude/0*.patch; do
  git apply --check "$p" && git apply "$p" || break
done
```

Checked on 2026-10-15 against the unpatched DuDe of this repository
(commit f74508b), in a copy outside any git repository:

- 0001 applies alone;
- 0002 applies after it;
- 0003 to 0011 apply after both;
- 0012 to 0015 (round 2) apply after 0001 to 0011, in this order;
- the result of 0001 to 0015 equals the patched DuDe byte for byte, and
  `skeleton/SE2d.cpp`, `skeleton/SE2d.h`, `dude_use.cpp` and
  `holecutconcavity.cpp` keep their CRLF line endings.

Run it outside any other git repository: inside one, `git apply` reads the
paths relative to that repository's root and silently skips them.

## The patches

The bytes are LeakSanitizer totals (the sanitizer build) of the H-03 test
alone (`DudeWrapper.H03_RepeatedDecompositionsGiveTheSameContours`: 50
decompositions of two rooms and a corridor), before → after, unless stated
otherwise. The proof of each is the H-03 test in the sanitizer build, and
the `DudeReference.*` tests (`test_dude_reference`), which must stay
unchanged. Patches 0003 to 0011 were checked one at a time, in the order
they were written, before 0001 and 0002 existed. Their release sites do
not depend on the order.

| # | ID | Title | Leak (stack, innermost first) | Bytes fixed | Owner and release | Marker | Test |
|---|----|-------|-------------------------------|-------------|-------------------|--------|------|
| 0001 | dude-0 | Order DuDe vertices and BPCs by creation, not by address | — | — | `dude_seq` (`polygon.h`): one process-wide `std::atomic<uint64_t>` (relaxed `fetch_add`). A copy gets a new number; an assignment keeps the target's. `ply_vertex` and `c_BPC` carry one. `ply_vertex_seq_less` (null first) orders the four `set<ply_vertex *>` of `dude.cpp` (`combineMinimumPnts`, `collectPMs`, the outer and the total feature points). `reorganize_kids_f` breaks ties by the kids' number. | `PATCH(dude-0)` | `DudeReference.ASecondPassGivesTheSameResult`, `TheExactDecompositionIsUnchanged` |
| 0002 | dude-0b | DuDe takes CGAL's CDT edges in a fixed order | — | — | `c_cdt::build` (`cdt2.cpp`) orients each unconstrained edge from its smaller end point (x, then y) and sorts with `std::stable_sort` by (x1, y1, x2, y2), exact doubles, NaN last. The test hook `g_cdt_edge_test_hook` (null) may reorder the edges first or skip the sort. | `PATCH(dude-0b)` | `TheExactResultDoesNotDependOnTheCdtEdgeOrder`, `TheGeometryDoesNotDependOnTheUnsortedCdtEdgeOrder` |
| 0003 | D1 | Release the triangulation arrays of `c_polygon::triangulate` | Direct: `c_polygon::triangulate` ← `enclosed` ← `c_dude::isDiagonal` ← `c_cdt::build` (`ringVN`, `V`, `T`) | 1,866,000 → 1,808,000 (58,000) | The function itself: `delete[]` after FIST, and `ringVN` on both early returns (`c_polygon` and `c_ply`). | `PATCH(dude-1)` | H-03 test, DudeReference |
| 0004 | D2 | A BPC releases its kids | Direct: `c_dude::decompose_bridge` (`new c_BPC`, 100 × 184 B) | 1,808,000 → 1,789,600 (18,400) | The parent BPC: `~c_BPC` deletes its `m_kids`. Each kid is in exactly one `m_kids`; no `c_BPC` is copied. | `PATCH(dude-2)` | H-03 test, DudeReference |
| 0005 | D3 | The skeleton extractor frees the se_m that splits replaced | Direct: `iterativeDecompose` (`new se_m`, 50 × 112 B) | 1,789,600 → 1,784,000 (5,600) | `SE2d::destroy()`: an se_m that left `todo_list` but stays in `ply2MouthMap` is deleted once; the map is cleared. | `PATCH(dude-3)` | H-03 test, DudeReference |
| 0006 | D4a | Free the visualization pocket copies of a BPC | Indirect: `c_ply::addVertex` ← `c_BPC::seg2Ply` ← `c_BPC::determineConcavity_real` | 1,784,000 → 1,066,400 (717,600) | `determineConcavity_real` destroys `tmpply` and `tmpChildren` at its end (no other return). | `PATCH(dude-4a)` | H-03 test, DudeReference |
| 0007 | D4b | Free the chains of each approximation pass | Indirect: `approximate` ← `approximateMedthod` ← `determineConcavity_real` / `collectConvexFeatPnts` (`new c_chain`) | 1,066,400 → 935,200 (131,200) | `approximate()` deletes the chains of each pass after their last use. | `PATCH(dude-4b)` | H-03 test, DudeReference |
| 0008 | D4c | Free the pocket copy of each bridge | Indirect: `c_ply::addVertex` ← `c_dude::create_ply_from_bridge` ← `decompose_bridge` | 935,200 → 384,000 (551,200) | `decompose_bridge` destroys `ply` after `delete dude2`; the kids use the original vertices through `vmap`. | `PATCH(dude-4c)` | H-03 test, DudeReference |
| 0009 | D4d | A c_dude frees its simplified polygon | Indirect: `c_ply::addVertex` ← `c_dude::buildSimpliedPly` ← `create_simplified_and_reduced` | 384,000 → 6,800 (377,200) | `~c_dude` destroys `m_simpilifed_P` (`m_reduced_P` shares the input's vertices: only cleared). A simplified ply with fewer than 3 vertices is destroyed where it is dropped. | `PATCH(dude-4d)` | H-03 test, DudeReference; whole `test_inc_dude_node` and `test_evaluation_node` in the sanitizer build |
| 0010 | D4e | The skeleton extractor frees the mouths | Indirect: `newMouth` ← `updateMouth` ← `decomposePolygonUsingCuts` | 6,800 → 0 | `SE2d::destroy()` collects the mouths of every se_m it frees and deletes each once, after the se_m. | `PATCH(dude-4e)` | H-03 test (no leak left) |
| 0011 | D4f | A c_dude frees the holes it removed as ignorable | `c_ply::copy` ← `c_polygon::copy` ← `Dual_decompose` (the vertices of a hole that `buildBPC` erased) | `test_evaluation_node`: 7,360 → 0 (5 holes × 8 vertices) | `c_dude` keeps the erased ply in `m_ignored_holes` and destroys it in `~c_dude` (`P.erase` drops it from P, so nothing else frees its vertices; reason corrected by 0015). | `PATCH(dude-4f)` | whole `test_evaluation_node` in the sanitizer build |
| 0012 | E1 (DP-1) | A c_dude keeps the BPCs of its holes until it ends | Use after free, not a leak: READ in `c_BPC::getSource1` ← `c_dude::is_bridge_breaking` ← `isDiagonal` ← `c_cdt::build`, freed by `~c_dude` ← `decompose_hole` ← `buildBPC_from_hole` (Freiburg52 at τ 20 px; all 19 / 24 unpatched crashes on the evaluation maps at τ 54 / 50 px) | — | The pointer: `concavity_bpc` of the hole vertices, set by the BPCs of `decompose_hole`'s local `c_dude`. `decompose_hole` moves those BPCs (and their kids) to the outer `c_dude`'s new `m_hole_bpcs`; `~c_dude` deletes them. | `PATCH(dude-5)` | `TheTau20DecompositionIsUnchanged`, `TheTau20ExactDecompositionIsUnchanged`, `NoVertexPointsAtAFreedBpcAfterBuild` (sanitizer build) |
| 0013 | E2 (DP-6) | Free the polygons that decomposePolygonUsingCuts split | Direct: `c_dude_use::decomposePolygonUsingCuts` ← `iterativeDecompose` ← `Dual_decompose` (+ the `c_polygon` vectors) | `test_dude_reference`: 1,287,552 → 0 (from these polygons) | `SE2d::splitPolygons` (new) collects each polygon an out-out cut cleared, except the caller's (`se.initPolygon`); freed at the end of `SE2d::destroy()`, after `ply2MouthMap` is cleared and after the se_m and mouths. | `PATCH(dude-6)` | `test_dude_reference` under LeakSanitizer; peak RSS unchanged |
| 0014 | E7 (DP-11) | Free the hole-cut tree of decompose_hole | Direct: `getCutDiagRoot` ← `decompose_hole` (+ `buildHoleDiagonalHierarchy`, vectors) | `test_dude_reference`: 528 → 0; evaluation maps at τ 20 px up to 6.7 KB → 0 | `getCutDiagRoot` clears `root2`'s child list (root1 owns them) and deletes `root2` as a node; `decompose_hole` deletes the returned root (`~HoleDiagonal` frees the children; each node is in exactly one `childHoleDiags`). | `PATCH(dude-7)` | `test_dude_reference` under LeakSanitizer |
| 0015 | E6 (DP-9) | Correct the D4f comment | — | — | Comment only (`dude.h`). | `PATCH(dude-4f)` (kept) | — |

Correction to 0011: its comment said `hole_PMs` may still point at the
erased hole. In fact `decompose_hole` returns before it adds anything to
`hole_PMs` for an ignorable hole. 0015 (round 2) corrects the comment.

## Deviations in round 2 from the planned fixes

- **0012 (E1), option B without the reset in the outer destructor.** The
  plan was to keep the hole BPCs in the outer `c_dude` and to reset the
  vertices' `concavity_bpc` in its destructor before freeing the BPCs.
  That reset is itself a heap-use-after-free:
  - `~DuDe_OpenCV_wrapper` frees the input polygon's vertices
    (`decomposed_polygon->destroy()`) in its body, before its `c_dude`
    member is destroyed.
  - A variant with the reset showed it under ASan: WRITE in
    `c_dude::~c_dude` ← `~DuDe_OpenCV_wrapper`, freed by
    `c_ply::destroy` ← `c_polygon::destroy` ← `~DuDe_OpenCV_wrapper`.

  So `~c_dude` frees the BPCs without touching vertices. This is safe
  because every vertex that outlives a `c_dude` (the sub-polygons of
  `iterativeDecompose`) is reset by `reInitializePolygon` first.
  `~c_BPC` reads no vertex. Option A (null the pointers in
  `decompose_hole`) gave identical dumps on all reference maps.
- **0013 (E2), the owner is `SE2d`.** The plan suggested a `c_dude_use` or
  the wrapper.
  - The `c_dude_use` objects are locals of `iterativeDecompose` and do not
    outlive `SE2d`.
  - The wrapper cannot see the split polygons without a change to DuDe's
    interface.
  - `SE2d` is passed to `decomposePolygonUsingCuts`, holds the map whose
    keys they are, and is destroyed by the wrapper after the result
    coordinates are copied.
  - Freeing them at the end of `SE2d::destroy()`, after the map is
    cleared, removes the address-reuse risk by timing.

## Round 2 line count

Round 2 changes 30 lines of DuDe (0012: 11, 0013: 11, 0014: 4, 0015: 4).
Round 1 changed 147 lines. The net difference of the patched DuDe from
the unpatched one is +161 / −12.

## Not DuDe patches

- `src/wrapper.cpp` (`include/wrapper.cpp` before L-17) clears DuDe's
  global `g_points` (`poly_approx.cpp`) with the decoration (E3, DP-5;
  first-party, no DuDe change).
- `PERMISSION.md` records the origin of DuDe and the permission to modify
  it.

The first-party side (not a DuDe patch): `src/wrapper.cpp` clears
DuDe's global `draw_decoration` (`allFeaturePMs`, `allFeatureConcavities`)
around each decomposition and in its destructor (D5, P4-N8).
