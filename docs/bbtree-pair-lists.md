# BB-tree cached-pair endpoint lists

The BB-tree keeps two intrusive lists per leaf: pairs where it is endpoint `a`, and pairs where it is endpoint `b`. The latter owns the cached collision callback. Both lists reuse the pair's existing `Thread` fields; there is no additional pair record allocation and no new solver/backend option.

Previously a leaf traversed one combined list, calling the collision callback for `b` entries and merely following the next link for `a` entries. In an Android ARM64 reference-suite profile, 47% of those visits were non-owner links. With separate endpoint lists, the cached pass walks only the owned `b` list. Insert/unlink also know their endpoint role and no longer inspect each neighbor to select its thread.

## Ordering, lifetime and bounds

Insertion at each role's head preserves the order obtained by filtering the old combined list for that role. Removal unlinks the opposite endpoint's list and recycles each pair exactly once. The recycle order may differ, but callback order and cached collision IDs are preserved. Pair storage is shared through the existing master-tree pool, including static/dynamic tree relationships.

The existing velocity-dependent bounding-box branch must treat a leaf as empty only when **both** endpoint lists are empty. No padding coefficients, collision tolerances, candidate filtering rules or generic spatial-index APIs are changed. The tree does not assume its objects are cpShapes or add body-specific filtering.

The independent mixed-list model test checks pointer topology, previous/next links, ownership-filtered callback order, cached IDs and repeated insert/clear/reuse across multiple pool buffers. A deterministic public trace additionally exercises static/dynamic indices, removal/reinsertion, reindexing, optimization and velocity-derived bounds. The trace can be compiled against an older tree source using the test-only `CP_BBTREE_PAIR_SOURCE` include override for cross-revision comparison.

## Memory

Private `Node` gains one pointer; private `Pair` is unchanged. On the tested 64-bit double builds Node grows **64 → 72 bytes**, while Pair remains **56 bytes**. On tested 64-bit float builds Node grows48 →56 bytes. These are private storage layouts, not changes to public structs or API ABI. Node pool capacity per fixed-size buffer decreases accordingly; an index with many objects can use more memory. Do not describe the optimization as free or universally beneficial.

## Measurements

Merged base86e6851 vs the split-list candidate, same per-space solver and compiler policy on each side. Research-only benchmark drivers use CLOCK_MONOTONIC whole-step wall time; that timer change is not part of the production patch. Initialization/cleanup are excluded. No HastySpace threads or SIMD kernel arithmetic changes are involved.

Galaxy S22 SM-S901B/Exynos2200, Android16, NDKr27d/API28, ARM64 doubles, Release/O3/default LTO, same Cortex-A710 core pair (mask0x30). All26 scenes, smoke and reference profiles, six paired rounds with alternating order, one untimed warmup plus one measured fresh-world simulation per scene;1248 measured samples. Negative values mean less time/faster.

| Solver | Smoke geometric time change | Reference geometric time change | Reference summed median scene time |
|---|---:|---:|---:|
| Baseline | −1.74% | −3.20% | −6.99% |
| Packet NEON | −1.84% | −3.24% | −7.10% |

The geometric column gives equal weight to per-scene median paired ratios; the summed column is time-weighted. Reference examples: baseline AddPair −31.35%, Multifixture −11.21%, Diagonal −9.82%; NEON AddPair −30.36%, Multifixture −11.12%, Diagonal −9.32%. Smaller regressions also occur: reference SlideConstraints +1.45% baseline/+1.70% NEON, among others. These are one device/core-pair/compiler results, not a universal speedup guarantee.

A bounded x86 check on the Intel server also improves the aggregate: three paired rounds, all26 scenes, one pinned logical CPU, default AVX2 build. Geometric smoke/reference time differences: baseline −3.10%/−2.97%, AVX2 −2.71%/−3.56%. The server is shared infrastructure and the shorter experiment is lower-confidence than the dedicated-device campaign; don't infer exact cross-platform speedup ratios from it.

All26 smoke/reference checkpoint captures match the unchanged tree byte-for-byte on the S22 for baseline and NEON. The role-list model/public trace matches the original tree for tested double/float configurations; ordinary suites, sanitizer and portability checks supplement rather than replace that comparison.
