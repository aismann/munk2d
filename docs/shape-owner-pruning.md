# Same-body broad-phase pruning

The complete implementation is **included in every build**, including direct-source
builds. There is no build flag or user-facing runtime setting:

```sh
cmake -S . -B build -DBUILD_DEMOS=OFF -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

This is shared C broad-phase code, independent of the ordinary, AVX2, Wasm SIMD
and ARM NEON contact solvers. It does not change their arithmetic, iterations,
precision or threading. It adds no public API.

## What is skipped

Shapes on the same body cannot collide with one another. A private BBTree
internal node may cache a common, non-NULL owner for its entire subtree. An
eligible leaf query can skip that subtree before descending into individual
shape pairs. Bounds, topology and the existing cached-pair lifecycle remain
independent of this metadata.

This mainly benefits overlapping compound bodies. Single-shape and sparse
workloads should not be expected to improve. All public spatial queries remain
unpruned, including point, bounding-box and segment queries.

## Maintenance and eligibility

Owner propagation stops when a cached value converges. Bounds are still updated
all the way to the root: owner convergence must not stop bounds maintenance.

For cpSpace shape trees, ownership is body identity. A shape is eligible when its
existing `next` or `prev` attachment link is non-NULL. On a well-formed body list,
this is equivalent to membership in a list of at least two shapes, without
following the body's list-head pointer. An unlinked shape conservatively declines
pruning, including during removal callbacks.

cpSpace opts into per-leaf eligibility caching. Shape attachment cannot change
while its collision traversal is locked, so eligibility remains stable throughout
one leaf's traversal and callbacks. This is not a general mutable-callback
assumption for arbitrary private BBTree users.

## Deferred bootstrap

cpSpace's linked dynamic/static trees start with dormant owner metadata. They
still perform ordinary bounds/topology/pair work. The first eligible query
rebuilds both linked roots before consuming ownership metadata. The rebuild is
iterative postorder traversal through existing parent/child links, uses constant
auxiliary storage and allocates nothing. Cold/noinline annotations keep this
one-time work outside the hot query path on supporting compilers.

Bootstrap is **not free**: it is O(number of nodes), and can introduce a one-time
latency spike when a compound body first appears. Activation is sticky; removing
all compounds does not turn metadata maintenance off again. Dormant internal
nodes still initialize their owner slots to NULL.

## Private BBTree contracts

These helpers are internal implementation interfaces, not public spatial-index
APIs:

- Owner keys must remain stable while an object is indexed.
- A non-NULL query key must equal the corresponding owner key; NULL may
  conservatively decline pruning.
- Linked trees must share configuration. Configuration is restricted to empty
  linked trees; other spatial-index implementations are not configured.
- Generic configured trees remain eager by default. Query eligibility is
  refreshed across callbacks unless the caller explicitly promises stable,
  side-effect-free eligibility for one leaf traversal.
- Deferred bootstrap is an explicit private opt-in requiring pure owner keys.
  Both linked roots become valid before pruning begins.

The differential tests cover mutable generic eligibility, nested public queries,
callback order/IDs, linked roots, node reuse, static/dynamic transfers, sleeping,
reindexing/optimization, empty/single-leaf/deep trees and allocation denial during
bootstrap. Shape-list tests exercise the actual cpSpace predicate.

## Storage and compatibility

Owner fields belong to private BBTree storage and are always compiled in.
Measured prototype configurations retained their existing Node/Pair sizes; tree
configuration storage increased slightly. This is not a promise that all pointer
widths, custom timestamp types, compilers or allocator layouts have identical
memory costs. Requested allocation checks and 64-bit timestamp tests are part of
validation. RSS and downstream allocator overhead are separate concerns.

There are no product configuration switches for this optimization. The old
research switches and the temporary feature gate have been removed.

## Performance evidence and limits

The precursor prototype's six-round measurements versus production showed:

| Runtime | Multifixture simulation time | 26-scene geometric mean |
|---|---:|---:|
| S22 ARM NEON | -46.15% | -2.26% |
| Server AVX2 | -37.01% | -1.53% |
| Node Wasm SIMD | -26.85% | -1.21% |

Negative means less time. These are **precursor results**, not final-patch
qualification. Ordinary solvers also benefited. Small regressions remained; no
main-scene median exceeded the predeclared 3% limit. Node is not a browser,
shared-server noise limits small conclusions, and these numbers do not establish
an ISA-specific cause.

The original Wasm Multifixture research target was a 30% gain; the precursor
missed it while passing every regression limit. Acceptance of the smaller Wasm
win is explicit. Final-patch validation retains the 30% native/ARM target and
uses a predeclared 25% Wasm target, with the same correctness, aggregate,
per-scene regression and allocation limits. Historical failures are not relabeled.

Consolidation can change code generation or placement even without changing
engine behavior. The final implementation therefore needs fresh measurements,
not merely the precursor's results. See the pull request for exact source
revision, final validation results, reproduction commands and limitations.
