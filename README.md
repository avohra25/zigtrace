# zigtrace

A physically based Monte Carlo path tracer written in Zig using only the
standard library. It renders scenes described in a small text format to PPM
or PNG, using a SAH bounding volume hierarchy and all CPU cores. For a given
seed the output is bit-for-bit identical no matter how many threads you use.

## Features

- **Geometry**: spheres (a negative radius gives a hollow bubble), infinite
  planes, triangles, and quads (stored as two triangles).
- **Materials**: Lambertian diffuse (cosine-weighted sampling), metal with
  adjustable roughness ("fuzz"), dielectric glass/water with Fresnel (Schlick)
  and total internal reflection, and one-sided emissive area lights.
- **Camera**: thin-lens perspective camera with field of view, aperture and
  focus distance, so you get depth of field.
- **Integrator**: unidirectional path tracing with Russian roulette after a
  few bounces, a configurable maximum depth, and dropping of non-finite
  samples.
- **Acceleration**: binned surface-area-heuristic BVH built over a flat node
  array and traversed iteratively, nearest child first. Unbounded planes are
  kept out of the BVH and tested separately.
- **Parallelism**: the image is split into tiles that worker threads
  (`std.Thread`) take from an atomic counter.
- **Determinism**: a seeded xoshiro256++ RNG with an independent stream for
  every pixel.
- **Output**: HDR framebuffer, exposure, `none`/`reinhard`/`aces` tone
  mapping, gamma, binary PPM, and a self-contained PNG encoder (zlib stream
  of stored DEFLATE blocks, CRC-32 and Adler-32 from `std.hash`).
- **Scene files**: a line-based text format with named materials and error
  messages that give the line number.

## Building

Requires Zig **0.14** (tested with 0.14.1). There are no other dependencies.

```sh
zig build                          # debug build -> zig-out/bin/zigtrace
zig build -Doptimize=ReleaseFast   # use this for real renders
zig build test                     # run the unit tests
```

## Usage

```sh
zig build -Doptimize=ReleaseFast
./zig-out/bin/zigtrace render examples/spheres.txt -o spheres.png --spp 64 --threads 8
./zig-out/bin/zigtrace render examples/cornell.txt -o cornell.ppm
./zig-out/bin/zigtrace info examples/glass.txt
```

```
zigtrace render <scene.txt> [options]
  -o, --output <path>   .ppm or .png (default: out.ppm)
  --spp <n>             samples per pixel
  --width/--height <n>  resolution
  --depth <n>           maximum bounces
  --threads <n>         worker threads (default: all CPUs)
  --seed <n>            RNG seed (default: 0)
  --tile <n>            tile size (default: 16)
  --tonemap <op>        none | reinhard | aces (default: aces)
  --exposure <x>        exposure multiplier (default: 1)
  --gamma <g>           display gamma (default: 2.2)
  -q, --quiet           no statistics
```

Command-line values override the scene file's `image`, `samples` and `depth`
settings.

### Example scenes

| File | What it shows |
|---|---|
| `examples/spheres.txt` | About 320 random spheres with depth of field (stresses the BVH) |
| `examples/glass.txt` | Glass, a hollow bubble, water, rough copper, a mirrored tetrahedron, a small emitter and sky light |
| `examples/cornell.txt` | The classic Cornell box with an area light, a glass sphere and a steel sphere |

On an 8-core machine with ReleaseFast, `spheres.txt` at 600×340 and 16 spp
renders in about 0.2 s. `cornell.txt` at 400×400 and 512 spp takes about 5 s.

## Scene format

```
# comments start with '#'
image 640 360                     # default resolution
samples 64                        # default samples per pixel
depth 16                          # default maximum path length
camera from 0 1 5 to 0 0 0 up 0 1 0 fov 40 aperture 0.1 focus 5
background sky 1 1 1  0.5 0.7 1.0 # horizon colour, zenith colour
background color 0 0 0            # or a constant colour

material ground lambertian 0.5 0.5 0.5
material steel  metal 0.8 0.8 0.8 0.05   # albedo, optional fuzz in [0,1]
material glass  dielectric 1.5           # index of refraction
material lamp   emissive 4 4 4           # radiance

sphere   0 1 0  1  glass                 # centre, radius, material
plane    0 0 0  0 1 0  ground            # point, normal, material
triangle -1 0 0  1 0 0  0 1 0  steel     # three vertices
quad     -1 3 -1  2 0 0  0 0 2  lamp     # corner, edge u, edge v (front = u x v)
```

You must define a material before any shape uses it. In the `camera` line,
`up`, `fov`, `aperture` and `focus` are optional. Without `focus`, the focus
distance is the distance from `from` to `to`.

## Design notes

- **Layout.** `src/root.zig` exposes the renderer as a library module, and
  `src/main.zig` is a thin CLI on top of it. Each file is one concern: `vec`,
  `rng`, `ray`, `aabb`, `geometry`, `material`, `bvh`, `camera`, `scene`,
  `render`, `image`, `png` and `scenefile`.
- **Shapes** are a tagged union (`Shape`) dispatched with `inline else`,
  which avoids vtables and keeps hit testing easy to inline. `Hit` always
  stores a normal that faces against the incoming ray, plus a `front_face`
  flag. Dielectrics use the flag to pick η, and one-sided lights use it to
  decide whether they emit.
- **BVH.** Primitives go into 12 centroid bins on each axis, and the split
  with the lowest SAH cost wins. Nodes are 2N−1 at most, preallocated, with
  siblings next to each other, so an interior node only stores its left
  child's index. A median split is the fallback when centroids coincide, and
  depth is capped so the fixed-size traversal stack cannot overflow. Tests
  compare it against brute-force intersection: thousands of rays over random
  sphere and triangle scenes must give exactly the same `t` and material.
- **Sampling.** The Lambertian direction is `n + unit_vector()`, which is
  cosine distributed, so the BRDF·cos/pdf weight is just the albedo. Russian
  roulette continues a path with probability `min(max(throughput), 0.95)` and
  divides the survivor's weight by that probability, so the estimate stays
  unbiased.
- **Determinism.** Each pixel seeds its own RNG from `(seed, pixel index)`
  through SplitMix64. Which thread renders a tile and in what order doesn't
  matter, and a test checks that 1 and 4 threads produce identical images.
- **Correctness tests** include analytic cases: an empty scene returns the
  background exactly; a convex diffuse sphere under a uniform sky returns
  exactly its albedo; a white furnace (glass shell around a white core) stays
  close to 1; and lights seen directly return their emission exactly. The
  tests also check Snell's law, Schlick at its endpoints, total internal
  reflection, camera depth of field (all lens rays meet at the focus point),
  PNG decoding with `std.compress.zlib`, scene-file error reporting and the
  CLI parser.
- **Not implemented (yet).** There is no next-event estimation or MIS, so
  small lights such as the one in the Cornell box converge slowly. There is
  no texture mapping and no mesh import.

## License

MIT © 2026 Aryan Vohra
