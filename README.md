# zigtrace

A physically based Monte Carlo path tracer in Zig (standard library only).

> Work in progress.

## Building

Requires Zig 0.14 (tested with 0.14.1).

```sh
zig build test
```

## Status

- [x] Vector math, rays, deterministic xoshiro256++ RNG with sampling helpers
- [x] HDR image buffer, tone mapping (none/Reinhard/ACES), gamma, PPM output
- [x] Spheres, planes, triangles, AABBs; Lambertian, metal, dielectric and emissive materials
- [x] Binned-SAH BVH, tested against brute-force intersection
- [x] Thin-lens camera, path tracing integrator, multithreaded tile renderer
- [ ] Scene files, CLI, PNG output

`zig build run -Doptimize=ReleaseFast` renders a built-in demo scene to `demo.ppm`.

## License

MIT © 2026 Aryan Vohra
