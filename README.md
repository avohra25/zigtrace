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
- [ ] BVH
- [ ] Camera, integrator, multithreaded renderer
- [ ] Scene files, CLI, PNG output

## License

MIT © 2026 Aryan Vohra
