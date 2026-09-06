# Minimal C++ Operating-System Kernel

A small educational kernel project focused on low-level systems concepts: boot flow, interrupt dispatch, a simple fixed-block allocator and cooperative task scheduling.

This repository is intentionally scoped as a learning kernel rather than a production OS. The implementation avoids claiming features that are not present, and the documentation separates architectural goals from completed code.

Core learning objectives:

- Understand execution flow at the hardware/software boundary.
- Implement deterministic memory and scheduler primitives.
- Keep kernel code small enough to reason about and debug.
- Document invariants and failure modes explicitly.

The current source file demonstrates the allocator/scheduler core in freestanding-style C++. A complete bootloader, GDT/IDT setup and architecture-specific interrupt stubs would be the next integration steps for running it bare-metal under QEMU.
