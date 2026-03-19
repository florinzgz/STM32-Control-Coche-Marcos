# KLEE Symbolic Execution — Not Executed

KLEE was not used in this analysis because:

1. CBMC already provides bounded symbolic execution with the same guarantees
   for the bounded depths used (unwind 10).
2. KLEE requires LLVM bitcode compilation which adds complexity without
   additional benefits for this codebase.
3. All CBMC-flagged paths were manually reviewed and confirmed as false
   positives (flash-mapped pointers, init-order assumptions).

If KLEE analysis is desired in the future:

```bash
# Install KLEE
sudo apt-get install -y klee
# Or build from source: https://klee.github.io/getting-started/

# Compile to LLVM bitcode
clang -std=c11 -DHOST_TEST -D_GNU_SOURCE \
    -Ianalysis_artifacts/stubs -ICore/Inc \
    -emit-llvm -c Core/Src/safety_system.c -o /tmp/safety.bc

# Run KLEE
klee --entry-point=Safety_SetState /tmp/safety.bc
```
