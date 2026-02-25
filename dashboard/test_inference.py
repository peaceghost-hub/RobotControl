#!/usr/bin/env python3
"""Quick diagnostic: check what query() actually returns."""
import torch, time, inspect, sys
from transformers import AutoModelForCausalLM, PreTrainedModel
from PIL import Image

print("Loading model...", flush=True)
_orig = PreTrainedModel._adjust_tied_keys_with_tied_pointers
def _safe(self_model, *a, **kw):
    if not hasattr(self_model, 'all_tied_weights_keys'):
        self_model.all_tied_weights_keys = (
            self_model.get_expanded_tied_weights_keys(all_submodels=False)
            if hasattr(self_model, 'get_expanded_tied_weights_keys') else {}
        )
    return _orig(self_model, *a, **kw)
PreTrainedModel._adjust_tied_keys_with_tied_pointers = _safe
model = AutoModelForCausalLM.from_pretrained(
    'vikhyatk/moondream2', revision='2025-06-21',
    trust_remote_code=True, dtype=torch.float32, low_cpu_mem_usage=True,
)
PreTrainedModel._adjust_tied_keys_with_tied_pointers = _orig
model.eval()
print("Model loaded OK", flush=True)

# ── Inspect signatures ──────────────────────────────
print("\n=== METHOD SIGNATURES ===", flush=True)
for name in ('query', 'detect', 'caption', 'point'):
    prop = getattr(type(model), name, None)
    if isinstance(prop, property):
        obj = getattr(model, name)
        if callable(obj):
            sig = inspect.signature(obj)
            print(f"  {name}(): {sig}", flush=True)
            print(f"    params: {list(sig.parameters.keys())}", flush=True)
        else:
            print(f"  {name}: NOT callable — type={type(obj)}", flush=True)
    elif callable(getattr(model, name, None)):
        sig = inspect.signature(getattr(model, name))
        print(f"  {name}(): {sig}", flush=True)
    else:
        print(f"  {name}: MISSING", flush=True)

# ── Create a simple test image ──────────────────────
import random
img = Image.new('RGB', (320, 240), color=(100, 150, 200))

# ── Test 1: basic query, NO extras ─────────────────
print("\n=== TEST 1: query(img, prompt) — no extras ===", flush=True)
try:
    t0 = time.time()
    with torch.inference_mode():
        result = model.query(img, "What do you see?")
    elapsed = time.time() - t0
    print(f"  type: {type(result)}", flush=True)
    print(f"  repr: {repr(result)[:500]}", flush=True)
    print(f"  time: {elapsed:.1f}s", flush=True)
    if isinstance(result, dict):
        for k, v in result.items():
            print(f"  key '{k}': type={type(v).__name__}, repr={repr(v)[:200]}", flush=True)
except Exception as e:
    print(f"  ERROR: {type(e).__name__}: {e}", flush=True)

# ── Test 2: query with settings + reasoning=False ──
print("\n=== TEST 2: query(img, prompt, reasoning=False, settings={temp:0}) ===", flush=True)
try:
    t0 = time.time()
    with torch.inference_mode():
        result = model.query(
            img, "What do you see?",
            reasoning=False,
            settings={"temperature": 0, "max_tokens": 100},
        )
    elapsed = time.time() - t0
    print(f"  type: {type(result)}", flush=True)
    print(f"  repr: {repr(result)[:500]}", flush=True)
    print(f"  time: {elapsed:.1f}s", flush=True)
    if isinstance(result, dict):
        for k, v in result.items():
            print(f"  key '{k}': type={type(v).__name__}, repr={repr(v)[:200]}", flush=True)
except Exception as e:
    print(f"  ERROR: {type(e).__name__}: {e}", flush=True)

# ── Test 3: detect ──────────────────────────────────
print("\n=== TEST 3: detect(img, 'object') ===", flush=True)
try:
    t0 = time.time()
    with torch.inference_mode():
        result = model.detect(img, "object")
    elapsed = time.time() - t0
    print(f"  type: {type(result)}", flush=True)
    print(f"  repr: {repr(result)[:500]}", flush=True)
    print(f"  time: {elapsed:.1f}s", flush=True)
except Exception as e:
    print(f"  ERROR: {type(e).__name__}: {e}", flush=True)

print("\n=== ALL TESTS DONE ===", flush=True)
