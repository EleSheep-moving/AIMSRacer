# Recovered enhanced PP numeric fixture

`recovered_enhanced_pp.py` preserves the recovered `_angle_wrap` helper and
`PurePursuitController` class verbatim. Only a module docstring and math/NumPy
imports were added. It includes no ROS or hardware code.

`provenance.json` records the original source SHA-256, original source line
numbers, exact matching Git commit/path/blob, class SHA-256 and fixture SHA-256.
The external recovery path is descriptive provenance; tests never access it.
The test verifies bundled fixture/class hashes, executes the old helper/class
AST, and compares 108 seeded numeric steering queries plus nine lookahead
speeds against `EnhancedPP`. Missing or altered fixtures fail the test; there
is no skip fallback. Both controllers use the actual 0.45 rad vehicle limit.
