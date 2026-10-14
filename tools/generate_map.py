"""Regenerate the committed map using the same geometry as the simulator."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from amr_edge_lab.core import warehouse

grid = warehouse()
pixels = bytes(0 if (x, y) in grid.occupied else 254 for y in reversed(range(grid.height)) for x in range(grid.width))
(ROOT / "config/warehouse.pgm").write_bytes(f"P5\n{grid.width} {grid.height}\n255\n".encode() + pixels)
