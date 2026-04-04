# Integration Guide: Commutator Animation Tab
## Adding the animated commutator view to kuka_movement_viewer.py

---

## Files

- `commutator_animation_tab.py` — the new tab module (place next to `kuka_movement_viewer.py`)
- `commutation_animation.py` — Manim script for generating standalone videos (optional, independent)

---

## 3 changes to `kuka_movement_viewer.py`

### Change 1: Add import (at the top, after the existing imports)

```python
from commutator_animation_tab import CommutatorAnimationTab
```

### Change 2: Add the tab (in `_build_ui()`, after the existing tab creation around line 398)

Find this block:
```python
        self.tab_motor= ttk.Frame(self.nb); self.nb.add(self.tab_motor,text="  Motor Angle Profiles  ")
        self.tab_table= ttk.Frame(self.nb); self.nb.add(self.tab_table,text="  Data Table  ")
```

Add after it:
```python
        self.tab_comm = ttk.Frame(self.nb)
        self.nb.add(self.tab_comm, text="  Commutator Animation  ")
        self.comm_anim = CommutatorAnimationTab(self.tab_comm, self)
```

### Change 3: Update data on recalculate (in `_recalculate()`, after `self._draw_motor()` around line 555)

Find this line:
```python
        self._draw_path(); self._draw_dist(); self._draw_mv()
        self._draw_lut(); self._draw_lens(); self._draw_motor()
        self._fill_table()
```

Add after `self._fill_table()`:
```python
        if hasattr(self, 'comm_anim'):
            self.comm_anim.update_data()
```

---

## What you get

A new tab called "Commutator Animation" with:

- **Animated commutator cross-section** showing the 5 segments rotating in sync with the motor angle profile
- **Two fixed brushes** (A at 0°, B at 180°) with color-coded contact arcs:
  - Green = on segment (good contact)
  - Amber = bridging gap (transitioning)
  - Red = on gap (marginal contact)
- **Playback controls**: Play/Pause, Reset, End, speed selection (1x, 5x, 10x, 50x)
- **Time slider**: drag to scrub through the profile manually
- **Motor selection**: switch between M1 (Zoom1/S8) and M2 (Zoom2/S23)
- **Real-time info panel** showing:
  - Current time, LUT index, focus distance
  - Motor angle, revolution count
  - Commutator sector and sub-sector fraction
  - Brush A/B states and contact quality
  - Angular change per cycle (highlighted red when sub-sector)
  - Angular speed (RPM) and linear carriage speed (mm/s)
  - Motor and commutator dimensional parameters

## Standalone testing

You can test the animation module without the full KUKA viewer:

```bash
python commutator_animation_tab.py
```

This runs with a synthetic oscillating distance profile so you can verify the animation works before integrating.

---

## How it connects to the Manim script

The Manim script (`commutation_animation.py`) is **completely independent** — it generates standalone MP4 videos for presentations or documentation. The Tkinter tab module (`commutator_animation_tab.py`) is the interactive version that integrates into your existing tool.

Both use the same motor constants and commutator geometry, so the visualization is consistent between the interactive GUI and the rendered videos.

| Feature | `commutator_animation_tab.py` | `commutation_animation.py` |
|---------|-------------------------------|----------------------------|
| Technology | Tkinter Canvas (real-time) | Manim (rendered video) |
| Interactivity | Full (play/pause/scrub/motor select) | None (fixed video) |
| Data source | Live from kuka_movement_viewer | Hardcoded constants |
| Output | In-app visualization | MP4 / GIF files |
| Use case | Engineering analysis | Documentation / presentations |
