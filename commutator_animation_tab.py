"""
Commutator Animation Tab for kuka_movement_viewer.py
=====================================================
Adds a new "Commutator Animation" tab showing an animated cross-section
of the Faulhaber 1016E012SR 5-segment commutator, synchronized with
the motor movement profile computed from the KUKA robot program.

Integration:
  1. Place this file next to kuka_movement_viewer.py
  2. In kuka_movement_viewer.py, add:
       from commutator_animation_tab import CommutatorAnimationTab
  3. In App._build_ui(), after the existing tab creation, add:
       self.tab_comm = ttk.Frame(self.nb)
       self.nb.add(self.tab_comm, text="  Commutator Animation  ")
       self.comm_anim = CommutatorAnimationTab(self.tab_comm, self)
  4. In App._recalculate(), after self._draw_motor(), add:
       if hasattr(self, 'comm_anim'):
           self.comm_anim.update_data()

  Or run standalone for testing:
       python commutator_animation_tab.py
"""

import tkinter as tk
from tkinter import ttk
import math
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ---------------------------------------------------------------------------
#  Motor / commutator constants (matching kuka_movement_viewer.py)
# ---------------------------------------------------------------------------
COMMUTATOR_SEGMENTS = 5
SECTOR_ANGLE_DEG    = 360.0 / COMMUTATOR_SEGMENTS   # 72°
COMMUTATOR_RADIUS   = 1.5   # mm
GAP_DEG             = 6.5   # degrees per insulating gap
SEG_DEG             = (360 - COMMUTATOR_SEGMENTS * GAP_DEG) / COMMUTATOR_SEGMENTS  # ~58.7°
BRUSH_PHYSICAL_LEN  = 1.11  # mm
BRUSH_EFFECTIVE_ARC = 17.2  # degrees (curvature-corrected effective contact)
CYCLE_TIME_MS       = 10    # 100 Hz generator rate

# Brush angular positions (fixed on end cap, matching photo orientation)
# In the photos, the brushes are at roughly 135° and 315° (upper-left and lower-right)
BRUSH_A_ANGLE       = 135.0  # degrees
BRUSH_B_ANGLE       = 315.0  # degrees (= 135 + 180)

# Segment fill colors
SEG_COLORS = ["#8FAFC4", "#A4C48F", "#C48FA4", "#8FC4B4", "#C4B48F"]
SEG_COLORS_DARK = ["#607080", "#708060", "#806070", "#607870", "#787060"]
BRUSH_COLOR = "#C8A832"
BRUSH_OUTLINE = "#806820"
CONTACT_GOOD = "#1D9E75"
CONTACT_WARN = "#EF9F27"
CONTACT_BAD  = "#E24B4A"


class CommutatorAnimationTab:
    """Animated commutator cross-section tab for the KUKA movement viewer."""

    def __init__(self, parent_frame, app=None):
        """
        parent_frame: the ttk.Frame for this tab
        app: reference to the main App instance (for accessing self.motor, self.lut)
        """
        self.parent = parent_frame
        self.app = app
        self.motor_data = None     # dict with m1_deg, m2_deg arrays
        self.lut_data = None       # distance LUT array
        self.n_samples = 0
        self.current_index = 0
        self.playing = False
        self.play_speed = 1        # 1x real-time
        self.selected_motor = "M1"  # which motor to animate
        self._after_id = None

        self._build_ui()

    def _build_ui(self):
        # Top control bar
        ctrl = ttk.Frame(self.parent, padding=4)
        ctrl.pack(fill=tk.X)

        ttk.Label(ctrl, text="Motor:").pack(side=tk.LEFT, padx=(0, 4))
        self.motor_var = tk.StringVar(value="M1")
        ttk.Radiobutton(ctrl, text="M1 (Zoom1/S8)", variable=self.motor_var,
                         value="M1", command=self._on_motor_changed).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(ctrl, text="M2 (Zoom2/S23)", variable=self.motor_var,
                         value="M2", command=self._on_motor_changed).pack(side=tk.LEFT, padx=2)

        ttk.Separator(ctrl, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        self.play_btn = ttk.Button(ctrl, text="\u25B6 Play", command=self._toggle_play, width=8)
        self.play_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="\u23EE Reset", command=self._reset, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl, text="\u23ED End", command=self._go_end, width=8).pack(side=tk.LEFT, padx=2)

        ttk.Separator(ctrl, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(ctrl, text="Speed:").pack(side=tk.LEFT, padx=(0, 4))
        self.speed_var = tk.DoubleVar(value=1)
        self._fractional_accum = 0.0
        for spd, lbl in [(0.01, "0.01x"), (0.1, "0.1x"), (1, "1x"), (5, "5x"), (10, "10x"), (50, "50x")]:
            ttk.Radiobutton(ctrl, text=lbl, variable=self.speed_var,
                             value=spd).pack(side=tk.LEFT, padx=2)

        # Time slider
        slider_frame = ttk.Frame(self.parent, padding=(4, 0))
        slider_frame.pack(fill=tk.X)
        ttk.Label(slider_frame, text="Time:").pack(side=tk.LEFT)
        self.time_slider = ttk.Scale(slider_frame, from_=0, to=100,
                                      orient=tk.HORIZONTAL, command=self._on_slider)
        self.time_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.time_label = ttk.Label(slider_frame, text="0.000 s", width=12)
        self.time_label.pack(side=tk.LEFT)

        # Options row: plot checkboxes + background offset
        opts = ttk.Frame(self.parent, padding=(4, 2))
        opts.pack(fill=tk.X)

        self.show_focus_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text="Focus Distance", variable=self.show_focus_var,
                         command=self._toggle_plots).pack(side=tk.LEFT, padx=4)
        self.show_revs_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text="Motor Revolutions", variable=self.show_revs_var,
                         command=self._toggle_plots).pack(side=tk.LEFT, padx=4)

        ttk.Separator(opts, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(opts, text="BG offset X:").pack(side=tk.LEFT, padx=(0, 2))
        self.bg_offset_x_var = tk.StringVar(value="0")
        e_x = ttk.Entry(opts, textvariable=self.bg_offset_x_var, width=6)
        e_x.pack(side=tk.LEFT, padx=(0, 8))
        e_x.bind("<Return>", lambda _: self._draw_frame())

        ttk.Label(opts, text="Y:").pack(side=tk.LEFT, padx=(0, 2))
        self.bg_offset_y_var = tk.StringVar(value="0")
        e_y = ttk.Entry(opts, textvariable=self.bg_offset_y_var, width=6)
        e_y.pack(side=tk.LEFT)
        e_y.bind("<Return>", lambda _: self._draw_frame())

        ttk.Button(opts, text="Apply", command=self._draw_frame, width=6).pack(side=tk.LEFT, padx=4)

        # Main canvas area - split into commutator view and info panel
        main = ttk.Frame(self.parent)
        main.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Canvas for the commutator drawing
        self.canvas = tk.Canvas(main, bg="#F8F7F3", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right info panel
        info = ttk.Frame(main, width=280, padding=8)
        info.pack(side=tk.RIGHT, fill=tk.Y)
        info.pack_propagate(False)

        ttk.Label(info, text="Commutator state", font=("Arial", 12, "bold")).pack(anchor=tk.W)
        ttk.Separator(info).pack(fill=tk.X, pady=4)

        self.info_labels = {}
        for key, label in [
            ("time", "Time"),
            ("index", "LUT index"),
            ("distance", "Focus distance"),
            ("motor_angle", "Motor angle"),
            ("motor_rev", "Motor revolutions"),
            ("comm_sector", "Commutator sector"),
            ("sector_frac", "Sector fraction"),
            ("brush_a", "Brush A state"),
            ("brush_b", "Brush B state"),
            ("contact", "Contact quality"),
            ("delta_angle", "\u0394angle/cycle"),
            ("delta_sector", "\u0394sector/cycle"),
            ("angular_speed", "Angular speed"),
            ("linear_speed", "Linear speed"),
        ]:
            ttk.Label(info, text=label, foreground="gray", font=("Arial", 9)).pack(anchor=tk.W, pady=(4, 0))
            lbl = ttk.Label(info, text="—", font=("Arial", 10, "bold"))
            lbl.pack(anchor=tk.W)
            self.info_labels[key] = lbl

        # Dimension info at bottom of panel
        ttk.Separator(info).pack(fill=tk.X, pady=8)
        ttk.Label(info, text="Faulhaber 1016E012SR", font=("Arial", 9, "bold")).pack(anchor=tk.W)
        ttk.Label(info, text=f"Commutator: {COMMUTATOR_SEGMENTS} segments, \u00f8{COMMUTATOR_RADIUS*2:.0f} mm",
                  foreground="gray", font=("Arial", 8)).pack(anchor=tk.W)
        ttk.Label(info, text=f"Segment arc: {SEG_DEG:.1f}\u00b0, gap: {GAP_DEG}\u00b0",
                  foreground="gray", font=("Arial", 8)).pack(anchor=tk.W)
        ttk.Label(info, text=f"Brush effective arc: ~{BRUSH_EFFECTIVE_ARC:.0f}\u00b0 (~0.45 mm)",
                  foreground="gray", font=("Arial", 8)).pack(anchor=tk.W)
        ttk.Label(info, text=f"Sagitta: ~0.106 mm",
                  foreground="gray", font=("Arial", 8)).pack(anchor=tk.W)

        # Optional time-series plots (below main area)
        self.plot_frame = ttk.Frame(self.parent)
        # Not packed by default — shown when a checkbox is toggled
        self.fig_plots = Figure(figsize=(8, 1.8), dpi=90)
        self.fig_plots.subplots_adjust(left=0.06, right=0.98, bottom=0.22, top=0.88, wspace=0.3)
        self.plot_canvas_mpl = FigureCanvasTkAgg(self.fig_plots, master=self.plot_frame)
        self.plot_canvas_mpl.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._focus_vline = None
        self._revs_vline = None
        self._plots_visible = False

        # Bind canvas resize
        self.canvas.bind("<Configure>", self._on_canvas_resize)

    def update_data(self):
        """Called when the main app recalculates. Pull motor profile data."""
        if self.app and hasattr(self.app, 'motor') and self.app.motor:
            self.motor_data = self.app.motor
            self.lut_data = self.app.lut if hasattr(self.app, 'lut') else None
            self.n_samples = len(self.motor_data.get("m1_deg", []))
            self.time_slider.configure(to=max(1, self.n_samples - 1))
            self.current_index = 0
            self._draw_frame()
            if self._plots_visible:
                self._rebuild_plots()
        else:
            self.motor_data = None
            self.n_samples = 0

    def _on_motor_changed(self):
        self.selected_motor = self.motor_var.get()
        self._draw_frame()
        if self._plots_visible:
            self._rebuild_plots()

    def _on_slider(self, value):
        if self.n_samples == 0:
            return
        self.current_index = int(float(value))
        self._draw_frame()

    def _toggle_play(self):
        if self.playing:
            self.playing = False
            self.play_btn.configure(text="\u25B6 Play")
            if self._after_id:
                self.parent.after_cancel(self._after_id)
                self._after_id = None
        else:
            if self.n_samples == 0:
                return
            self.playing = True
            self.play_btn.configure(text="\u23F8 Pause")
            self._play_step()

    def _play_step(self):
        if not self.playing or self.n_samples == 0:
            return
        speed = self.speed_var.get()
        self._fractional_accum += speed
        steps = int(self._fractional_accum)
        self._fractional_accum -= steps
        if steps < 1:
            self._after_id = self.parent.after(CYCLE_TIME_MS, self._play_step)
            return
        self.current_index += steps
        if self.current_index >= self.n_samples:
            self.current_index = self.n_samples - 1
            self.playing = False
            self.play_btn.configure(text="\u25B6 Play")
            self._draw_frame()
            return
        self.time_slider.set(self.current_index)
        self._draw_frame()
        self._after_id = self.parent.after(CYCLE_TIME_MS, self._play_step)

    def _reset(self):
        self.current_index = 0
        self._fractional_accum = 0.0
        self.time_slider.set(0)
        self._draw_frame()

    def _go_end(self):
        if self.n_samples > 0:
            self.current_index = self.n_samples - 1
            self.time_slider.set(self.current_index)
            self._draw_frame()

    def _on_canvas_resize(self, event):
        self._draw_frame()

    def _get_motor_angle(self, index):
        """Get the current motor angle for the selected motor at given index."""
        if not self.motor_data or index >= self.n_samples:
            return 0.0
        key = "m1_deg" if self.selected_motor == "M1" else "m2_deg"
        return self.motor_data[key][index]

    def _get_brush_state(self, commutator_angle_offset, brush_fixed_angle):
        """Determine brush contact state.
        commutator_angle_offset: how much the commutator has rotated (degrees)
        brush_fixed_angle: the fixed angular position of the brush (0° or 180°)
        """
        # Relative position of brush on commutator
        rel_pos = (brush_fixed_angle - commutator_angle_offset) % 360
        half_brush = BRUSH_EFFECTIVE_ARC / 2

        for i in range(COMMUTATOR_SEGMENTS):
            seg_start = i * (SEG_DEG + GAP_DEG)
            seg_end = seg_start + SEG_DEG
            gap_end = seg_end + GAP_DEG

            # Well within segment
            if self._angle_in_range(rel_pos, seg_start + half_brush + 2, seg_end - half_brush - 2):
                return {"state": f"Seg {i+1}", "color": CONTACT_GOOD, "quality": "good", "seg": i}

            # Near or on gap
            if self._angle_in_range(rel_pos, seg_end - half_brush - 2, gap_end + half_brush + 2):
                next_seg = (i + 1) % COMMUTATOR_SEGMENTS
                gap_center = seg_end + GAP_DEG / 2
                dist = abs(self._angle_diff(rel_pos, gap_center))
                if dist < 2:
                    return {"state": f"Gap {i+1}-{next_seg+1}", "color": CONTACT_BAD,
                            "quality": "marginal", "seg": i}
                return {"state": f"Bridge {i+1}-{next_seg+1}", "color": CONTACT_WARN,
                        "quality": "bridging", "seg": i}

        return {"state": "Transition", "color": CONTACT_WARN, "quality": "bridging", "seg": 0}

    @staticmethod
    def _angle_in_range(a, s, e):
        a = a % 360; s = s % 360; e = e % 360
        if s <= e:
            return s <= a <= e
        return a >= s or a <= e

    @staticmethod
    def _angle_diff(a, b):
        return ((a - b + 540) % 360) - 180

    def _draw_frame(self):
        """Draw the actual motor end-cap photo as background, with only the
        commutator segments drawn as a rotating overlay in the center.
        Everything else (housing, plastic cap, brush wires, terminals) is
        part of the photo and stays static.
        """
        c = self.canvas
        c.delete("all")

        w = c.winfo_width()
        h = c.winfo_height()
        if w < 100 or h < 100:
            return

        # ── Display the background photo ──
        if not hasattr(self, '_bg_photo_cache') or self._bg_cache_size != (w, h):
            self._prepare_bg_photo(w, h)

        # ── Background offset from user entry ──
        try:
            bg_ox = float(self.bg_offset_x_var.get())
        except (ValueError, tk.TclError):
            bg_ox = 0.0
        try:
            bg_oy = float(self.bg_offset_y_var.get())
        except (ValueError, tk.TclError):
            bg_oy = 0.0

        if self._bg_photo_tk:
            c.create_image(w // 2 + bg_ox, h // 2 + bg_oy,
                           image=self._bg_photo_tk, anchor=tk.CENTER)

        # ── Motor angle ──
        if self.motor_data and self.current_index < self.n_samples:
            motor_angle_deg = self._get_motor_angle(self.current_index)
            comm_rotation = motor_angle_deg % 360
        else:
            comm_rotation = 0
            motor_angle_deg = 0

        # ── Commutator position calibrated to the photo ──
        # In the original 988x950 photo the commutator center is at ~(478,478)
        # and the segment ring has outer radius ~87 px, inner ~48 px.
        photo_w, photo_h = 988, 950
        img_scale = min(w / photo_w, h / photo_h)
        img_disp_w = photo_w * img_scale
        img_disp_h = photo_h * img_scale
        off_x = (w - img_disp_w) / 2 + bg_ox
        off_y = (h - img_disp_h) / 2 + bg_oy

        cx = off_x + 478 * img_scale
        cy = off_y + 478 * img_scale
        R_comm = 87 * img_scale
        R_comm_inner = 48 * img_scale
        R_shaft = 10 * img_scale

        steps = 30

        # ── Cover the static commutator in the photo with a dark disc ──
        c.create_oval(cx - R_comm - 2, cy - R_comm - 2,
                      cx + R_comm + 2, cy + R_comm + 2,
                      fill="#2E2E2C", outline="", width=0)

        # ── Draw rotating commutator segments ──
        for i in range(COMMUTATOR_SEGMENTS):
            seg_start = i * (SEG_DEG + GAP_DEG) + comm_rotation
            seg_end = seg_start + SEG_DEG
            pts = []
            for s in range(steps + 1):
                a = math.radians(seg_start + (seg_end - seg_start) * s / steps)
                pts.extend([cx + R_comm * math.cos(a), cy - R_comm * math.sin(a)])
            for s in range(steps, -1, -1):
                a = math.radians(seg_start + (seg_end - seg_start) * s / steps)
                pts.extend([cx + R_comm_inner * math.cos(a),
                           cy - R_comm_inner * math.sin(a)])
            c.create_polygon(pts, fill=SEG_COLORS[i], outline="#555555", width=1)

            mid_a = math.radians(seg_start + SEG_DEG / 2)
            lr = (R_comm + R_comm_inner) / 2
            c.create_text(cx + lr * math.cos(mid_a), cy - lr * math.sin(mid_a),
                         text=str(i + 1), font=("Arial", 9, "bold"), fill="#222")

            gs, ge = seg_end, seg_end + GAP_DEG
            gp = []
            for s in range(steps + 1):
                a = math.radians(gs + (ge - gs) * s / steps)
                gp.extend([cx + R_comm * math.cos(a), cy - R_comm * math.sin(a)])
            for s in range(steps, -1, -1):
                a = math.radians(gs + (ge - gs) * s / steps)
                gp.extend([cx + R_comm_inner * math.cos(a),
                           cy - R_comm_inner * math.sin(a)])
            c.create_polygon(gp, fill="#2A2A28", outline="", width=0)

        # Shaft bore
        c.create_oval(cx - R_comm_inner + 1, cy - R_comm_inner + 1,
                      cx + R_comm_inner - 1, cy + R_comm_inner - 1,
                      fill="#404040", outline="#333", width=0.5)
        c.create_oval(cx - R_shaft, cy - R_shaft,
                      cx + R_shaft, cy + R_shaft,
                      fill="#888", outline="#666", width=1)

        # ── Brushes (5x longer wedge representation) ──
        brush_radial = 30 * img_scale   # 5× the original ~6 px extent
        brush_r_inner = R_comm + 1
        brush_r_outer = R_comm + 1 + brush_radial
        n_arc = 20

        for brush_angle, label, lx_sign, ly_sign in [
            (BRUSH_A_ANGLE, "A", -1, -1),
            (BRUSH_B_ANGLE, "B", 1, 1),
        ]:
            state = self._get_brush_state(comm_rotation, brush_angle)
            contact_half = BRUSH_EFFECTIVE_ARC / 2

            # Brush body – arc-shaped wedge polygon
            pts = []
            for s in range(n_arc + 1):
                a = math.radians(brush_angle - contact_half +
                                 BRUSH_EFFECTIVE_ARC * s / n_arc)
                pts.extend([cx + brush_r_outer * math.cos(a),
                            cy - brush_r_outer * math.sin(a)])
            for s in range(n_arc, -1, -1):
                a = math.radians(brush_angle - contact_half +
                                 BRUSH_EFFECTIVE_ARC * s / n_arc)
                pts.extend([cx + brush_r_inner * math.cos(a),
                            cy - brush_r_inner * math.sin(a)])
            c.create_polygon(pts, fill=BRUSH_COLOR, outline=BRUSH_OUTLINE, width=1)

            # Contact-state indicator stripe on the inner face
            arc_pts = []
            for s in range(n_arc + 1):
                a = math.radians(brush_angle - contact_half +
                                 BRUSH_EFFECTIVE_ARC * s / n_arc)
                arc_pts.extend([cx + (brush_r_inner + 2) * math.cos(a),
                                cy - (brush_r_inner + 2) * math.sin(a)])
            if len(arc_pts) >= 4:
                c.create_line(arc_pts, fill=state["color"], width=3,
                              smooth=True, capstyle=tk.ROUND)

            # Label
            ba_rad = math.radians(brush_angle)
            lx = cx + (brush_r_outer + 14) * math.cos(ba_rad) + lx_sign * 28
            ly = cy - (brush_r_outer + 14) * math.sin(ba_rad) + ly_sign * 8
            c.create_text(lx, ly, text=f"Brush {label}: {state['state']}",
                          font=("Arial", 9, "bold"), fill=state["color"])

        # ── Angle readout (top bar) ──
        c.create_rectangle(0, 0, w, 20, fill="white", outline="")
        c.create_text(w // 2, 10,
                     text=f"Motor: {motor_angle_deg:.1f}\u00b0  |  "
                          f"Commutator: {comm_rotation:.1f}\u00b0",
                     font=("Arial", 9), fill="#333333")

        # Update info panel
        self._update_info_panel(comm_rotation, motor_angle_deg)

        # Update time-series plot markers
        if self._plots_visible:
            self._update_plot_markers()

    def _prepare_bg_photo(self, canvas_w, canvas_h):
        """Load and scale the end-cap photo to fit the canvas."""
        self._bg_cache_size = (canvas_w, canvas_h)
        self._bg_photo_tk = None
        try:
            from PIL import Image, ImageTk
            img_path = self._find_bg_image()
            if not img_path:
                return
            img = Image.open(img_path)
            img_w, img_h = img.size
            scale = min(canvas_w / img_w, canvas_h / img_h)
            new_w = int(img_w * scale)
            new_h = int(img_h * scale)
            img = img.resize((new_w, new_h), Image.LANCZOS)
            self._bg_photo_tk = ImageTk.PhotoImage(img)
        except Exception:
            self._bg_photo_tk = None

    def _find_bg_image(self):
        """Find the end-cap photo file next to the script."""
        import os
        candidates = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "endcap_photo.png"),
            os.path.join(os.getcwd(), "endcap_photo.png"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "endcap_photo.jpg"),
            os.path.join(os.getcwd(), "endcap_photo.jpg"),
        ]
        for p in candidates:
            if os.path.isfile(p):
                return p
        return None

    def _update_info_panel(self, comm_rotation, motor_angle_deg):
        """Update the right-side information panel."""
        idx = self.current_index
        dt = CYCLE_TIME_MS / 1000.0

        self.info_labels["time"].configure(text=f"{idx * dt:.3f} s")
        self.info_labels["index"].configure(text=f"{idx} / {self.n_samples}")
        self.time_label.configure(text=f"{idx * dt:.3f} s")

        if self.lut_data and idx < len(self.lut_data):
            self.info_labels["distance"].configure(text=f"{self.lut_data[idx]} mm")
        else:
            self.info_labels["distance"].configure(text="—")

        self.info_labels["motor_angle"].configure(text=f"{motor_angle_deg:.2f}\u00b0")
        self.info_labels["motor_rev"].configure(text=f"{motor_angle_deg / 360:.3f} rev")

        sector = motor_angle_deg / SECTOR_ANGLE_DEG
        sector_int = int(sector) % COMMUTATOR_SEGMENTS
        sector_frac = (sector % 1.0)
        self.info_labels["comm_sector"].configure(text=f"{sector:.3f} ({sector_int + 1} of {COMMUTATOR_SEGMENTS})")
        self.info_labels["sector_frac"].configure(text=f"{sector_frac:.4f} ({sector_frac * SECTOR_ANGLE_DEG:.2f}\u00b0 into sector)")

        # Brush states
        state_a = self._get_brush_state(comm_rotation, 0)
        state_b = self._get_brush_state(comm_rotation, 180)
        self.info_labels["brush_a"].configure(text=state_a["state"], foreground=state_a["color"])
        self.info_labels["brush_b"].configure(text=state_b["state"], foreground=state_b["color"])

        # Contact quality
        if state_a["quality"] == "good" and state_b["quality"] == "good":
            self.info_labels["contact"].configure(text="Good", foreground=CONTACT_GOOD)
        elif state_a["quality"] == "marginal" or state_b["quality"] == "marginal":
            self.info_labels["contact"].configure(text="Marginal (on gap)", foreground=CONTACT_BAD)
        else:
            self.info_labels["contact"].configure(text="Bridging", foreground=CONTACT_WARN)

        # Delta angle per cycle
        if self.motor_data and idx > 0 and idx < self.n_samples:
            key = "m1_deg" if self.selected_motor == "M1" else "m2_deg"
            delta = self.motor_data[key][idx] - self.motor_data[key][idx - 1]
            delta_sector = delta / SECTOR_ANGLE_DEG
            self.info_labels["delta_angle"].configure(text=f"{delta:.2f}\u00b0/cycle")
            self.info_labels["delta_sector"].configure(text=f"{delta_sector:.4f} sectors/cycle")

            # Angular and linear speed
            omega_deg_s = delta / dt  # deg/s
            omega_rpm = omega_deg_s / 360 * 60
            linear_speed = abs(delta / 360 / (161.0 / (math.pi * 9.0))) / dt  # mm/s
            self.info_labels["angular_speed"].configure(text=f"{omega_rpm:.0f} RPM ({omega_deg_s:.0f}\u00b0/s)")
            self.info_labels["linear_speed"].configure(text=f"{linear_speed:.3f} mm/s")

            # Color the delta by sub-sector status
            if abs(delta) < SECTOR_ANGLE_DEG and abs(delta) > 0.01:
                self.info_labels["delta_angle"].configure(foreground=CONTACT_BAD)
                self.info_labels["delta_sector"].configure(foreground=CONTACT_BAD)
            else:
                self.info_labels["delta_angle"].configure(foreground="black")
                self.info_labels["delta_sector"].configure(foreground="black")
        else:
            self.info_labels["delta_angle"].configure(text="—")
            self.info_labels["delta_sector"].configure(text="—")
            self.info_labels["angular_speed"].configure(text="—")
            self.info_labels["linear_speed"].configure(text="—")

    # ------------------------------------------------------------------
    #  Optional time-series plots (focus distance / motor revolutions)
    # ------------------------------------------------------------------

    def _toggle_plots(self):
        """Show or hide the plot frame based on checkbox state."""
        show_any = self.show_focus_var.get() or self.show_revs_var.get()
        if show_any:
            self.plot_frame.pack(fill=tk.BOTH, padx=4, pady=(0, 4))
            self._plots_visible = True
            self._rebuild_plots()
        else:
            self.plot_frame.pack_forget()
            self._plots_visible = False

    def _rebuild_plots(self):
        """Redraw the full time-series curves (called when toggling or new data)."""
        self.fig_plots.clear()
        self._focus_vline = None
        self._revs_vline = None

        show_focus = self.show_focus_var.get()
        show_revs = self.show_revs_var.get()
        n_plots = int(show_focus) + int(show_revs)
        if n_plots == 0 or self.n_samples == 0:
            self.plot_canvas_mpl.draw_idle()
            return

        dt = CYCLE_TIME_MS / 1000.0
        t = np.arange(self.n_samples) * dt

        plot_idx = 1
        if show_focus:
            ax = self.fig_plots.add_subplot(1, n_plots, plot_idx)
            if self.lut_data and len(self.lut_data) >= self.n_samples:
                ax.plot(t, self.lut_data[:self.n_samples], color="#2171B5", linewidth=0.8)
            ax.set_xlabel("Time (s)", fontsize=8)
            ax.set_ylabel("Focus dist (mm)", fontsize=8)
            ax.set_title("Focus Distance", fontsize=9)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)
            self._focus_vline = ax.axvline(0, color="red", linewidth=1, linestyle="--")
            plot_idx += 1

        if show_revs:
            ax = self.fig_plots.add_subplot(1, n_plots, plot_idx)
            key = "m1_deg" if self.selected_motor == "M1" else "m2_deg"
            if self.motor_data and key in self.motor_data:
                revs = np.array(self.motor_data[key][:self.n_samples]) / 360.0
                ax.plot(t, revs, color="#D6604D", linewidth=0.8)
            ax.set_xlabel("Time (s)", fontsize=8)
            ax.set_ylabel("Revolutions", fontsize=8)
            ax.set_title(f"Motor Revolutions ({self.selected_motor})", fontsize=9)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)
            self._revs_vline = ax.axvline(0, color="red", linewidth=1, linestyle="--")

        self.fig_plots.subplots_adjust(left=0.06, right=0.98, bottom=0.22, top=0.88, wspace=0.3)
        self.plot_canvas_mpl.draw()

    def _update_plot_markers(self):
        """Move the vertical time-marker lines to the current index."""
        dt = CYCLE_TIME_MS / 1000.0
        t_now = self.current_index * dt
        changed = False
        if self._focus_vline:
            self._focus_vline.set_xdata([t_now, t_now])
            changed = True
        if self._revs_vline:
            self._revs_vline.set_xdata([t_now, t_now])
            changed = True
        if changed:
            self.plot_canvas_mpl.draw_idle()


# ---------------------------------------------------------------------------
#  Standalone test mode
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Commutator Animation — Standalone Test")
    root.geometry("1100x700")

    frame = ttk.Frame(root)
    frame.pack(fill=tk.BOTH, expand=True)

    anim = CommutatorAnimationTab(frame)

    # Generate a synthetic motor profile for testing
    n = 5000  # 50 seconds at 100 Hz
    t = np.linspace(0, 50, n)
    # Simulate a focus distance profile: oscillating between 2000-4000 mm
    dist = 3000 + 1000 * np.sin(2 * np.pi * 0.1 * t) + 500 * np.sin(2 * np.pi * 0.3 * t)

    # Convert to motor angles (simplified)
    MOTOR_REV_PER_MM = 161.0 / (math.pi * 9.0)
    INCR_PER_MM = 1.0 / 0.0005  # increments per mm

    # Fake zoom table: linear mapping for demo
    lens_mm = (dist - 1500) / 23500 * 5.0  # 0-5 mm travel
    motor_deg = lens_mm * MOTOR_REV_PER_MM * 360.0

    anim.motor_data = {
        "m1_deg": motor_deg.tolist(),
        "m2_deg": (motor_deg * 0.7).tolist(),  # M2 has different curve
        "m1_incr": (lens_mm * INCR_PER_MM).tolist(),
        "m2_incr": (lens_mm * 0.7 * INCR_PER_MM).tolist(),
        "m1_mm": lens_mm.tolist(),
        "m2_mm": (lens_mm * 0.7).tolist(),
        "m1_sectors": (motor_deg / 72.0).tolist(),
        "m2_sectors": (motor_deg * 0.7 / 72.0).tolist(),
    }
    anim.lut_data = [int(d) for d in dist]
    anim.n_samples = n
    anim.time_slider.configure(to=n - 1)
    anim._draw_frame()

    root.mainloop()
