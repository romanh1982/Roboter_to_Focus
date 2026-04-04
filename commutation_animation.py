"""
Faulhaber 1016E012SR Commutation System Animation
===================================================
Animated technical diagram of the 5-segment precious metal
commutation system with real dimensions.

Motor: Faulhaber 1016E012SR (10mm dia, 12V, precious metal commutation)
Commutator: 5 segments, ~3mm diameter
Brushes: 3-finger gold alloy, ~1.11mm physical length, ~0.45mm effective contact

Usage:
    manim -pql commutation_animation.py CommutationScene
    manim -pqh commutation_animation.py CommutationScene    (high quality)
    manim -pqh --format=gif commutation_animation.py CommutationScene  (GIF)
"""

from manim import *
import numpy as np

# ============================================================
# Motor parameters (all in mm, converted to Manim units)
# ============================================================
COMM_RADIUS = 1.5          # mm - commutator radius
COMM_DIAMETER = 3.0        # mm
NUM_SEGMENTS = 5
GAP_DEG = 6.5              # degrees per gap between segments
SEG_DEG = (360 - NUM_SEGMENTS * GAP_DEG) / NUM_SEGMENTS  # ~58.7°
BRUSH_PHYSICAL_LEN = 1.11  # mm
BRUSH_WIDTH = 0.11         # mm (axial)
BRUSH_EFFECTIVE_ARC = 17.2 # degrees (~0.45mm effective contact on r=1.5mm)
SCALE = 2.0                # scale factor: mm -> Manim units

# Colors
SEG_COLORS = ["#8FAFC4", "#A4C48F", "#C48FA4", "#8FC4B4", "#C4B48F"]
SEG_STROKE = "#555555"
BRUSH_COLOR = "#C8A832"
BRUSH_STROKE = "#806820"
CONTACT_GOOD = "#1D9E75"
CONTACT_WARN = "#EF9F27"
CONTACT_BAD = "#E24B4A"
CURRENT_COLOR = "#378ADD"
GAP_COLOR = "#E8E6DF"
DIM_COLOR = "#888888"


class CommutationScene(Scene):
    """Main scene: cross-section view with animated commutation."""

    def construct(self):
        # Title
        title = Text(
            "Faulhaber 1016E012SR — Commutation system",
            font_size=28, weight=BOLD
        ).to_edge(UP, buff=0.3)
        subtitle = Text(
            "5-segment precious metal commutator, ø3 mm",
            font_size=20, color=GRAY
        ).next_to(title, DOWN, buff=0.1)
        self.play(Write(title), FadeIn(subtitle), run_time=1.5)

        # Build commutator cross-section
        comm_group = self.build_commutator()
        comm_group.move_to(ORIGIN + DOWN * 0.3)

        # Dimension annotations
        dims = self.build_dimensions(comm_group)

        self.play(FadeIn(comm_group), run_time=1.0)
        self.play(FadeIn(dims), run_time=0.8)
        self.wait(1)

        # Build brushes (stationary)
        brush_a, brush_a_contact = self.build_brush(0, "A")
        brush_b, brush_b_contact = self.build_brush(180, "B")
        brushes = VGroup(brush_a, brush_b)
        contacts = VGroup(brush_a_contact, brush_b_contact)

        for b in [brush_a, brush_b]:
            b.move_to(comm_group.get_center(), aligned_edge=ORIGIN)
        for c in [brush_a_contact, brush_b_contact]:
            c.move_to(comm_group.get_center(), aligned_edge=ORIGIN)

        self.play(FadeIn(brushes), FadeIn(contacts), run_time=0.8)
        self.wait(0.5)

        # Status display
        status_a = Text("Brush A: —", font_size=18, color=WHITE)
        status_b = Text("Brush B: —", font_size=18, color=WHITE)
        status_q = Text("Contact: —", font_size=18, color=WHITE)
        status_group = VGroup(status_a, status_b, status_q).arrange(
            RIGHT, buff=1.0
        ).to_edge(DOWN, buff=0.4)
        self.play(FadeIn(status_group), run_time=0.5)

        # Animate rotation - 2 full revolutions
        total_angle = 720  # degrees
        duration = 12       # seconds
        steps = 360
        dt = duration / steps
        da = total_angle / steps

        for step in range(steps):
            angle = da  # incremental angle per step
            # Rotate commutator
            comm_group.rotate(
                -angle * DEGREES,
                about_point=comm_group.get_center()
            )
            # Update status text
            current_deg = step * da
            sa = self.get_brush_state(current_deg, 0)
            sb = self.get_brush_state(current_deg, 180)
            new_a = Text(f"Brush A: {sa['text']}", font_size=18, color=sa['color'])
            new_b = Text(f"Brush B: {sb['text']}", font_size=18, color=sb['color'])
            qual = self.get_quality(sa, sb)
            new_q = Text(f"Contact: {qual['text']}", font_size=18, color=qual['color'])

            new_group = VGroup(new_a, new_b, new_q).arrange(
                RIGHT, buff=1.0
            ).to_edge(DOWN, buff=0.4)

            status_group.become(new_group)
            self.wait(dt)

        self.wait(2)

    def build_commutator(self):
        """Build the 5-segment commutator cross-section."""
        R = COMM_RADIUS * SCALE
        group = VGroup()

        # Segments
        for i in range(NUM_SEGMENTS):
            start_angle = i * (SEG_DEG + GAP_DEG)
            end_angle = start_angle + SEG_DEG

            seg = AnnularSector(
                inner_radius=R * 0.65,
                outer_radius=R,
                angle=SEG_DEG * DEGREES,
                start_angle=start_angle * DEGREES,
                fill_color=SEG_COLORS[i],
                fill_opacity=0.7,
                stroke_color=SEG_STROKE,
                stroke_width=1.0,
            )
            # Segment label
            mid_angle = (start_angle + SEG_DEG / 2) * DEGREES
            label_r = R * 0.825
            lx = label_r * np.cos(mid_angle)
            ly = label_r * np.sin(mid_angle)
            label = Text(str(i + 1), font_size=16, color=WHITE).move_to([lx, ly, 0])

            group.add(seg, label)

        # Gaps (visual markers)
        for i in range(NUM_SEGMENTS):
            gap_start = i * (SEG_DEG + GAP_DEG) + SEG_DEG
            gap = AnnularSector(
                inner_radius=R * 0.65,
                outer_radius=R,
                angle=GAP_DEG * DEGREES,
                start_angle=gap_start * DEGREES,
                fill_color=GAP_COLOR,
                fill_opacity=0.4,
                stroke_width=0,
            )
            group.add(gap)

        # Inner shaft area
        shaft = Circle(
            radius=R * 0.6,
            fill_color="#F0EDE5",
            fill_opacity=0.3,
            stroke_color=GRAY,
            stroke_width=0.5,
        )
        shaft_dot = Dot(ORIGIN, radius=0.08, color=GRAY)
        group.add(shaft, shaft_dot)

        return group

    def build_brush(self, angle_deg, label_text):
        """Build a brush assembly at a fixed angular position."""
        R = COMM_RADIUS * SCALE
        angle_rad = angle_deg * DEGREES

        # Brush body (triangular taper shape)
        tip_r = R + 0.05
        mid_r = R + 0.4
        base_r = R + 0.7

        half_arc = 8 * DEGREES  # visual half-width of brush body

        p1 = np.array([tip_r * np.cos(angle_rad - half_arc),
                        tip_r * np.sin(angle_rad - half_arc), 0])
        p2 = np.array([tip_r * np.cos(angle_rad + half_arc),
                        tip_r * np.sin(angle_rad + half_arc), 0])
        p3 = np.array([mid_r * np.cos(angle_rad + half_arc * 0.4),
                        mid_r * np.sin(angle_rad + half_arc * 0.4), 0])
        p4 = np.array([base_r * np.cos(angle_rad),
                        base_r * np.sin(angle_rad), 0])
        p5 = np.array([mid_r * np.cos(angle_rad - half_arc * 0.4),
                        mid_r * np.sin(angle_rad - half_arc * 0.4), 0])

        brush = Polygon(
            p1, p2, p3, p4, p5,
            fill_color=BRUSH_COLOR,
            fill_opacity=0.85,
            stroke_color=BRUSH_STROKE,
            stroke_width=1.0,
        )

        # Label
        label_r = R + 0.95
        lx = label_r * np.cos(angle_rad)
        ly = label_r * np.sin(angle_rad)
        label = Text(
            f"Brush {label_text}", font_size=16, color=BRUSH_COLOR
        ).move_to([lx, ly, 0])

        brush_group = VGroup(brush, label)

        # Contact arc (green line showing effective contact)
        contact_half = BRUSH_EFFECTIVE_ARC / 2 * DEGREES
        contact_arc = Arc(
            radius=R + 0.02,
            start_angle=angle_rad - contact_half,
            angle=BRUSH_EFFECTIVE_ARC * DEGREES,
            stroke_color=CONTACT_GOOD,
            stroke_width=3.0,
        )

        return brush_group, contact_arc

    def build_dimensions(self, comm_ref):
        """Build dimension annotations around the commutator."""
        R = COMM_RADIUS * SCALE
        cx, cy = comm_ref.get_center()[:2]
        dims = VGroup()

        # Diameter dimension line (horizontal)
        left_pt = np.array([cx - R, cy - R - 0.5, 0])
        right_pt = np.array([cx + R, cy - R - 0.5, 0])
        dim_line = DoubleArrow(
            left_pt, right_pt,
            buff=0, stroke_width=1.5,
            tip_length=0.12, color=DIM_COLOR
        )
        dim_text = Text(
            "ø3.0 mm", font_size=14, color=DIM_COLOR
        ).next_to(dim_line, DOWN, buff=0.08)
        # Extension lines
        ext_l = DashedLine(
            np.array([cx - R, cy, 0]),
            np.array([cx - R, cy - R - 0.6, 0]),
            stroke_width=0.5, color=DIM_COLOR, dash_length=0.08
        )
        ext_r = DashedLine(
            np.array([cx + R, cy, 0]),
            np.array([cx + R, cy - R - 0.6, 0]),
            stroke_width=0.5, color=DIM_COLOR, dash_length=0.08
        )
        dims.add(dim_line, dim_text, ext_l, ext_r)

        # Segment arc annotation
        seg_arc_r = R + 1.3
        seg_start_angle = 0
        seg_end_angle = SEG_DEG * DEGREES
        arc_dim = Arc(
            radius=seg_arc_r,
            start_angle=seg_start_angle,
            angle=seg_end_angle,
            stroke_color=DIM_COLOR,
            stroke_width=1.0,
        ).move_arc_center_to(np.array([cx, cy, 0]))
        arc_label = Text(
            f"{SEG_DEG:.1f}°", font_size=13, color=DIM_COLOR
        )
        mid_a = seg_end_angle / 2
        arc_label.move_to(np.array([
            cx + (seg_arc_r + 0.3) * np.cos(mid_a),
            cy + (seg_arc_r + 0.3) * np.sin(mid_a), 0
        ]))
        dims.add(arc_dim, arc_label)

        # Gap annotation
        gap_start = SEG_DEG * DEGREES
        gap_arc = Arc(
            radius=seg_arc_r,
            start_angle=gap_start,
            angle=GAP_DEG * DEGREES,
            stroke_color="#E24B4A",
            stroke_width=1.5,
        ).move_arc_center_to(np.array([cx, cy, 0]))
        gap_label = Text(
            f"gap {GAP_DEG}°", font_size=12, color="#E24B4A"
        )
        gap_mid = gap_start + GAP_DEG * DEGREES / 2
        gap_label.move_to(np.array([
            cx + (seg_arc_r + 0.4) * np.cos(gap_mid),
            cy + (seg_arc_r + 0.4) * np.sin(gap_mid), 0
        ]))
        dims.add(gap_arc, gap_label)

        # Brush effective contact arc label
        brush_label = Text(
            f"Brush effective arc: ~{BRUSH_EFFECTIVE_ARC:.0f}° (~0.45 mm)",
            font_size=14, color=CONTACT_GOOD
        ).move_to(np.array([cx, cy + R + 1.5, 0]))
        dims.add(brush_label)

        # Sagitta note
        sag_note = Text(
            "Sagitta at brush ends: ~0.106 mm",
            font_size=13, color=DIM_COLOR
        ).next_to(brush_label, DOWN, buff=0.15)
        dims.add(sag_note)

        return dims

    def get_brush_state(self, rotation_deg, brush_offset_deg):
        """Determine which segment(s) a brush contacts at a given rotation."""
        # Brush is fixed; commutator rotates
        # Effective brush position on commutator = -rotation + offset
        pos = (-rotation_deg + brush_offset_deg) % 360
        half_brush = BRUSH_EFFECTIVE_ARC / 2

        for i in range(NUM_SEGMENTS):
            seg_start = i * (SEG_DEG + GAP_DEG)
            seg_end = seg_start + SEG_DEG
            seg_mid = seg_start + SEG_DEG / 2
            gap_center = seg_end + GAP_DEG / 2

            # Check if brush center is well within segment
            margin = half_brush + 2
            if self._angle_in_range(pos, seg_start + margin, seg_end - margin):
                return {
                    "text": f"Seg {i+1}",
                    "color": CONTACT_GOOD,
                    "quality": "good"
                }

            # Check if near gap
            gap_s = seg_end
            gap_e = gap_s + GAP_DEG
            if self._angle_in_range(pos, gap_s - half_brush, gap_e + half_brush):
                next_seg = (i + 1) % NUM_SEGMENTS + 1
                dist = min(
                    abs(self._angle_diff(pos, gap_s)),
                    abs(self._angle_diff(pos, gap_e))
                )
                if dist < 2:
                    return {
                        "text": f"Gap {i+1}-{next_seg}",
                        "color": CONTACT_BAD,
                        "quality": "marginal"
                    }
                return {
                    "text": f"Bridge {i+1}-{next_seg}",
                    "color": CONTACT_WARN,
                    "quality": "bridging"
                }

        return {"text": "Transitioning", "color": CONTACT_WARN, "quality": "bridging"}

    def get_quality(self, sa, sb):
        if sa["quality"] == "good" and sb["quality"] == "good":
            return {"text": "Good", "color": CONTACT_GOOD}
        elif sa["quality"] == "marginal" or sb["quality"] == "marginal":
            return {"text": "Marginal — on gap", "color": CONTACT_BAD}
        else:
            return {"text": "Transitioning", "color": CONTACT_WARN}

    @staticmethod
    def _angle_in_range(a, s, e):
        a = a % 360
        s = s % 360
        e = e % 360
        if s <= e:
            return s <= a <= e
        return a >= s or a <= e

    @staticmethod
    def _angle_diff(a, b):
        return ((a - b + 540) % 360) - 180


class UnrolledView(Scene):
    """Second scene: unrolled commutator with brush sweep animation."""

    def construct(self):
        title = Text(
            "Unrolled commutator — brush sweep",
            font_size=26, weight=BOLD
        ).to_edge(UP, buff=0.3)
        self.play(Write(title), run_time=1.0)

        # Unrolled commutator strip
        strip_width = 11.0
        strip_height = 1.2
        strip_x = -strip_width / 2
        strip_y = 0.5

        strip_bg = Rectangle(
            width=strip_width, height=strip_height,
            fill_color="#F0EDE5", fill_opacity=0.3,
            stroke_color=GRAY, stroke_width=0.5
        ).move_to([0, strip_y, 0])

        segments = VGroup()
        seg_w = strip_width * SEG_DEG / 360
        gap_w = strip_width * GAP_DEG / 360

        for i in range(NUM_SEGMENTS):
            x = strip_x + i * (seg_w + gap_w) + seg_w / 2
            seg = Rectangle(
                width=seg_w, height=strip_height - 0.1,
                fill_color=SEG_COLORS[i], fill_opacity=0.6,
                stroke_color=SEG_STROKE, stroke_width=0.5
            ).move_to([x, strip_y, 0])
            label = Text(str(i + 1), font_size=18).move_to([x, strip_y, 0])
            segments.add(seg, label)

        # Scale bar
        scale_bar = VGroup()
        for i in range(NUM_SEGMENTS + 1):
            x_pos = strip_x + i * (seg_w + gap_w)
            mm_val = i * (SEG_DEG + GAP_DEG) / 360 * (np.pi * COMM_DIAMETER)
            tick = Line(
                [x_pos, strip_y - strip_height / 2 - 0.1, 0],
                [x_pos, strip_y - strip_height / 2 - 0.25, 0],
                stroke_width=0.5, color=DIM_COLOR
            )
            val = Text(f"{mm_val:.1f}", font_size=11, color=DIM_COLOR).next_to(
                tick, DOWN, buff=0.05
            )
            scale_bar.add(tick, val)
        mm_label = Text("mm", font_size=12, color=DIM_COLOR).next_to(
            scale_bar, RIGHT, buff=0.2
        )
        scale_bar.add(mm_label)

        self.play(FadeIn(strip_bg), FadeIn(segments), FadeIn(scale_bar), run_time=1.0)

        # Brush overlay (moving)
        brush_w = strip_width * BRUSH_EFFECTIVE_ARC / 360
        brush_rect = Rectangle(
            width=brush_w, height=strip_height + 0.4,
            fill_color=BRUSH_COLOR, fill_opacity=0.3,
            stroke_color=BRUSH_STROKE, stroke_width=1.0
        )
        brush_label = Text("Brush A", font_size=14, color=BRUSH_COLOR)
        brush_marker = VGroup(brush_rect, brush_label).arrange(DOWN, buff=0.1)
        start_x = strip_x + brush_w / 2
        brush_marker.move_to([start_x, strip_y + 0.2, 0])

        # Contact quality indicator
        contact_line = Line(
            [start_x - brush_w / 2, strip_y + strip_height / 2 + 0.02, 0],
            [start_x + brush_w / 2, strip_y + strip_height / 2 + 0.02, 0],
            stroke_color=CONTACT_GOOD, stroke_width=4
        )

        # Status text
        status = Text(
            "State: On segment 1", font_size=20, color=CONTACT_GOOD
        ).to_edge(DOWN, buff=0.6)

        self.play(FadeIn(brush_marker), FadeIn(contact_line), FadeIn(status))
        self.wait(0.5)

        # Animate brush sweep across 2 full commutator lengths
        sweep_dist = strip_width * 2
        steps = 500
        step_dx = sweep_dist / steps
        dt = 16.0 / steps  # total animation time

        for step in range(steps):
            x = start_x + step * step_dx
            # Wrap around
            if x > strip_x + strip_width:
                x -= strip_width
            brush_marker.move_to([x, strip_y + 0.2, 0])
            contact_line.move_to([x, strip_y + strip_height / 2 + 0.02, 0])

            # Determine state
            rel_pos = (x - strip_x) / strip_width * 360
            state_info = self._get_state(rel_pos)
            new_status = Text(
                f"State: {state_info['text']}", font_size=20,
                color=state_info['color']
            ).to_edge(DOWN, buff=0.6)
            contact_line.set_color(state_info['color'])
            status.become(new_status)

            self.wait(dt)

        self.wait(2)

    def _get_state(self, pos_deg):
        pos_deg = pos_deg % 360
        for i in range(NUM_SEGMENTS):
            seg_start = i * (SEG_DEG + GAP_DEG)
            seg_end = seg_start + SEG_DEG
            gap_end = seg_end + GAP_DEG
            half = BRUSH_EFFECTIVE_ARC / 2

            if seg_start + half < pos_deg < seg_end - half:
                return {"text": f"On segment {i+1}", "color": CONTACT_GOOD}
            if seg_end - half <= pos_deg <= seg_end + half:
                ns = (i + 1) % NUM_SEGMENTS + 1
                return {"text": f"Bridging {i+1}→{ns}", "color": CONTACT_WARN}
            if seg_end + half < pos_deg < gap_end - half + BRUSH_EFFECTIVE_ARC:
                ns = (i + 1) % NUM_SEGMENTS + 1
                return {"text": f"On gap {i+1}-{ns}!", "color": CONTACT_BAD}
        return {"text": "Transitioning", "color": CONTACT_WARN}


class CurvatureDetail(Scene):
    """Third scene: close-up of flat brush on curved commutator surface."""

    def construct(self):
        title = Text(
            "Flat brush on curved commutator — sagitta analysis",
            font_size=24, weight=BOLD
        ).to_edge(UP, buff=0.3)
        self.play(Write(title), run_time=1.0)

        # Large scale cross-section
        S = 40  # pixels per mm (large scale for detail)
        R_px = COMM_RADIUS * S
        cx, cy = 0, -0.5

        # Commutator arc (top portion only)
        arc = Arc(
            radius=R_px / 40,  # convert back to Manim units
            start_angle=50 * DEGREES,
            angle=80 * DEGREES,
            stroke_color=GRAY,
            stroke_width=2.0,
        ).move_to([cx, cy, 0])

        # Actually let's use a simpler scaling
        R_m = 3.0  # Manim units for the radius (large for detail view)

        comm_arc = Arc(
            radius=R_m,
            start_angle=60 * DEGREES,
            angle=60 * DEGREES,
            stroke_color="#888888",
            stroke_width=3.0,
        ).move_to([cx, cy - R_m + 1.5, 0])

        # Flat brush line on top
        brush_len_m = BRUSH_PHYSICAL_LEN / COMM_RADIUS * R_m  # scaled
        # The brush sits as a chord on the circle
        # Chord half-length in Manim units
        half_chord = brush_len_m / 2
        # The brush tangent point is at the top of the arc
        tangent_y = cy - R_m + 1.5 + R_m  # top of circle

        brush_line = Line(
            [cx - half_chord, tangent_y, 0],
            [cx + half_chord, tangent_y, 0],
            stroke_color=BRUSH_COLOR,
            stroke_width=4.0,
        )
        brush_label = Text(
            f"Flat brush: {BRUSH_PHYSICAL_LEN} mm",
            font_size=16, color=BRUSH_COLOR
        ).next_to(brush_line, UP, buff=0.2)

        self.play(Create(comm_arc), run_time=0.8)
        self.play(Create(brush_line), FadeIn(brush_label), run_time=0.8)

        # Contact point
        contact_dot = Dot(
            [cx, tangent_y, 0], radius=0.08, color=CONTACT_GOOD
        )
        contact_label = Text(
            "Tangent contact", font_size=14, color=CONTACT_GOOD
        ).next_to(contact_dot, DOWN, buff=0.5)

        self.play(FadeIn(contact_dot), FadeIn(contact_label), run_time=0.5)

        # Sagitta arrows at brush ends
        # Calculate sagitta: h = R - sqrt(R² - (L/2)²)
        sagitta_mm = COMM_RADIUS - np.sqrt(
            COMM_RADIUS**2 - (BRUSH_PHYSICAL_LEN / 2)**2
        )
        sagitta_m = sagitta_mm / COMM_RADIUS * R_m  # scaled

        # Left end gap
        left_brush_y = tangent_y
        left_comm_y = tangent_y - sagitta_m
        gap_arrow_l = DoubleArrow(
            [cx - half_chord - 0.15, left_comm_y, 0],
            [cx - half_chord - 0.15, left_brush_y, 0],
            buff=0, stroke_width=1.5, tip_length=0.1,
            color="#E24B4A"
        )
        gap_label_l = Text(
            f"{sagitta_mm:.3f} mm", font_size=13, color="#E24B4A"
        ).next_to(gap_arrow_l, LEFT, buff=0.1)

        # Right end gap
        gap_arrow_r = DoubleArrow(
            [cx + half_chord + 0.15, left_comm_y, 0],
            [cx + half_chord + 0.15, left_brush_y, 0],
            buff=0, stroke_width=1.5, tip_length=0.1,
            color="#E24B4A"
        )
        gap_label_r = Text(
            f"{sagitta_mm:.3f} mm", font_size=13, color="#E24B4A"
        ).next_to(gap_arrow_r, RIGHT, buff=0.1)

        sag_title = Text(
            "Sagitta (gap at brush ends)", font_size=16, color="#E24B4A"
        ).next_to(VGroup(gap_label_l, gap_label_r), DOWN, buff=0.5)

        self.play(
            FadeIn(gap_arrow_l), FadeIn(gap_label_l),
            FadeIn(gap_arrow_r), FadeIn(gap_label_r),
            FadeIn(sag_title),
            run_time=1.0
        )

        # Effective contact arc highlight
        # The effective contact is the portion where the gap < some threshold
        # (determined by brush compliance)
        eff_half_angle = BRUSH_EFFECTIVE_ARC / 2 * DEGREES
        eff_arc = Arc(
            radius=R_m + 0.05,
            start_angle=90 * DEGREES - eff_half_angle,
            angle=BRUSH_EFFECTIVE_ARC * DEGREES,
            stroke_color=CONTACT_GOOD,
            stroke_width=5.0,
        ).move_to([cx, cy - R_m + 1.5, 0])

        eff_label = Text(
            f"Effective contact: ~{BRUSH_EFFECTIVE_ARC:.0f}° (~0.45 mm)",
            font_size=16, color=CONTACT_GOOD
        ).to_edge(DOWN, buff=0.4)

        self.play(Create(eff_arc), FadeIn(eff_label), run_time=1.0)

        # Radius dimension
        radius_line = DashedLine(
            [cx, tangent_y, 0],
            [cx, cy - R_m + 1.5, 0],
            stroke_width=1.0, color=DIM_COLOR, dash_length=0.1
        )
        r_label = Text(
            f"r = {COMM_RADIUS} mm", font_size=14, color=DIM_COLOR
        ).next_to(radius_line, RIGHT, buff=0.15)

        self.play(Create(radius_line), FadeIn(r_label), run_time=0.5)
        self.wait(3)


# To render all scenes sequentially, run:
#   manim -pql commutation_animation.py CommutationScene UnrolledView CurvatureDetail
#
# Or render individually:
#   manim -pql commutation_animation.py CommutationScene
#   manim -pql commutation_animation.py UnrolledView
#   manim -pql commutation_animation.py CurvatureDetail
#
# Quality flags:
#   -ql  = low quality (480p, fast render)
#   -qm  = medium quality (720p)
#   -qh  = high quality (1080p)
#   -qk  = 4K quality
#   -p   = preview (open after rendering)
#   --format=gif  = output as GIF instead of MP4
