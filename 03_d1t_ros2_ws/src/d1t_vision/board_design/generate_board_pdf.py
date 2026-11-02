#!/usr/bin/env python3
"""Generate an exact-scale PDF from the existing unmeasured VISION-01 board spec."""
import hashlib
import json
from pathlib import Path

import cv2
from reportlab.lib.colors import HexColor
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib.utils import ImageReader

ROOT = Path(__file__).resolve().parent
SPEC_PATH = ROOT / "board-design.json"
PNG_PATH = ROOT / "VISION-01-charuco-design-not-measured.png"
PDF_PATH = ROOT / "VISION-01-charuco-design-not-measured.pdf"
CHECK_PATH = ROOT / "GENERATION-CHECK.json"

spec = json.loads(SPEC_PATH.read_text(encoding="utf-8"))
expected = {
    "kind": "charuco", "dictionary": "DICT_4X4_50", "squares_x": 5, "squares_y": 5,
    "square_length_m": 0.03, "marker_length_m": 0.022, "print_scale": 1.0,
    "origin": "outer_top_left_x_right_y_down_z_cross", "legacy_pattern": True, "measured": False,
    "board_id": "DESIGN_EXAMPLE_NOT_MEASURED",
}
for key, value in expected.items():
    if spec.get(key) != value:
        raise ValueError(f"unexpected existing design value: {key}={spec.get(key)!r}")

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
args = (spec["squares_x"], spec["squares_y"], spec["square_length_m"], spec["marker_length_m"], dictionary)
board = aruco.CharucoBoard_create(*args) if hasattr(aruco, "CharucoBoard_create") else aruco.CharucoBoard(args[:2], *args[2:])
if hasattr(board, "setLegacyPattern"):
    board.setLegacyPattern(spec["legacy_pattern"])
elif not spec["legacy_pattern"]:
    # Match d1t_vision.board.opencv_board(): an older OpenCV API is only
    # permitted for the existing legacy=true pattern.
    raise RuntimeError("OpenCV lacks setLegacyPattern; non-legacy board would be ambiguous")

pixels_per_square = 300
image_px = spec["squares_x"] * pixels_per_square
image = board.generateImage((image_px, image_px)) if hasattr(board, "generateImage") else board.draw((image_px, image_px))
if image.shape != (image_px, image_px) or image.dtype != "uint8":
    raise RuntimeError(f"unexpected generated board image: {image.shape} {image.dtype}")
if not cv2.imwrite(str(PNG_PATH), image):
    raise RuntimeError("could not write board PNG")

page_w, page_h = A4
board_size_mm = spec["squares_x"] * spec["square_length_m"] * 1000 * spec["print_scale"]
board_size_pt = board_size_mm * mm
board_x = (page_w - board_size_pt) / 2
board_y = 42 * mm

canvas = Canvas(str(PDF_PATH), pagesize=A4, pageCompression=1)
canvas.setTitle("VISION-01 ChArUco design - NOT MEASURED")
canvas.setAuthor("d1t_vision existing board-design.example.json")
canvas.setFont("Helvetica-Bold", 15)
canvas.setFillColor(HexColor("#B00020"))
canvas.drawCentredString(page_w / 2, page_h - 18 * mm, "NOT FOR REAL CALIBRATION UNTIL MEASURED")
canvas.setFillColor(HexColor("#202020"))
canvas.setFont("Helvetica", 8.5)
canvas.drawCentredString(page_w / 2, page_h - 24 * mm, "VISION-01 existing design: ChArUco DICT_4X4_50 | 5 x 5 | 30 mm squares | 22 mm markers")
canvas.drawCentredString(page_w / 2, page_h - 29 * mm, "Print only at 100% / Actual size. Board outer edge below is exactly 150 x 150 mm in this PDF.")
canvas.drawImage(ImageReader(str(PNG_PATH)), board_x, board_y, width=board_size_pt, height=board_size_pt, mask="auto")
canvas.setStrokeColor(HexColor("#005A9C"))
canvas.setLineWidth(0.75)
canvas.rect(board_x, board_y, board_size_pt, board_size_pt, stroke=1, fill=0)
canvas.setFillColor(HexColor("#202020"))
canvas.setFont("Helvetica", 8)
canvas.drawCentredString(page_w / 2, board_y - 7 * mm, "Measure the black/white checkerboard outer edge, not this blue verification outline.")
canvas.drawCentredString(page_w / 2, board_y - 11 * mm, "Coordinate origin: printed-face outer top-left; +X right, +Y down, +Z points through the board rear.")
canvas.showPage()

canvas.setFont("Helvetica-Bold", 18)
canvas.setFillColor(HexColor("#B00020"))
canvas.drawString(20 * mm, page_h - 24 * mm, "PRINT CHECK - NOT MEASURED")
canvas.setFillColor(HexColor("#202020"))
canvas.setFont("Helvetica", 11)
lines = [
    "1. Print page 1 only on A4 using 100% / Actual size; disable fit-to-page and auto scaling.",
    "2. Measure the checkerboard outer edge: X = 150.0 mm and Y = 150.0 mm.",
    "3. Check multiple squares: 30.0 mm. Check markers: 22.0 mm. Record tool and readings.",
    "4. Reject and remake if X/Y scale differs, the board bends, an edge is clipped, or a marker is obscured.",
    "5. Do not set measured=true until physical dimensions, print scale, board ID, and",
    "   measurement record are verified.",
    "This PDF is an offline design artifact only; it is not a calibration result or a permission to capture samples.",
]
y = page_h - 39 * mm
for line in lines:
    canvas.drawString(23 * mm, y, line)
    y -= 11 * mm
canvas.setFont("Helvetica-Oblique", 9)
canvas.drawString(20 * mm, 20 * mm, "Specification source: d1t_vision/config/board-design.example.json; measured=false preserved.")
canvas.save()

def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

CHECK_PATH.write_text(json.dumps({
    "result": "OK", "source_spec": str(SPEC_PATH), "spec_sha256": sha256(SPEC_PATH),
    "dictionary": spec["dictionary"], "legacy_pattern": spec["legacy_pattern"], "measured": spec["measured"],
    "board_image": {"path": PNG_PATH.name, "sha256": sha256(PNG_PATH), "pixels": [image_px, image_px], "pixels_per_square": pixels_per_square},
    "pdf": {"path": PDF_PATH.name, "sha256": sha256(PDF_PATH), "page": "A4", "board_outer_size_mm": [board_size_mm, board_size_mm], "board_rect_pt": [board_x, board_y, board_size_pt, board_size_pt]},
}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
