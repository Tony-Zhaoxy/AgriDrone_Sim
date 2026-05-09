import os
import cv2
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.platypus import SimpleDocTemplate, Image, Paragraph, Spacer, PageBreak
from reportlab.lib.styles import getSampleStyleSheet

# -------- settings --------
OUTPUT_DIR = "aruco_output"
PDF_NAME = "aruco_markers_15cm_A4.pdf"

# OpenCV ArUco dictionary
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Marker IDs and labels
MARKERS = [
    (0, "INIT"),
    (1, "LEFT"),
    (2, "RIGHT"),
    (10, "LAND"),
]

# Real printed black-square size
MARKER_SIZE_CM = 15
IMAGE_SIZE_PX = 1200
# -------------------------

os.makedirs(OUTPUT_DIR, exist_ok=True)

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)

styles = getSampleStyleSheet()
pdf_path = os.path.join(OUTPUT_DIR, PDF_NAME)

doc = SimpleDocTemplate(pdf_path, pagesize=A4)
elements = []

for marker_id, label in MARKERS:
    # Generate marker image
    marker_img = aruco.generateImageMarker(dictionary, marker_id, IMAGE_SIZE_PX)

    img_path = os.path.join(OUTPUT_DIR, f"marker_{marker_id}_{label}.png")
    cv2.imwrite(img_path, marker_img)

    # Add one marker per page
    elements.append(Paragraph(f"Marker ID {marker_id} — {label}", styles["Title"]))
    elements.append(Spacer(1, 1 * cm))
    elements.append(
        Image(
            img_path,
            width=MARKER_SIZE_CM * cm,
            height=MARKER_SIZE_CM * cm,
        )
    )
    elements.append(Spacer(1, 0.5 * cm))
    elements.append(
        Paragraph(
            f"Print at 100% scale. Black square must measure {MARKER_SIZE_CM} cm × {MARKER_SIZE_CM} cm.",
            styles["Normal"],
        )
    )
    elements.append(PageBreak())

doc.build(elements)

print(f"Saved PDF: {pdf_path}")
print("Saved PNG markers in:", OUTPUT_DIR)
