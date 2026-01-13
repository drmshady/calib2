"""
Generate ChArUco calibration board image for printing.

Creates a high-resolution board image with AprilTag36h11 markers.

Usage:
    python tools/generate_charuco_board.py --output calib/charuco_board_9x6.pdf
    python tools/generate_charuco_board.py --output calib/charuco_board_9x6.png --dpi 600
"""

import cv2
import numpy as np
import argparse
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from PIL import Image


def generate_charuco_board(
    squares_x: int = 9,
    squares_y: int = 6,
    square_size_mm: float = 10.0,
    marker_size_mm: float = 7.0,
    dpi: int = 600,
    output_path: str = "charuco_board.png",
    dict_type: str = "DICT_APRILTAG_36h11"
):
    """Generate ChArUco board image for printing.
    
    Args:
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_size_mm: Size of each square in mm
        marker_size_mm: Size of AprilTag markers in mm
        dpi: Print resolution (dots per inch)
        output_path: Output file path (.png, .jpg, or .pdf)
        dict_type: Dictionary type (default: DICT_APRILTAG_36h11)
    """
    # Dictionary mapping
    dict_mapping = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }
    
    # Create dictionary
    dict_id = dict_mapping.get(dict_type, cv2.aruco.DICT_APRILTAG_36h11)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    
    # Create ChArUco board
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_size_mm,
        marker_size_mm,
        dictionary
    )
    
    # Calculate image size in pixels
    # Convert mm to inches, then to pixels
    mm_to_inch = 1.0 / 25.4
    board_width_mm = squares_x * square_size_mm
    board_height_mm = squares_y * square_size_mm
    
    width_px = int(board_width_mm * mm_to_inch * dpi)
    height_px = int(board_height_mm * mm_to_inch * dpi)
    
    print(f"Generating ChArUco board:")
    print(f"  Dictionary: {dict_type}")
    print(f"  Grid: {squares_x}×{squares_y} squares")
    print(f"  Square size: {square_size_mm} mm")
    print(f"  Marker size: {marker_size_mm} mm")
    print(f"  Board dimensions: {board_width_mm} × {board_height_mm} mm")
    print(f"  Image size: {width_px} × {height_px} px")
    print(f"  Resolution: {dpi} DPI")
    print(f"  Output: {output_path}")
    
    # Generate board image
    img = board.generateImage((width_px, height_px), marginSize=0, borderBits=1)
    
    # Add margin (5mm on all sides)
    margin_mm = 5.0
    margin_px = int(margin_mm * mm_to_inch * dpi)
    img_with_margin = cv2.copyMakeBorder(
        img, margin_px, margin_px, margin_px, margin_px,
        cv2.BORDER_CONSTANT, value=255
    )
    
    # Save image
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Calculate physical dimensions with margin
    img_width_mm = img_with_margin.shape[1] / (mm_to_inch * dpi)
    img_height_mm = img_with_margin.shape[0] / (mm_to_inch * dpi)
    
    if output_path.suffix.lower() == '.pdf':
        # Create PDF with exact dimensions (100% scale)
        pdf = canvas.Canvas(str(output_path), pagesize=(img_width_mm * mm, img_height_mm * mm))
        
        # Convert numpy array to PIL Image and save to temp file
        pil_img = Image.fromarray(img_with_margin)
        temp_img_path = output_path.with_suffix('.temp.png')
        pil_img.save(str(temp_img_path), dpi=(dpi, dpi))
        
        # Draw image at exact size (100% scale)
        pdf.drawImage(str(temp_img_path), 0, 0, width=img_width_mm * mm, height=img_height_mm * mm)
        pdf.save()
        
        # Clean up temp file
        temp_img_path.unlink()
        
        print(f"\n✓ PDF saved: {output_path}")
        print(f"  PDF page size: {img_width_mm:.1f} × {img_height_mm:.1f} mm (100% scale)")
        print(f"\nPrinting instructions:")
        print(f"  1. Print PDF at 100% scale (no fit-to-page)")
        print(f"  2. Verify dimensions with ruler: {board_width_mm} × {board_height_mm} mm")
        print(f"  3. Mount on rigid, flat surface (aluminum plate or acrylic)")
        print(f"  4. Ensure no warping (±0.1mm flatness)")
    else:
        # Save as PNG/JPG
        success = cv2.imwrite(str(output_path), img_with_margin)
        
        if success:
            print(f"\n✓ Board image saved to: {output_path}")
            print(f"\nPrinting instructions:")
            print(f"  1. Print at exactly {dpi} DPI (disable scaling)")
            print(f"  2. Verify dimensions with ruler: {board_width_mm} × {board_height_mm} mm")
            print(f"  3. Mount on rigid, flat surface (aluminum plate or acrylic)")
            print(f"  4. Ensure no warping (±0.1mm flatness)")
        else:
            print(f"\n✗ Failed to save image to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate ChArUco calibration board for printing"
    )
    parser.add_argument(
        "--squares-x", type=int, default=9,
        help="Number of squares in X direction (default: 9)"
    )
    parser.add_argument(
        "--squares-y", type=int, default=6,
        help="Number of squares in Y direction (default: 6)"
    )
    parser.add_argument(
        "--square-size", type=float, default=10.0,
        help="Size of each square in mm (default: 10.0)"
    )
    parser.add_argument(
        "--marker-size", type=float, default=7.0,
        help="Size of AprilTag markers in mm (default: 7.0)"
    )
    parser.add_argument(
        "--dpi", type=int, default=600,
        help="Print resolution in DPI (default: 600)"
    )
    parser.add_argument(
        "--dict", type=str, default="DICT_APRILTAG_36h11",
        help="ArUco dictionary type (default: DICT_APRILTAG_36h11)"
    )
    parser.add_argument(
        "--output", type=str, default="calib/charuco_board_9x6.png",
        help="Output file path (default: calib/charuco_board_9x6.png)"
    )
    
    args = parser.parse_args()
    
    generate_charuco_board(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_size_mm=args.square_size,
        marker_size_mm=args.marker_size,
        dpi=args.dpi,
        output_path=args.output,
        dict_type=args.dict
    )


if __name__ == "__main__":
    main()
