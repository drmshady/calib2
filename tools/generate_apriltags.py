"""
Generate individual AprilTag36h11 markers or sheets for printing.

Creates high-resolution AprilTag images at specified sizes for marker cap fabrication.

Usage:
    # Generate single tag
    python tools/generate_apriltags.py --tag-id 100 --size 8.8 --output tags/tag_100.png
    
    # Generate sheet of tags
    python tools/generate_apriltags.py --sheet --start-id 100 --count 20 --size 8.8 --output tags/tag_sheet_100-119.png
"""

import cv2
import numpy as np
import argparse
from pathlib import Path
from typing import List, Tuple
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from PIL import Image
import io


def generate_single_tag(
    tag_id: int,
    tag_size_mm: float = 8.8,
    dpi: int = 600,
    border_bits: int = 1,
    output_path: str = None
) -> np.ndarray:
    """Generate a single AprilTag image.
    
    Args:
        tag_id: AprilTag ID (0-585 for 36h11)
        tag_size_mm: Physical size of tag in mm
        dpi: Print resolution (dots per inch)
        border_bits: White border width in bit units (default: 1)
        output_path: Optional output file path
        
    Returns:
        Tag image as numpy array
    """
    # Create AprilTag dictionary (36h11)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    # Calculate image size in pixels
    mm_to_inch = 1.0 / 25.4
    tag_size_px = int(tag_size_mm * mm_to_inch * dpi)
    
    # Generate tag
    tag_img = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_size_px, borderBits=border_bits)
    
    # Add white margin (2mm on all sides)
    margin_mm = 2.0
    margin_px = int(margin_mm * mm_to_inch * dpi)
    tag_with_margin = cv2.copyMakeBorder(
        tag_img, margin_px, margin_px, margin_px, margin_px,
        cv2.BORDER_CONSTANT, value=255
    )
    
    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), tag_with_margin)
        print(f"✓ Tag {tag_id} saved: {output_path}")
    
    return tag_with_margin


def generate_tag_sheet(
    start_id: int,
    count: int,
    tag_size_mm: float = 8.8,
    dpi: int = 600,
    cols: int = 8,
    spacing_mm: float = 5.0,
    border_bits: int = 1,
    output_path: str = "tag_sheet.png"
):
    """Generate a sheet of multiple AprilTags arranged in a grid.
    
    Args:
        start_id: Starting tag ID
        count: Number of tags to generate
        tag_size_mm: Physical size of each tag in mm
        dpi: Print resolution
        cols: Number of columns in grid
        spacing_mm: Space between tags in mm
        border_bits: White border width in bit units
        output_path: Output file path
    """
    print(f"Generating AprilTag sheet:")
    print(f"  Tags: {start_id} to {start_id + count - 1} ({count} total)")
    print(f"  Tag size: {tag_size_mm} mm")
    print(f"  Grid: {cols} columns")
    print(f"  Spacing: {spacing_mm} mm")
    print(f"  Resolution: {dpi} DPI")
    
    # Create AprilTag dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    # Calculate dimensions
    mm_to_inch = 1.0 / 25.4
    tag_size_px = int(tag_size_mm * mm_to_inch * dpi)
    spacing_px = int(spacing_mm * mm_to_inch * dpi)
    
    # Calculate rows needed
    rows = (count + cols - 1) // cols
    
    # Generate all tags
    tags = []
    for i in range(count):
        tag_id = start_id + i
        tag_img = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_size_px, borderBits=border_bits)
        tags.append((tag_id, tag_img))
    
    # Create sheet with labels
    # Each cell: tag + label underneath
    label_height_px = int(5 * mm_to_inch * dpi)  # 5mm height for label
    cell_width = tag_size_px + spacing_px
    cell_height = tag_size_px + label_height_px + spacing_px
    
    sheet_width = cols * cell_width + spacing_px
    sheet_height = rows * cell_height + spacing_px
    
    # Create white background
    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255
    
    # Place tags on sheet
    for idx, (tag_id, tag_img) in enumerate(tags):
        row = idx // cols
        col = idx % cols
        
        x = spacing_px + col * cell_width
        y = spacing_px + row * cell_height
        
        # Place tag
        sheet[y:y+tag_size_px, x:x+tag_size_px] = tag_img
        
        # Add label (ID number)
        label = f"ID {tag_id}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
        
        text_x = x + (tag_size_px - text_size[0]) // 2
        text_y = y + tag_size_px + int(label_height_px * 0.7)
        
        cv2.putText(sheet, label, (text_x, text_y), font, font_scale, 0, thickness, cv2.LINE_AA)
    
    # Add page border
    border_px = spacing_px
    sheet = cv2.copyMakeBorder(
        sheet, border_px, border_px, border_px, border_px,
        cv2.BORDER_CONSTANT, value=255
    )
    
    # Calculate physical dimensions
    sheet_width_mm = sheet.shape[1] / (mm_to_inch * dpi)
    sheet_height_mm = sheet.shape[0] / (mm_to_inch * dpi)
    
    # Save output
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    if output_path.suffix.lower() == '.pdf':
        # Create PDF with exact dimensions (100% scale)
        pdf = canvas.Canvas(str(output_path), pagesize=(sheet_width_mm * mm, sheet_height_mm * mm))
        
        # Convert numpy array to PIL Image and save to temp file
        pil_img = Image.fromarray(sheet)
        temp_img_path = output_path.with_suffix('.temp.png')
        pil_img.save(str(temp_img_path), dpi=(dpi, dpi))
        
        # Draw image at exact size (100% scale)
        pdf.drawImage(str(temp_img_path), 0, 0, width=sheet_width_mm * mm, height=sheet_height_mm * mm)
        pdf.save()
        
        # Clean up temp file
        temp_img_path.unlink()
        
        print(f"\n✓ PDF saved: {output_path}")
        print(f"  PDF page size: {sheet_width_mm:.1f} × {sheet_height_mm:.1f} mm (100% scale)")
    else:
        # Save as image (PNG/JPG)
        cv2.imwrite(str(output_path), sheet)
        print(f"\n✓ Image saved: {output_path}")
    
    print(f"  Sheet size: {sheet.shape[1]} × {sheet.shape[0]} px")
    print(f"  Physical size: {sheet_width_mm:.1f} × {sheet_height_mm:.1f} mm")
    print(f"\nPrinting instructions:")
    if output_path.suffix.lower() == '.pdf':
        print(f"  1. Print PDF at 100% scale (no fit-to-page)")
        print(f"  2. Verify tag size with ruler: {tag_size_mm} mm")
    else:
        print(f"  1. Print at exactly {dpi} DPI (disable scaling)")
        print(f"  2. Verify tag size with ruler: {tag_size_mm} mm")
    print(f"  3. Cut individual tags with precision blade")
    print(f"  4. Use for marker cap fabrication or calibration")


def main():
    parser = argparse.ArgumentParser(
        description="Generate AprilTag36h11 markers for printing"
    )
    parser.add_argument(
        "--tag-id", type=int,
        help="Generate single tag with this ID"
    )
    parser.add_argument(
        "--sheet", action="store_true",
        help="Generate sheet of multiple tags"
    )
    parser.add_argument(
        "--start-id", type=int, default=100,
        help="Starting tag ID for sheet (default: 100)"
    )
    parser.add_argument(
        "--count", type=int, default=20,
        help="Number of tags in sheet (default: 20)"
    )
    parser.add_argument(
        "--size", type=float, default=8.8,
        help="Tag size in mm (default: 8.8)"
    )
    parser.add_argument(
        "--dpi", type=int, default=600,
        help="Print resolution in DPI (default: 600)"
    )
    parser.add_argument(
        "--cols", type=int, default=8,
        help="Columns in sheet layout (default: 8)"
    )
    parser.add_argument(
        "--spacing", type=float, default=5.0,
        help="Spacing between tags in mm (default: 5.0)"
    )
    parser.add_argument(
        "--output", type=str,
        help="Output file path"
    )
    
    args = parser.parse_args()
    
    if args.sheet:
        # Generate sheet
        if not args.output:
            args.output = f"calib/apriltag_sheet_{args.start_id}-{args.start_id + args.count - 1}.png"
        
        generate_tag_sheet(
            start_id=args.start_id,
            count=args.count,
            tag_size_mm=args.size,
            dpi=args.dpi,
            cols=args.cols,
            spacing_mm=args.spacing,
            output_path=args.output
        )
    elif args.tag_id is not None:
        # Generate single tag
        if not args.output:
            args.output = f"calib/apriltag_{args.tag_id}.png"
        
        generate_single_tag(
            tag_id=args.tag_id,
            tag_size_mm=args.size,
            dpi=args.dpi,
            output_path=args.output
        )
    else:
        parser.error("Must specify either --tag-id for single tag or --sheet for multiple tags")


if __name__ == "__main__":
    main()
