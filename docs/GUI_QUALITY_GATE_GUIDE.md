# Quality Gate Tab - User Guide

## Overview

The **Quality Gate** tab provides an interactive interface for analyzing per-image reprojection errors and removing problematic images from your reconstruction.

## Features

### 1. Load & Analyze Reconstruction
- Browse to your reconstruction output directory (e.g., `runs/phase3_clean`)
- Click **Load & Analyze** to compute per-image errors
- Supports any reconstruction with `metadata.json` file

### 2. Interactive Error Visualization
- **Sortable table** showing all images with their error statistics:
  - **Mean Error**: Average reprojection error across all observations
  - **Max Error**: Worst single observation in that image
  - **Median Error**: Median reprojection error (robust to outliers)
- **Color coding**: Red = will be removed, Green = will be kept

### 3. Adjustable Filtering Threshold
- **Slider control**: Set removal threshold from 0% to 50%
- **Error criterion selector**:
  - **Mean**: Remove images with highest average error (recommended)
  - **Max**: Remove images with worst individual observations
  - **Median**: Remove based on median error (robust metric)
- **Live preview**: Statistics update as you move the slider

### 4. Statistics Display
Shows real-time impact of your threshold selection:
```
📊 19 images → Remove 2 (10%) = 17 remaining
📉 Mean error: 3.44px → 2.43px (+29.4% improvement)
🗑️ Worst: DSC_0284, DSC_0288
```

### 5. Apply Filter & Re-optimize
- Click **Apply Filter & Re-optimize** to:
  1. Remove selected images from reconstruction
  2. Clean up 3D points (remove orphaned observations)
  3. Re-run bundle adjustment on filtered structure
  4. Save results to `filtered/` subdirectory
- **Safety checks**:
  - Always keeps at least 50% of images
  - Ensures all 3D points have ≥2 remaining views
  - Confirms action with user before proceeding

### 6. Export Statistics
- Save per-image error table to CSV file
- Use for analysis, documentation, or reporting
- Format: `Image, Mean_Error_px, Max_Error_px, Median_Error_px`

## Workflow Example

### Step 1: Run Reconstruction
First, run Phase 3 reconstruction from the **Phase 3/4 Reconstruction** tab or command line:
```bash
python tools/phase3_clean_pipeline.py \
    --images "calib/test/DSC_*.TIF" \
    --calib calib/1_10/camera_intrinsics.json \
    --layout calib/fixtures/layout_4tags.json \
    --output runs/phase3_clean
```

### Step 2: Analyze in Quality Gate Tab
1. Switch to **Quality Gate** tab
2. Browse to `runs/phase3_clean`
3. Click **Load & Analyze**
4. Review the sorted error table

### Step 3: Set Threshold
1. Examine which images have high errors
2. Use the **slider** to set removal percentage
3. Watch the **statistics** update:
   - How many images will be removed?
   - What's the expected error improvement?
   - Which images will be affected?

### Step 4: Apply Filter
1. Click **Apply Filter & Re-optimize**
2. Confirm the action
3. Wait for re-optimization (shows progress dialog)
4. Results saved to `runs/phase3_clean/filtered/`

### Step 5: Validate Results
The filtered reconstruction automatically reloads in the table. Compare:
- **Before**: `runs/phase3_clean/metadata.json`
- **After**: `runs/phase3_clean/filtered/metadata.json`

## Best Practices

### Choosing the Right Threshold

**Conservative (5-10%)**:
- Removes only the worst outliers
- Minimal data loss
- Good for high-quality datasets
- **Use when**: Most images are already good quality

**Moderate (10-20%)**:
- Balanced approach
- Noticeable error improvement
- Acceptable data loss
- **Use when**: Mixed quality dataset

**Aggressive (20-30%)**:
- Maximum error reduction
- Significant data loss
- Risk of removing useful views
- **Use when**: Dataset has many poor-quality images

### Choosing the Right Criterion

**Mean Error (Recommended)**:
- Best overall metric
- Balances individual outliers with average quality
- Most consistent results
- **Use when**: General-purpose filtering

**Max Error**:
- Focuses on worst observations
- Good for finding images with detection failures
- Can be overly aggressive
- **Use when**: You see extreme outlier errors (>20px)

**Median Error**:
- Most robust to outliers
- Good for skewed error distributions
- Conservative filtering
- **Use when**: Dataset has occasional detection glitches

### Safety Limits

The tool enforces these constraints:
- **Minimum images**: Always keeps at least 5 images
- **Minimum retention**: Always keeps at least 50% of images
- **Point visibility**: Removes 3D points that drop below 2 views
- **Track connectivity**: Maintains reconstruction graph integrity

### Iterative Filtering

For very poor quality datasets:
1. Apply 10% filter
2. Analyze results
3. If errors still high, apply another 10% to filtered results
4. Repeat until QA thresholds met
5. **Warning**: Don't remove more than 30-40% total

## Output Files

After applying the filter, the `filtered/` directory contains:

**refpoints_L.json**:
- Filtered 3D reconstruction
- Camera poses (excluding removed images)
- 3D points with updated observations

**metadata.json**:
- Quality gate report (which images removed, why)
- Bundle adjustment statistics
- Camera and point data for GUI reload

## Troubleshooting

### "No metadata.json found"
**Solution**: Ensure you're loading a valid reconstruction output directory. The directory must contain `metadata.json` created by Phase 3 pipeline.

### "Filter application failed"
**Causes**:
- Too few images remaining (violates 50% safety limit)
- Too many 3D points lost (all points require ≥2 views)
- Bundle adjustment failed to converge

**Solution**: Try a lower threshold (remove fewer images).

### "Filtering had no effect"
**Causes**:
- Threshold too low (removes 0 images)
- All images have similar errors

**Solution**: Increase threshold or check if dataset quality is already uniform.

### GUI freezes during "Apply Filter"
**Expected behavior**: Bundle adjustment can take 10-60 seconds depending on reconstruction size. The GUI will show a message dialog during processing.

## Tips & Tricks

### Quick Assessment
1. Load reconstruction
2. Sort by mean error (already done automatically)
3. Look at top 10% - are they significantly worse?
4. If yes, remove them; if no, keep all

### Finding the Sweet Spot
1. Set slider to 20%
2. Slowly decrease until improvement plateaus
3. That's your optimal threshold

### Exporting for Reports
1. Set desired threshold
2. Click **Export Statistics**
3. Open CSV in Excel/Google Sheets
4. Create charts showing error distribution
5. Highlight removed images

### Batch Processing
For multiple reconstructions:
1. Load first reconstruction
2. Apply filter with chosen threshold
3. Note the improvement statistics
4. Load next reconstruction
5. Apply same threshold
6. Compare improvement across datasets

## Integration with Phase 3 Pipeline

The Quality Gate tab uses the same filtering logic as the automatic 10% removal in Phase 3 pipeline:
- **Automatic** (in pipeline): Always removes worst 10% by mean error
- **Manual** (in GUI): Choose threshold and criterion interactively

**Use GUI when**:
- You want to inspect errors before removal
- Dataset quality is unknown
- Need to tune threshold for specific project
- Want to document which images were removed

**Use automatic when**:
- Standard workflow with known data quality
- Batch processing many datasets
- 10% removal is sufficient
- No manual intervention needed

## Related Documentation

- [ROADMAP.md](../ROADMAP.md) - Quality gate filtering workflow
- [QUALITY_GATE_IMPLEMENTATION.md](QUALITY_GATE_IMPLEMENTATION.md) - Technical details
- [PROJECT_CONSTITUTION.md](../PROJECT_CONSTITUTION.md) - Coordinate system conventions

---

**Last Updated**: January 13, 2026  
**Version**: 1.0
