# Phase 3 Unknown Layout Pipeline Test
# Run from D:\calib2 directory

Write-Host "Phase 3: Unknown Layout Reconstruction" -ForegroundColor Cyan
Write-Host "Dataset: calib/test2 (58 images)" -ForegroundColor Cyan
Write-Host ""

python tools\phase3_unknown_layout_pipeline.py `
    --images "calib\test2\DSC_*.TIF" `
    --calib "calib\test2\camera_intrinsics.json" `
    --output "runs\test2_unknown_layout" `
    --tag-size 8.8 `
    --verbose

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "✅ Pipeline SUCCESS" -ForegroundColor Green
    Write-Host ""
    Write-Host "Results in: runs\test2_unknown_layout\" -ForegroundColor Yellow
    Write-Host "  - refpoints_L.json (3D points in L-frame, mm)" -ForegroundColor Yellow
    Write-Host "  - camera_poses.json (camera R, t)" -ForegroundColor Yellow
    Write-Host "  - qa_report.json (quality metrics)" -ForegroundColor Yellow
    Write-Host "  - observations.json (2D-3D correspondences)" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "View QA report:" -ForegroundColor Cyan
    Write-Host "  cat runs\test2_unknown_layout\qa_report.json" -ForegroundColor White
} else {
    Write-Host ""
    Write-Host "❌ Pipeline FAILED (exit code: $LASTEXITCODE)" -ForegroundColor Red
}
