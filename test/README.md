# Test Suite

Unit and integration tests for the photogrammetry pipeline.

## Test Files

- **`test_transforms.py`** - SE(3)/Sim(3) correctness tests
- **`test_phase0_refpoints.py`** - Triangulation correctness tests

## Running Tests

```bash
# Run all tests
python -m pytest test/

# Run specific test file
python -m pytest test/test_transforms.py -v

# Run with coverage
python -m pytest test/ --cov=src --cov-report=html
```

## Test Coverage Goals

- Phase 0: Transform operations, triangulation accuracy
- Phase 1: Calibration loading, undistortion accuracy
- Phase 3: SfM initialization, incremental reconstruction, QA gates
- Phase 4: Frame transformations, alignment residuals
