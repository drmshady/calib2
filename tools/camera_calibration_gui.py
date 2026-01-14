"""
Camera Calibration GUI

Graphical interface for camera calibration using ChArUco boards and validation using AprilTags.

Usage:
    python tools/camera_calibration_gui.py
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import threading
from pathlib import Path
import sys
import json
from datetime import datetime

# Import calibration functions from camera_calibration.py
from camera_calibration import CharucoBoard, calibrate_camera, save_calibration

# Import validation functions from apriltag_pnp_ba.py
from apriltag_pnp_ba import load_calibration, load_tag_layout, process_images


class CalibrationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Calibration & Validation Tool")
        self.root.geometry("900x750")
        
        # Calibration Variables
        self.image_folder = tk.StringVar()
        self.output_file = tk.StringVar()
        self.squares_x = tk.IntVar(value=6)
        self.squares_y = tk.IntVar(value=9)
        self.square_size = tk.DoubleVar(value=10.0)
        self.marker_size = tk.DoubleVar(value=7.0)
        self.dict_type = tk.StringVar(value="DICT_4X4_1000")
        self.min_images = tk.IntVar(value=20)
        self.min_corners = tk.IntVar(value=30)
        self.min_angles = tk.IntVar(value=8)
        self.calib_model = tk.StringVar(value="standard")
        
        # Validation Variables
        self.test_folder = tk.StringVar()
        self.calib_file = tk.StringVar()
        self.layout_file = tk.StringVar()
        self.validation_dict = tk.StringVar(value="DICT_4X4_1000")
        self.validation_output = tk.StringVar()
        self.quality_warning = tk.DoubleVar(value=2.0)
        self.quality_fail = tk.DoubleVar(value=5.0)
        
        # Phase 3/4 Reconstruction Variables
        self.recon_images = tk.StringVar()
        self.recon_calib = tk.StringVar()
        self.recon_output = tk.StringVar()
        self.recon_layout = tk.StringVar()
        self.recon_tag_size = tk.DoubleVar(value=8.8)
        self.recon_layout_mode = tk.StringVar(value="unknown")  # "known" or "unknown"
        self.recon_phase4 = tk.BooleanVar(value=False)
        self.recon_phase4_method = tk.StringVar(value="reference_plate")
        # Default reference plate for unknown layout (tags 100-103)
        self.recon_ref_plate = tk.StringVar(value="calib/fixtures/reference_plate_tags100_103.json")

        # Phase 3 (unknown layout): optional dot-assisted refinement
        self.recon_refine_with_dots = tk.BooleanVar(value=False)
        self.recon_cap_model = tk.StringVar(
            value="aox-photogrammetry-flags/out_aox_flag_v2/models/cap_AOX_FLAG_10x10x10_ID100.json"
        )
        self.recon_dot_roi_half = tk.IntVar(value=30)
        self.recon_dot_max_center_dist_px = tk.DoubleVar(value=14.0)
        self.recon_dot_min_views = tk.IntVar(value=2)
        self.recon_dot_max_reproj_px = tk.DoubleVar(value=3.0)
        self.recon_qa_include_dots = tk.BooleanVar(value=False)
        self.recon_export_include_dots = tk.BooleanVar(value=False)
        self.recon_save_log = tk.BooleanVar(value=True)  # Auto-save logs
        self.recon_accumulated_log = []  # Store all log lines
        
        # Quality Gate Variables
        self.qg_reconstruction_dir = tk.StringVar()
        self.qg_cutoff_percentile = tk.DoubleVar(value=10.0)
        self.qg_criterion = tk.StringVar(value="mean")
        self.qg_sfm = None
        self.qg_image_errors = []  # List of (image_name, mean, max, median) tuples
        self.qg_manual_status = {}  # image_id -> bool (True=REMOVE, False=KEEP)
        self.qg_loaded_metadata = None
        self.qg_loaded_structure_path = None
        
        # Phase 6 IOS Integration Variables
        self.p6_refpoints = tk.StringVar()
        self.p6_implants_u = tk.StringVar()
        self.p6_tag_size = tk.DoubleVar(value=8.8)
        self.p6_ios_file = tk.StringVar()
        self.p6_implants_i = tk.StringVar()
        self.p6_marker_ids = tk.StringVar(value="100,101,102,103")
        self.p6_transform_file = tk.StringVar()
        self.p6_rmse_threshold = tk.DoubleVar(value=5.0)
        self.p6_allow_scale = tk.BooleanVar(value=False)
        self.p6_constellation_u = tk.StringVar()
        self.p6_constellation_i = tk.StringVar()
        # Step 4 (STL) options
        self.p6_use_scanbody_stl = tk.BooleanVar(value=False)
        self.p6_scanbody_stl = tk.StringVar()
        self.p6_scanbody_axis = tk.StringVar(value='z')
        self.p6_scanbody_scale = tk.DoubleVar(value=1.0)
        self.p6_scanbody_recenter = tk.StringVar(value='none')
        self.p6_package_dir = tk.StringVar()
        self.p6_case_name = tk.StringVar(value="Patient001")
        # Mapping interface variables
        self.p6_sb_marker_map = {}  # {sb_position: marker_id}
        self.p6_tooth_positions = {}  # {marker_id: tooth_number}
        
        self.is_running = False
        
        self.create_widgets()
        
    def create_widgets(self):
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create tabs
        calib_tab = ttk.Frame(self.notebook, padding="10")
        validation_tab = ttk.Frame(self.notebook, padding="10")
        reconstruction_tab = ttk.Frame(self.notebook, padding="10")
        quality_gate_tab = ttk.Frame(self.notebook, padding="10")
        phase6_tab = ttk.Frame(self.notebook, padding="10")
        
        self.calib_tab = calib_tab
        self.validation_tab = validation_tab
        self.reconstruction_tab = reconstruction_tab
        self.quality_gate_tab = quality_gate_tab
        self.phase6_tab = phase6_tab

        self.notebook.add(calib_tab, text="Calibration")
        self.notebook.add(validation_tab, text="Validation (PnP + BA)")
        self.notebook.add(reconstruction_tab, text="Phase 3/4 Reconstruction")
        self.notebook.add(quality_gate_tab, text="Quality Gate")
        self.notebook.add(phase6_tab, text="Phase 6: IOS Integration")
        
        # Create calibration interface
        self.create_calibration_tab(calib_tab)
        
        # Create validation interface
        self.create_validation_tab(validation_tab)
        
        # Create reconstruction interface
        self.create_reconstruction_tab(reconstruction_tab)
        
        # Create quality gate interface
        self.create_quality_gate_tab(quality_gate_tab)
        
        # Create Phase 6 interface
        self.create_phase6_tab(phase6_tab)
    
    def create_calibration_tab(self, parent):
        # Main container with padding
        main_frame = ttk.Frame(parent)
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Configure grid weights
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        row = 0
        
        # Title
        title = ttk.Label(main_frame, text="Camera Calibration Tool", font=('Arial', 16, 'bold'))
        title.grid(row=row, column=0, columnspan=3, pady=(0, 20))
        row += 1
        
        # Image Folder Section
        ttk.Label(main_frame, text="Image Folder:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=5)
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.image_folder, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_folder).grid(row=row, column=2)
        row += 1
        
        # Output File Section
        ttk.Label(main_frame, text="Output File:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.output_file, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_output).grid(row=row, column=2)
        row += 1
        
        # Board Configuration Section
        board_frame = ttk.LabelFrame(main_frame, text="ChArUco Board Configuration", padding="10")
        board_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(15, 5))
        board_frame.columnconfigure(1, weight=1)
        board_frame.columnconfigure(3, weight=1)
        row += 1
        
        board_row = 0
        ttk.Label(board_frame, text="Squares X:").grid(row=board_row, column=0, sticky=tk.W, padx=(0, 5))
        ttk.Spinbox(board_frame, from_=3, to=20, textvariable=self.squares_x, width=10).grid(row=board_row, column=1, sticky=tk.W)
        
        ttk.Label(board_frame, text="Squares Y:").grid(row=board_row, column=2, sticky=tk.W, padx=(20, 5))
        ttk.Spinbox(board_frame, from_=3, to=20, textvariable=self.squares_y, width=10).grid(row=board_row, column=3, sticky=tk.W)
        board_row += 1
        
        ttk.Label(board_frame, text="Square Size (mm):").grid(row=board_row, column=0, sticky=tk.W, padx=(0, 5), pady=5)
        ttk.Spinbox(board_frame, from_=1.0, to=100.0, increment=0.1, textvariable=self.square_size, width=10).grid(row=board_row, column=1, sticky=tk.W, pady=5)
        
        ttk.Label(board_frame, text="Marker Size (mm):").grid(row=board_row, column=2, sticky=tk.W, padx=(20, 5), pady=5)
        ttk.Spinbox(board_frame, from_=1.0, to=100.0, increment=0.1, textvariable=self.marker_size, width=10).grid(row=board_row, column=3, sticky=tk.W, pady=5)
        board_row += 1
        
        ttk.Label(board_frame, text="Dictionary:").grid(row=board_row, column=0, sticky=tk.W, padx=(0, 5))
        dict_combo = ttk.Combobox(board_frame, textvariable=self.dict_type, width=20, state='readonly')
        dict_combo['values'] = [
            'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000',
            'DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000',
            'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250', 'DICT_6X6_1000',
            'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250', 'DICT_7X7_1000',
            'DICT_APRILTAG_16h5', 'DICT_APRILTAG_25h9', 'DICT_APRILTAG_36h10', 'DICT_APRILTAG_36h11'
        ]
        dict_combo.grid(row=board_row, column=1, columnspan=3, sticky="we")
        
        # Calibration Parameters Section
        calib_frame = ttk.LabelFrame(main_frame, text="Calibration Parameters", padding="10")
        calib_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=5)
        calib_frame.columnconfigure(1, weight=1)
        calib_frame.columnconfigure(3, weight=1)
        row += 1
        
        calib_row = 0
        ttk.Label(calib_frame, text="Min Images:").grid(row=calib_row, column=0, sticky=tk.W, padx=(0, 5))
        ttk.Spinbox(calib_frame, from_=10, to=100, textvariable=self.min_images, width=10).grid(row=calib_row, column=1, sticky=tk.W)
        
        ttk.Label(calib_frame, text="Min Corners/Image:").grid(row=calib_row, column=2, sticky=tk.W, padx=(20, 5))
        ttk.Spinbox(calib_frame, from_=10, to=100, textvariable=self.min_corners, width=10).grid(row=calib_row, column=3, sticky=tk.W)
        calib_row += 1
        
        ttk.Label(calib_frame, text="Min Unique Angles:").grid(row=calib_row, column=0, sticky=tk.W, padx=(0, 5), pady=5)
        ttk.Spinbox(calib_frame, from_=4, to=16, textvariable=self.min_angles, width=10).grid(row=calib_row, column=1, sticky=tk.W, pady=5)
        
        ttk.Label(calib_frame, text="Calibration Model:").grid(row=calib_row, column=2, sticky=tk.W, padx=(20, 5), pady=5)
        model_combo = ttk.Combobox(calib_frame, textvariable=self.calib_model, width=18, state='readonly')
        model_combo['values'] = ['standard', 'rational', 'thin-prism', 'tilted']
        model_combo.grid(row=calib_row, column=3, sticky=tk.W, pady=5)
        
        # Control Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=row, column=0, columnspan=3, pady=(15, 10))
        row += 1
        
        self.run_button = ttk.Button(button_frame, text="Run Calibration", command=self.run_calibration, width=20)
        self.run_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_calibration, state=tk.DISABLED, width=15)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(button_frame, text="Clear Log", command=self.clear_log, width=15).pack(side=tk.LEFT, padx=5)
        
        # Progress bar
        self.progress = ttk.Progressbar(main_frame, mode='indeterminate')
        self.progress.grid(row=row, column=0, columnspan=3, sticky="we", pady=(0, 10))
        row += 1
        
        # Log output
        ttk.Label(main_frame, text="Calibration Log:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W)
        row += 1
        
        self.log_text = scrolledtext.ScrolledText(main_frame, height=15, width=90, wrap=tk.WORD, state=tk.DISABLED)
        self.log_text.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(0, 10))
        main_frame.rowconfigure(row, weight=1)
        
    def browse_folder(self):
        folder = filedialog.askdirectory(title="Select Image Folder")
        if folder:
            self.image_folder.set(folder)
            # Auto-suggest output file
            if not self.output_file.get():
                self.output_file.set(str(Path(folder) / "camera_intrinsics.json"))
    
    def browse_output(self):
        file = filedialog.asksaveasfilename(
            title="Save Calibration Results",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file:
            self.output_file.set(file)
    
    def log(self, message):
        """Add message to log window."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.root.update_idletasks()
    
    def clear_log(self):
        """Clear log window."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def validate_inputs(self):
        """Validate user inputs."""
        if not self.image_folder.get():
            messagebox.showerror("Error", "Please select an image folder")
            return False
        
        if not Path(self.image_folder.get()).exists():
            messagebox.showerror("Error", "Image folder does not exist")
            return False
        
        if not self.output_file.get():
            messagebox.showerror("Error", "Please specify an output file")
            return False
        
        return True
    
    def run_calibration(self):
        """Run calibration in a separate thread."""
        if not self.validate_inputs():
            return
        
        if self.is_running:
            messagebox.showwarning("Warning", "Calibration is already running")
            return
        
        # Disable controls
        self.run_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.is_running = True
        self.progress.start()
        
        # Clear previous log
        self.clear_log()
        
        # Run in thread
        thread = threading.Thread(target=self._calibration_thread, daemon=True)
        thread.start()
    
    def _calibration_thread(self):
        """Calibration worker thread."""
        try:
            self.log("=" * 80)
            self.log("Starting Camera Calibration")
            self.log("=" * 80)
            
            # Find images
            image_folder = Path(self.image_folder.get())
            image_paths = []
            for ext in ['*.jpg', '*.jpeg', '*.png', '*.tif', '*.tiff', '*.JPG', '*.JPEG', '*.PNG', '*.TIF', '*.TIFF']:
                image_paths.extend(image_folder.glob(ext))
            image_paths = sorted(set(image_paths))
            
            if not image_paths:
                self.log(f"ERROR: No images found in {image_folder}")
                self.finish_calibration(success=False)
                return
            
            self.log(f"Found {len(image_paths)} images in {image_folder}")
            self.log("")
            
            # Create board
            board = CharucoBoard(
                squares_x=self.squares_x.get(),
                squares_y=self.squares_y.get(),
                square_size_mm=self.square_size.get(),
                marker_size_mm=self.marker_size.get(),
                dict_type=self.dict_type.get()
            )
            
            self.log(f"Board Configuration:")
            self.log(f"  Squares: {self.squares_x.get()}×{self.squares_y.get()}")
            self.log(f"  Square Size: {self.square_size.get()} mm")
            self.log(f"  Marker Size: {self.marker_size.get()} mm")
            self.log(f"  Dictionary: {self.dict_type.get()}")
            self.log(f"  Calibration Model: {self.calib_model.get()}")
            self.log("")
            
            # Map calibration model to flags
            import cv2
            calib_model_flags = {
                "standard": 0,
                "rational": cv2.CALIB_RATIONAL_MODEL,
                "thin-prism": cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_THIN_PRISM_MODEL,
                "tilted": cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_THIN_PRISM_MODEL | cv2.CALIB_TILTED_MODEL
            }
            
            # Redirect stdout to log
            original_stdout = sys.stdout
            sys.stdout = LogRedirector(self.log)
            
            try:
                # Run calibration
                results = calibrate_camera(
                    [str(p) for p in image_paths],
                    board,
                    min_images=self.min_images.get(),
                    min_corners_per_image=self.min_corners.get(),
                    min_unique_angles=self.min_angles.get(),
                    calib_flags=calib_model_flags[self.calib_model.get()]
                )
                
                # Save results
                save_calibration(results, self.output_file.get(), board)
                
                self.log("")
                self.log("=" * 80)
                self.log("✓ Calibration Successful!")
                self.log("=" * 80)
                self.log(f"Results saved to: {self.output_file.get()}")
                
                self.finish_calibration(success=True)
                
            finally:
                sys.stdout = original_stdout
                
        except Exception as e:
            self.log("")
            self.log("=" * 80)
            self.log(f"✗ Calibration Failed: {str(e)}")
            self.log("=" * 80)
            self.finish_calibration(success=False)
    
    def stop_calibration(self):
        """Stop calibration (not fully implemented - would need to add cancellation logic)."""
        messagebox.showinfo("Info", "Stop functionality not yet implemented")
    
    def finish_calibration(self, success):
        """Finish calibration and re-enable controls."""
        self.is_running = False
        self.progress.stop()
        self.run_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        
        if success:
            messagebox.showinfo("Success", "Calibration completed successfully!")
    
    def create_validation_tab(self, parent):
        """Create validation tab for PnP + BA testing."""
        main_frame = ttk.Frame(parent)
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        row = 0
        
        # Title
        title = ttk.Label(main_frame, text="Calibration Validation (AprilTag PnP + BA)", font=('Arial', 14, 'bold'))
        title.grid(row=row, column=0, columnspan=3, pady=(0, 15))
        row += 1
        
        # Test Images Folder
        ttk.Label(main_frame, text="Test Images Folder:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=5)
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.test_folder, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_test_folder).grid(row=row, column=2)
        row += 1
        
        # Calibration File
        ttk.Label(main_frame, text="Calibration File (JSON):", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.calib_file, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_calib_file).grid(row=row, column=2)
        row += 1
        
        # Layout File
        ttk.Label(main_frame, text="Tag Layout File (JSON):", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.layout_file, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_layout_file).grid(row=row, column=2)
        row += 1
        
        # Parameters Frame
        param_frame = ttk.LabelFrame(main_frame, text="Parameters", padding="10")
        param_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(15, 5))
        param_frame.columnconfigure(1, weight=1)
        row += 1
        
        param_row = 0
        ttk.Label(param_frame, text="Dictionary:").grid(row=param_row, column=0, sticky=tk.W, padx=(0, 5))
        val_dict_combo = ttk.Combobox(param_frame, textvariable=self.validation_dict, width=20, state='readonly')
        val_dict_combo['values'] = [
            'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000',
            'DICT_APRILTAG_16h5', 'DICT_APRILTAG_25h9', 'DICT_APRILTAG_36h10', 'DICT_APRILTAG_36h11'
        ]
        val_dict_combo.grid(row=param_row, column=1, sticky=tk.W)
        param_row += 1
        
        ttk.Label(param_frame, text="Quality Warning (px):").grid(row=param_row, column=0, sticky=tk.W, padx=(0, 5), pady=5)
        ttk.Entry(param_frame, textvariable=self.quality_warning, width=10).grid(row=param_row, column=1, sticky=tk.W, pady=5)
        param_row += 1
        
        ttk.Label(param_frame, text="Quality Fail (px):").grid(row=param_row, column=0, sticky=tk.W, padx=(0, 5), pady=5)
        ttk.Entry(param_frame, textvariable=self.quality_fail, width=10).grid(row=param_row, column=1, sticky=tk.W, pady=5)
        param_row += 1
        
        ttk.Label(param_frame, text="Output File (optional):").grid(row=param_row, column=0, sticky=tk.W, padx=(0, 5), pady=5)
        ttk.Entry(param_frame, textvariable=self.validation_output, width=40).grid(row=param_row, column=1, sticky="we", pady=5)
        
        # Control Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=row, column=0, columnspan=3, pady=(15, 10))
        row += 1
        
        self.val_run_button = ttk.Button(button_frame, text="Run Validation", command=self.run_validation, width=20)
        self.val_run_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(button_frame, text="Clear Log", command=self.clear_log, width=15).pack(side=tk.LEFT, padx=5)
        
        # Progress bar
        self.val_progress = ttk.Progressbar(main_frame, mode='indeterminate')
        self.val_progress.grid(row=row, column=0, columnspan=3, sticky="we", pady=(0, 10))
        row += 1
        
        # Log output
        ttk.Label(main_frame, text="Validation Log:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W)
        row += 1
        
        self.val_log_text = scrolledtext.ScrolledText(main_frame, height=15, width=90, wrap=tk.WORD, state=tk.DISABLED)
        self.val_log_text.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(0, 10))
        main_frame.rowconfigure(row, weight=1)
    
    def browse_test_folder(self):
        folder = filedialog.askdirectory(title="Select Test Images Folder")
        if folder:
            self.test_folder.set(folder)
    
    def browse_calib_file(self):
        file = filedialog.askopenfilename(
            title="Select Calibration File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file:
            self.calib_file.set(file)
    
    def browse_layout_file(self):
        file = filedialog.askopenfilename(
            title="Select Tag Layout File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file:
            self.layout_file.set(file)
    
    def val_log(self, message):
        """Add message to validation log window."""
        self.val_log_text.config(state=tk.NORMAL)
        self.val_log_text.insert(tk.END, message + "\n")
        self.val_log_text.see(tk.END)
        self.val_log_text.config(state=tk.DISABLED)
        self.root.update_idletasks()
    
    def clear_val_log(self):
        """Clear validation log window."""
        self.val_log_text.config(state=tk.NORMAL)
        self.val_log_text.delete(1.0, tk.END)
        self.val_log_text.config(state=tk.DISABLED)
    
    def run_validation(self):
        """Run validation in a separate thread."""
        if not self.test_folder.get():
            messagebox.showerror("Error", "Please select test images folder")
            return
        
        if not self.calib_file.get():
            messagebox.showerror("Error", "Please select calibration file")
            return
        
        if not self.layout_file.get():
            messagebox.showerror("Error", "Please select tag layout file")
            return
        
        if self.is_running:
            messagebox.showwarning("Warning", "Validation is already running")
            return
        
        # Disable controls
        self.val_run_button.config(state=tk.DISABLED)
        self.is_running = True
        self.val_progress.start()
        
        # Clear previous log
        self.clear_val_log()
        
        # Run in thread
        thread = threading.Thread(target=self._validation_thread, daemon=True)
        thread.start()
    
    def _validation_thread(self):
        """Validation worker thread."""
        try:
            self.val_log("=" * 80)
            self.val_log("Starting Calibration Validation")
            self.val_log("=" * 80)
            
            # Load calibration
            self.val_log(f"Loading calibration from: {self.calib_file.get()}")
            calib = load_calibration(self.calib_file.get())
            self.val_log(f"  Image size: {calib['image_size'][0]}×{calib['image_size'][1]} px")
            self.val_log(f"  fx={calib['K'][0,0]:.1f}, fy={calib['K'][1,1]:.1f}")
            self.val_log("")
            
            # Load layout
            self.val_log(f"Loading tag layout from: {self.layout_file.get()}")
            layout = load_tag_layout(self.layout_file.get())
            self.val_log(f"  Tag size: {layout['tag_size_mm']} mm")
            self.val_log(f"  Number of tags: {len(layout['tag_corners_3d'])}")
            self.val_log(f"  Tag IDs: {sorted(layout['tag_corners_3d'].keys())}")
            self.val_log("")
            
            # Find images
            image_folder = Path(self.test_folder.get())
            image_paths = []
            for ext in ['*.jpg', '*.jpeg', '*.png', '*.tif', '*.tiff', '*.JPG', '*.JPEG', '*.PNG', '*.TIF', '*.TIFF']:
                image_paths.extend(image_folder.glob(ext))
            image_paths = sorted(set(image_paths))
            
            if not image_paths:
                self.val_log(f"ERROR: No images found in {image_folder}")
                self.finish_validation(success=False)
                return
            
            self.val_log(f"Found {len(image_paths)} images in {image_folder}")
            self.val_log("")
            
            # Redirect stdout to log
            original_stdout = sys.stdout
            sys.stdout = LogRedirector(self.val_log)
            
            try:
                # Run validation
                results = process_images(
                    [str(p) for p in image_paths],
                    calib,
                    layout,
                    self.validation_dict.get(),
                    quality_gate_warning=self.quality_warning.get(),
                    quality_gate_fail=self.quality_fail.get()
                )
                
                # Print summary
                if results['images']:
                    stats = results['statistics']
                    self.val_log("")
                    self.val_log("=" * 80)
                    self.val_log("RESULTS SUMMARY")
                    self.val_log("=" * 80)
                    self.val_log(f"Images processed: {stats['n_images_processed']}")
                    self.val_log("")
                    self.val_log("PnP (initial):")
                    self.val_log(f"  RMS error (mean):   {stats['rms_error_mean_px']:.3f} px")
                    self.val_log(f"  RMS error (median): {stats['rms_error_median_px']:.3f} px")
                    self.val_log(f"  RMS error (max):    {stats['rms_error_max_px']:.3f} px")
                    
                    if results['bundle_adjustment']:
                        ba = results['bundle_adjustment']
                        self.val_log("")
                        self.val_log("Bundle Adjustment:")
                        self.val_log(f"  RMS error (mean):   {stats['rms_error_ba_mean_px']:.3f} px")
                        self.val_log(f"  RMS error (median): {stats['rms_error_ba_median_px']:.3f} px")
                        self.val_log(f"  RMS error (max):    {stats['rms_error_ba_max_px']:.3f} px")
                        self.val_log(f"  Improvement:        {ba['improvement_px']:.3f} px ({ba['improvement_pct']:.1f}%)")
                        self.val_log(f"  Iterations:         {ba['iterations']}")
                        self.val_log(f"  Status:             {'✓ Success' if ba['success'] else '✗ Failed'}")
                    
                    # Quality gate results
                    if 'quality_gate' in results:
                        qg = results['quality_gate']
                        self.val_log("")
                        self.val_log("Quality Gate:")
                        self.val_log(f"  Warning threshold:  {qg['warning_threshold_px']:.1f} px")
                        self.val_log(f"  Fail threshold:     {qg['fail_threshold_px']:.1f} px")
                        self.val_log(f"  Pass rate:          {qg['pass_rate']:.1f}% ({stats['n_images_processed'] - qg['n_failures']}/{stats['n_images_processed']})")
                        
                        if qg['n_failures'] > 0:
                            self.val_log("")
                            self.val_log(f"  ✗ FAILURES ({qg['n_failures']}):")
                            for item in qg['failures']:
                                self.val_log(f"      {item['image']}: {item['rms_error']:.3f} px")
                        
                        if qg['n_warnings'] > 0:
                            self.val_log("")
                            self.val_log(f"  ⚠ WARNINGS ({qg['n_warnings']}):")
                            for item in qg['warnings']:
                                self.val_log(f"      {item['image']}: {item['rms_error']:.3f} px")
                        
                        if qg['n_failures'] == 0 and qg['n_warnings'] == 0:
                            self.val_log("")
                            self.val_log("  ✓ All images passed quality gates")
                
                # Save results if output specified
                if self.validation_output.get():
                    from datetime import datetime
                    output_data = {
                        'timestamp': datetime.now().isoformat(),
                        'calibration_file': self.calib_file.get(),
                        'layout_file': self.layout_file.get(),
                        'dictionary': self.validation_dict.get(),
                        'results': results
                    }
                    
                    output_path = Path(self.validation_output.get())
                    output_path.parent.mkdir(parents=True, exist_ok=True)
                    with open(output_path, 'w') as f:
                        json.dump(output_data, f, indent=2)
                    self.val_log("")
                    self.val_log(f"✓ Results saved to: {output_path}")
                
                self.val_log("")
                self.val_log("=" * 80)
                self.val_log("✓ Validation Complete!")
                self.val_log("=" * 80)
                
                self.finish_validation(success=True)
                
            finally:
                sys.stdout = original_stdout
                
        except Exception as e:
            self.val_log("")
            self.val_log("=" * 80)
            self.val_log(f"✗ Validation Failed: {str(e)}")
            self.val_log("=" * 80)
            import traceback
            self.val_log(traceback.format_exc())
            self.finish_validation(success=False)
    
    def finish_validation(self, success):
        """Finish validation and re-enable controls."""
        self.is_running = False
        try:
            self.val_progress.stop()
        except Exception:
            pass
        try:
            self.val_run_button.config(state=tk.NORMAL)
        except Exception:
            pass

        if success:
            messagebox.showinfo("Success", "Validation completed successfully!")
    
    def create_reconstruction_tab(self, parent):
        """Create Phase 3/4 reconstruction interface."""
        # Create canvas with scrollbar for scrollable content
        canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Pack canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Enable mousewheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        main_frame = scrollable_frame
        main_frame.columnconfigure(1, weight=1)
        
        row = 0
        
        # Title
        title = ttk.Label(main_frame, text="Phase 3 & 4: Multi-View Reconstruction", font=('Arial', 16, 'bold'))
        title.grid(row=row, column=0, columnspan=3, pady=(0, 20))
        row += 1
        
        # Input Images Section
        ttk.Label(main_frame, text="Input Images:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=5)
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.recon_images, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_recon_images).grid(row=row, column=2)
        row += 1
        ttk.Label(main_frame, text="Glob pattern (e.g., data/*.TIF)", font=('Arial', 8)).grid(row=row, column=0, columnspan=2, sticky=tk.W)
        row += 1
        
        # Calibration File Section
        ttk.Label(main_frame, text="Calibration File:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.recon_calib, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_recon_calib).grid(row=row, column=2)
        row += 1
        
        # Output Directory Section
        ttk.Label(main_frame, text="Output Directory:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.recon_output, width=60).grid(row=row, column=0, columnspan=2, sticky="we", padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_recon_output).grid(row=row, column=2)
        row += 1
        
        # Layout Mode Selection
        layout_mode_frame = ttk.LabelFrame(main_frame, text="Layout Mode", padding="10")
        layout_mode_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(15, 5))
        row += 1
        
        ttk.Radiobutton(layout_mode_frame, text="Unknown Layout (Triangulate from observations)", 
                       variable=self.recon_layout_mode, value="unknown", 
                       command=self.toggle_layout_mode).grid(row=0, column=0, sticky=tk.W, pady=5)
        ttk.Label(layout_mode_frame, text="Use for production: 4+ flags without known positions", 
                 font=('Arial', 8), foreground='gray').grid(row=1, column=0, sticky=tk.W, padx=(20, 0))
        
        ttk.Radiobutton(layout_mode_frame, text="Known Layout (Validation with layout file)", 
                       variable=self.recon_layout_mode, value="known", 
                       command=self.toggle_layout_mode).grid(row=2, column=0, sticky=tk.W, pady=(10, 5))
        ttk.Label(layout_mode_frame, text="Use for testing/validation with known tag positions", 
                 font=('Arial', 8), foreground='gray').grid(row=3, column=0, sticky=tk.W, padx=(20, 0))
        
        # Layout File (only for known mode)
        self.layout_file_label = ttk.Label(layout_mode_frame, text="Layout File:")
        self.layout_file_label.grid(row=4, column=0, sticky=tk.W, pady=(10, 5))
        
        layout_entry_frame = ttk.Frame(layout_mode_frame)
        layout_entry_frame.grid(row=5, column=0, sticky="we", pady=(0, 5))
        layout_entry_frame.columnconfigure(0, weight=1)
        
        self.layout_file_entry = ttk.Entry(layout_entry_frame, textvariable=self.recon_layout, width=50)
        self.layout_file_entry.grid(row=0, column=0, sticky="we", padx=(0, 5))
        self.layout_file_button = ttk.Button(layout_entry_frame, text="Browse...", command=self.browse_recon_layout)
        self.layout_file_button.grid(row=0, column=1)
        
        # Initially set layout controls based on default mode
        self.toggle_layout_mode()
        
        # Parameters Frame
        params_frame = ttk.LabelFrame(main_frame, text="Parameters", padding="10")
        params_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(15, 10))
        row += 1
        
        # Tag Size
        ttk.Label(params_frame, text="AprilTag Edge Length (mm):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(params_frame, textvariable=self.recon_tag_size, width=15).grid(row=0, column=1, sticky=tk.W, padx=5)

        # Dot refinement (unknown layout only)
        dot_frame = ttk.LabelFrame(main_frame, text="Dot Refinement (Unknown Layout Only)", padding="10")
        dot_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(0, 10))
        dot_frame.columnconfigure(1, weight=1)
        row += 1

        self.recon_dots_check = ttk.Checkbutton(
            dot_frame,
            text="Refine with dots (AOX flag v2)",
            variable=self.recon_refine_with_dots,
            command=self.toggle_dot_refinement,
        )
        self.recon_dots_check.grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 5))

        ttk.Label(dot_frame, text="Cap model JSON:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.recon_cap_model_entry = ttk.Entry(dot_frame, textvariable=self.recon_cap_model, width=60)
        self.recon_cap_model_entry.grid(row=1, column=1, sticky="we", padx=5)
        self.recon_cap_model_button = ttk.Button(dot_frame, text="Browse...", command=self.browse_recon_cap_model)
        self.recon_cap_model_button.grid(row=1, column=2, padx=5)

        ttk.Label(dot_frame, text="ROI half-size (px):").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.recon_dot_roi_half_entry = ttk.Entry(dot_frame, textvariable=self.recon_dot_roi_half, width=10)
        self.recon_dot_roi_half_entry.grid(row=2, column=1, sticky=tk.W, padx=5, pady=2)

        ttk.Label(dot_frame, text="Max center dist (px):").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        self.recon_dot_max_center_dist_entry = ttk.Entry(dot_frame, textvariable=self.recon_dot_max_center_dist_px, width=10)
        self.recon_dot_max_center_dist_entry.grid(row=3, column=1, sticky=tk.W, padx=5, pady=2)

        ttk.Label(dot_frame, text="Min views:").grid(row=4, column=0, sticky=tk.W, padx=5, pady=2)
        self.recon_dot_min_views_entry = ttk.Entry(dot_frame, textvariable=self.recon_dot_min_views, width=10)
        self.recon_dot_min_views_entry.grid(row=4, column=1, sticky=tk.W, padx=5, pady=2)

        ttk.Label(dot_frame, text="Max reproj (px):").grid(row=5, column=0, sticky=tk.W, padx=5, pady=2)
        self.recon_dot_max_reproj_entry = ttk.Entry(dot_frame, textvariable=self.recon_dot_max_reproj_px, width=10)
        self.recon_dot_max_reproj_entry.grid(row=5, column=1, sticky=tk.W, padx=5, pady=2)

        self.recon_qa_include_dots_check = ttk.Checkbutton(
            dot_frame, text="Include dots in QA gates", variable=self.recon_qa_include_dots
        )
        self.recon_qa_include_dots_check.grid(row=6, column=0, columnspan=3, sticky=tk.W, padx=5, pady=(6, 0))

        self.recon_export_include_dots_check = ttk.Checkbutton(
            dot_frame,
            text="Write legacy outputs with dots included",
            variable=self.recon_export_include_dots,
        )
        self.recon_export_include_dots_check.grid(row=7, column=0, columnspan=3, sticky=tk.W, padx=5, pady=(2, 0))

        ttk.Label(
            dot_frame,
            text="Default keeps QA + legacy outputs tag-only; dots export separately.",
            font=('Arial', 8),
            foreground='gray',
        ).grid(row=8, column=0, columnspan=3, sticky=tk.W, padx=5, pady=(4, 0))

        # Initialize dot controls now that widgets exist
        self.toggle_dot_refinement()
        
        # Phase 4 Options Frame
        phase4_frame = ttk.LabelFrame(main_frame, text="Phase 4: User Frame Transform", padding="10")
        phase4_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(10, 10))
        row += 1
        
        ttk.Checkbutton(phase4_frame, text="Enable Phase 4 (L→U Transform)", variable=self.recon_phase4, 
                       command=self.toggle_phase4).grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=5)
        
        ttk.Label(phase4_frame, text="Method:").grid(row=1, column=0, sticky=tk.W, padx=(20, 5), pady=5)
        ttk.Radiobutton(phase4_frame, text="Reference Plate (Option U2)", variable=self.recon_phase4_method, 
                       value="reference_plate").grid(row=1, column=1, sticky=tk.W, pady=5)
        ttk.Radiobutton(phase4_frame, text="Implant-based (Option U1)", variable=self.recon_phase4_method, 
                       value="implant_based").grid(row=1, column=2, sticky=tk.W, pady=5)
        
        ttk.Label(phase4_frame, text="Reference Plate File:").grid(row=2, column=0, sticky=tk.W, padx=(20, 5), pady=5)
        self.ref_plate_entry = ttk.Entry(phase4_frame, textvariable=self.recon_ref_plate, width=40, state=tk.DISABLED)
        self.ref_plate_entry.grid(row=2, column=1, sticky="we", padx=5)
        self.ref_plate_button = ttk.Button(phase4_frame, text="Browse...", command=self.browse_ref_plate, state=tk.DISABLED)
        self.ref_plate_button.grid(row=2, column=2, padx=5)
        
        # Progress and Log
        progress_frame = ttk.Frame(main_frame)
        progress_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(10, 5))
        row += 1
        
        self.recon_progress = ttk.Progressbar(progress_frame, mode='indeterminate')
        self.recon_progress.pack(fill=tk.X, pady=5)
        
        # Log Output
        log_frame = ttk.LabelFrame(main_frame, text="Reconstruction Log", padding="5")
        log_frame.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(5, 10))
        main_frame.rowconfigure(row, weight=1)
        row += 1
        
        # Log controls
        log_controls = ttk.Frame(log_frame)
        log_controls.pack(fill=tk.X, pady=(0, 5))
        ttk.Checkbutton(log_controls, text="Auto-save log to output directory", 
                       variable=self.recon_save_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(log_controls, text="Save Log Now", 
                  command=self.save_reconstruction_log).pack(side=tk.LEFT, padx=5)
        
        self.recon_log = scrolledtext.ScrolledText(log_frame, height=15, width=80, state=tk.DISABLED)
        self.recon_log.pack(fill=tk.BOTH, expand=True)
        
        # Run Button
        self.recon_run_button = ttk.Button(main_frame, text="Run Reconstruction", 
                                          command=self.run_reconstruction, style='Accent.TButton')
        self.recon_run_button.grid(row=row, column=0, columnspan=3, pady=(10, 0))
    
    def toggle_phase4(self):
        """Toggle Phase 4 controls."""
        if self.recon_phase4.get():
            self.ref_plate_entry.config(state=tk.NORMAL)
            self.ref_plate_button.config(state=tk.NORMAL)
        else:
            self.ref_plate_entry.config(state=tk.DISABLED)
            self.ref_plate_button.config(state=tk.DISABLED)
    
    def browse_recon_images(self):
        """Browse for reconstruction image folder."""
        folder = filedialog.askdirectory(title="Select Image Folder")
        if folder:
            self.recon_images.set(folder + "/*.TIF")
    
    def browse_recon_calib(self):
        """Browse for calibration file."""
        file = filedialog.askopenfilename(
            title="Select Calibration File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file:
            self.recon_calib.set(file)
    
    def browse_recon_output(self):
        """Browse for output directory."""
        folder = filedialog.askdirectory(title="Select Output Directory")
        if folder:
            self.recon_output.set(folder)
    
    def browse_recon_layout(self):
        """Browse for layout file."""
        file = filedialog.askopenfilename(
            title="Select Layout File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file:
            self.recon_layout.set(file)

    def browse_recon_cap_model(self):
        """Browse for AOX cap model JSON used for dot refinement."""
        file = filedialog.askopenfilename(
            title="Select Cap Model JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if file:
            self.recon_cap_model.set(file)

    def toggle_dot_refinement(self):
        """Enable/disable dot refinement controls based on layout mode and checkbox."""
        in_unknown_layout = self.recon_layout_mode.get() == "unknown"
        enabled = bool(self.recon_refine_with_dots.get()) and in_unknown_layout

        state_all = tk.NORMAL if in_unknown_layout else tk.DISABLED
        state_details = tk.NORMAL if enabled else tk.DISABLED

        if hasattr(self, "recon_dots_check"):
            try:
                self.recon_dots_check.config(state=state_all)
            except Exception:
                pass

        for widget_name in (
            "recon_cap_model_entry",
            "recon_cap_model_button",
            "recon_dot_roi_half_entry",
            "recon_dot_max_center_dist_entry",
            "recon_dot_min_views_entry",
            "recon_dot_max_reproj_entry",
            "recon_qa_include_dots_check",
            "recon_export_include_dots_check",
        ):
            widget = getattr(self, widget_name, None)
            if widget is None:
                continue
            try:
                widget.config(state=state_details)
            except Exception:
                pass
    
    def toggle_layout_mode(self):
        """Toggle layout file controls based on mode."""
        if self.recon_layout_mode.get() == "known":
            self.layout_file_label.config(state=tk.NORMAL)
            self.layout_file_entry.config(state=tk.NORMAL)
            self.layout_file_button.config(state=tk.NORMAL)
            # Set default reference plate for known layout (tags 1-4)
            if not self.recon_ref_plate.get() or "tags100_103" in self.recon_ref_plate.get():
                self.recon_ref_plate.set("calib/fixtures/reference_plate_4tags.json")
        else:
            self.layout_file_label.config(state=tk.DISABLED)
            self.layout_file_entry.config(state=tk.DISABLED)
            self.layout_file_button.config(state=tk.DISABLED)
            # Set default reference plate for unknown layout (tags 100-103)
            if not self.recon_ref_plate.get() or "4tags.json" in self.recon_ref_plate.get():
                self.recon_ref_plate.set("calib/fixtures/reference_plate_tags100_103.json")

        # Dot refinement only applies to unknown-layout pipeline
        self.toggle_dot_refinement()
    
    def browse_ref_plate(self):
        """Browse for reference plate file."""
        file = filedialog.askopenfilename(
            title="Select Reference Plate File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file:
            self.recon_ref_plate.set(file)
    
    def recon_log_text(self, text):
        """Add text to reconstruction log."""
        self.recon_log.config(state=tk.NORMAL)
        self.recon_log.insert(tk.END, text + "\n")
        self.recon_log.see(tk.END)
        self.recon_log.config(state=tk.DISABLED)
        self.recon_accumulated_log.append(text)
        self.root.update()
    
    def run_reconstruction(self):
        """Run Phase 3/4 reconstruction pipeline."""
        # Validate inputs
        if not self.recon_images.get():
            messagebox.showerror("Error", "Please specify input images!")
            return
        
        if not self.recon_calib.get():
            messagebox.showerror("Error", "Please specify calibration file!")
            return
        
        if not self.recon_output.get():
            messagebox.showerror("Error", "Please specify output directory!")
            return
        
        if self.recon_layout_mode.get() == "known" and not self.recon_layout.get():
            messagebox.showerror("Error", "Please specify layout file for known layout mode!")
            return
        
        if self.recon_phase4.get() and self.recon_phase4_method.get() == "reference_plate" and not self.recon_ref_plate.get():
            messagebox.showerror("Error", "Please specify reference plate file for Phase 4!")
            return
        
        if self.is_running:
            messagebox.showwarning("Warning", "Reconstruction is already running!")
            return
        
        # Disable button and start progress
        self.is_running = True
        self.recon_run_button.config(state=tk.DISABLED)
        self.recon_progress.start()
        
        # Clear log and accumulated text
        self.recon_log.config(state=tk.NORMAL)
        self.recon_log.delete(1.0, tk.END)
        self.recon_log.config(state=tk.DISABLED)
        self.recon_accumulated_log = []  # Clear accumulated log
        
        # Run in separate thread
        thread = threading.Thread(target=self._run_reconstruction_worker)
        thread.daemon = True
        thread.start()
    
    def _run_reconstruction_worker(self):
        """Worker thread for reconstruction."""
        try:
            # Add paths
            # Ensure src/ modules win over legacy tools/ modules (bundle_adjustment name collision)
            src_dir = str(Path(__file__).parent.parent / "src")
            if src_dir in sys.path:
                sys.path.remove(src_dir)
            sys.path.insert(0, src_dir)

            # If an older tools/bundle_adjustment.py was already imported, purge it
            mod = sys.modules.get('bundle_adjustment')
            if mod is not None:
                mod_file = getattr(mod, '__file__', '') or ''
                if mod_file.replace('\\', '/').endswith('/tools/bundle_adjustment.py'):
                    del sys.modules['bundle_adjustment']
            
            # Import reconstruction functions
            from glob import glob
            
            # Redirect stdout
            original_stdout = sys.stdout
            sys.stdout = LogRedirector(self.recon_log_text)
            
            try:
                # Find images
                image_paths = sorted(glob(self.recon_images.get()))
                
                if not image_paths:
                    self.recon_log_text(f"ERROR: No images found matching pattern: {self.recon_images.get()}")
                    self.finish_reconstruction(success=False)
                    return
                
                self.recon_log_text(f"Found {len(image_paths)} images")
                self.recon_log_text(f"Layout mode: {self.recon_layout_mode.get().upper()}")
                self.recon_log_text("")
                
                # Run Phase 3 with appropriate pipeline
                if self.recon_layout_mode.get() == "unknown":
                    # Unknown layout: triangulate from observations
                    import phase3_unknown_layout_pipeline
                    
                    sfm, phase3_metadata = phase3_unknown_layout_pipeline.run_unknown_layout_pipeline(
                        image_paths=image_paths,
                        calib_file=self.recon_calib.get(),
                        output_dir=self.recon_output.get(),
                        tag_size_mm=self.recon_tag_size.get(),
                        refine_with_dots=bool(self.recon_refine_with_dots.get()),
                        cap_model_file=(self.recon_cap_model.get().strip() or None),
                        dot_roi_half=int(self.recon_dot_roi_half.get()),
                        dot_max_center_dist_px=float(self.recon_dot_max_center_dist_px.get()),
                        dot_min_views=int(self.recon_dot_min_views.get()),
                        dot_max_reproj_px=float(self.recon_dot_max_reproj_px.get()),
                        qa_include_dots=bool(self.recon_qa_include_dots.get()),
                        export_include_dots=bool(self.recon_export_include_dots.get()),
                        phase4_enabled=self.recon_phase4.get(),
                        reference_plate_file=self.recon_ref_plate.get() if self.recon_phase4.get() else None,
                        verbose=True
                    )
                else:
                    # Known layout: use PnP-based pipeline
                    import phase3_test_pipeline
                    
                    sfm, phase3_metadata = phase3_test_pipeline.run_phase3_pipeline(
                        image_paths=image_paths,
                        calib_file=self.recon_calib.get(),
                        output_dir=self.recon_output.get(),
                        layout_file=self.recon_layout.get(),
                        tag_size_mm=self.recon_tag_size.get(),
                        verbose=True
                    )

                # Log dot refinement stats if applicable
                if self.recon_layout_mode.get() == "unknown" and phase3_metadata.get("dot_refinement", {}).get("enabled"):
                    dot_stats = phase3_metadata.get("dot_refinement", {})
                    points_added = dot_stats.get("points_added", 0)
                    tracks_used = dot_stats.get("tracks_used", 0)
                    if points_added > 0:
                        self.recon_log_text("")
                        self.recon_log_text(f"✓ Dot refinement: added {points_added} points (from {tracks_used} tracks)")

                # Always allow outlier analysis + re-opt even if QA failed.
                if not phase3_metadata.get("qa_passed", False):
                    self.recon_log_text("")
                    self.recon_log_text("⚠ Phase 3 QA did not pass. Use the Quality Gate tab to remove outlier images and re-run BA + QA.")

                # Kick the user into Quality Gate with this output directory.
                self.root.after(0, lambda: self.open_quality_gate(self.recon_output.get()))
                
                # Run Phase 4 if enabled (for known layout only; unknown layout handles it internally)
                if self.recon_phase4.get() and self.recon_layout_mode.get() == "known":
                    import phase3_test_pipeline
                    phase4_success, phase4_metadata = phase3_test_pipeline.run_phase4_transform(
                        sfm=sfm,
                        reference_plate_file=self.recon_ref_plate.get(),
                        output_dir=self.recon_output.get(),
                        verbose=True
                    )
                    
                    if not phase4_success:
                        self.recon_log_text("")
                        self.recon_log_text("❌ Phase 4 FAILED - hard-stop validation gates not passed")
                        self.finish_reconstruction(success=False)
                        return
                
                # For unknown layout, check Phase 4 result from metadata
                if self.recon_layout_mode.get() == "unknown" and self.recon_phase4.get():
                    phase4_result = phase3_metadata.get("phase4_result", {})
                    if phase4_result.get("enabled") and not phase4_result.get("passed"):
                        self.recon_log_text("")
                        self.recon_log_text("⚠️  Phase 4 did not fully pass (see log above)")
                
                self.recon_log_text("")
                self.recon_log_text("✅ Reconstruction completed successfully!")
                self.finish_reconstruction(success=True)
                
            finally:
                sys.stdout = original_stdout
                
        except Exception as e:
            self.recon_log_text("")
            self.recon_log_text(f"✗ Reconstruction Failed: {str(e)}")
            import traceback
            self.recon_log_text(traceback.format_exc())
            self.finish_reconstruction(success=False)
    
    def create_quality_gate_tab(self, parent):
        """Create quality gate image filtering interface."""
        main_frame = ttk.Frame(parent)
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(3, weight=1)  # Make table expandable
        
        row = 0
        
        # Title
        title = ttk.Label(main_frame, text="Quality Gate: Image Error Analysis", font=('Arial', 16, 'bold'))
        title.grid(row=row, column=0, pady=(0, 20), sticky=tk.W)
        row += 1
        
        # Load Section
        load_frame = ttk.LabelFrame(main_frame, text="Load Reconstruction", padding="10")
        load_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        load_frame.columnconfigure(1, weight=1)
        row += 1
        
        ttk.Label(load_frame, text="Reconstruction Directory:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(load_frame, textvariable=self.qg_reconstruction_dir, width=60).grid(row=0, column=1, sticky="we", padx=5)
        ttk.Button(load_frame, text="Browse...", command=self.qg_browse_reconstruction).grid(row=0, column=2, padx=5)
        ttk.Button(load_frame, text="Load & Analyze", command=self.qg_load_reconstruction, style='Accent.TButton').grid(row=0, column=3, padx=5)
        
        # Filter Controls
        control_frame = ttk.LabelFrame(main_frame, text="Filter Settings", padding="10")
        control_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        row += 1
        
        # Criterion selection
        criterion_frame = ttk.Frame(control_frame)
        criterion_frame.grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=5)
        ttk.Label(criterion_frame, text="Error Criterion:").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(criterion_frame, text="Mean", variable=self.qg_criterion, value="mean", command=self.qg_update_preview).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(criterion_frame, text="Max", variable=self.qg_criterion, value="max", command=self.qg_update_preview).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(criterion_frame, text="Median", variable=self.qg_criterion, value="median", command=self.qg_update_preview).pack(side=tk.LEFT, padx=5)
        
        # Percentile slider
        ttk.Label(control_frame, text="Remove Worst:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.qg_percentile_label = ttk.Label(control_frame, text="10.0%", font=('Arial', 10, 'bold'))
        self.qg_percentile_label.grid(row=1, column=1, sticky=tk.W, padx=5)
        
        slider_frame = ttk.Frame(control_frame)
        slider_frame.grid(row=2, column=0, columnspan=4, sticky="we", pady=5)
        slider_frame.columnconfigure(0, weight=1)
        
        self.qg_slider = ttk.Scale(slider_frame, from_=0, to=50, orient=tk.HORIZONTAL, 
                                   variable=self.qg_cutoff_percentile, command=self.qg_slider_changed)
        self.qg_slider.grid(row=0, column=0, sticky="we", padx=5)
        
        ttk.Label(slider_frame, text="0%").grid(row=1, column=0, sticky=tk.W, padx=5)
        ttk.Label(slider_frame, text="50%").grid(row=1, column=0, sticky=tk.E, padx=5)
        
        # Statistics display
        stats_frame = ttk.Frame(control_frame)
        stats_frame.grid(row=3, column=0, columnspan=4, sticky="we", pady=10)
        
        self.qg_stats_label = ttk.Label(stats_frame, text="Load reconstruction to see statistics", 
                                        font=('Arial', 9), foreground='gray')
        self.qg_stats_label.pack()
        
        # Image Error Table
        table_frame = ttk.LabelFrame(main_frame, text="Per-Image Errors (sorted worst to best)", padding="5")
        table_frame.grid(row=row, column=0, sticky="nsew", pady=(0, 10))
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)
        row += 1
        
        # Create treeview with scrollbar
        tree_scroll = ttk.Scrollbar(table_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.qg_tree = ttk.Treeview(table_frame, columns=('Image', 'Mean', 'Max', 'Median', 'Status'), 
                                     show='headings', height=15, yscrollcommand=tree_scroll.set)
        tree_scroll.config(command=self.qg_tree.yview)
        
        # Configure columns
        self.qg_tree.heading('Image', text='Image Name')
        self.qg_tree.heading('Mean', text='Mean Error (px)')
        self.qg_tree.heading('Max', text='Max Error (px)')
        self.qg_tree.heading('Median', text='Median Error (px)')
        self.qg_tree.heading('Status', text='Status')
        
        self.qg_tree.column('Image', width=200, anchor=tk.W)
        self.qg_tree.column('Mean', width=120, anchor=tk.CENTER)
        self.qg_tree.column('Max', width=120, anchor=tk.CENTER)
        self.qg_tree.column('Median', width=120, anchor=tk.CENTER)
        self.qg_tree.column('Status', width=100, anchor=tk.CENTER)
        
        # Configure tags for coloring
        self.qg_tree.tag_configure('remove', background='#ffcccc')
        self.qg_tree.tag_configure('keep', background='#ccffcc')
        
        self.qg_tree.pack(fill=tk.BOTH, expand=True)
        
        # Action Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=row, column=0, pady=(10, 0))
        row += 1
        
        ttk.Button(button_frame, text="Apply Filter & Re-optimize", command=self.qg_apply_filter, 
                  style='Accent.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Mark Selected REMOVE", command=lambda: self.qg_mark_selected(True)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Mark Selected KEEP", command=lambda: self.qg_mark_selected(False)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Reset Manual Marks", command=self.qg_reset_manual_marks).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Export Statistics", command=self.qg_export_stats).pack(side=tk.LEFT, padx=5)

        # Double-click toggles KEEP/REMOVE
        self.qg_tree.bind('<Double-1>', self.qg_toggle_selected)

    def open_quality_gate(self, reconstruction_dir: str):
        """Open Quality Gate tab and load a reconstruction directory."""
        if reconstruction_dir:
            self.qg_reconstruction_dir.set(reconstruction_dir)
            try:
                self.qg_load_reconstruction()
            except Exception:
                # qg_load_reconstruction will show its own error dialog
                pass
        try:
            self.notebook.select(self.quality_gate_tab)
        except Exception:
            pass
    
    def qg_browse_reconstruction(self):
        """Browse for reconstruction output directory."""
        folder = filedialog.askdirectory(title="Select Reconstruction Output Directory")
        if folder:
            self.qg_reconstruction_dir.set(folder)
    
    def qg_load_reconstruction(self):
        """Load reconstruction and compute per-image errors."""
        if not self.qg_reconstruction_dir.get():
            messagebox.showerror("Error", "Please select a reconstruction directory!")
            return
        
        recon_dir = Path(self.qg_reconstruction_dir.get())

        # Prefer loading from canonical SfM export; fall back to older structure_L.json
        refpoints_path = recon_dir / "refpoints_L.json"
        structure_path = recon_dir / "structure_L.json"
        if refpoints_path.exists():
            structure_to_load = refpoints_path
        elif structure_path.exists():
            structure_to_load = structure_path
        else:
            messagebox.showerror("Error", f"No refpoints_L.json or structure_L.json found in {recon_dir}")
            return

        self.qg_loaded_structure_path = str(structure_to_load)

        # metadata.json is optional (used only for extra info)
        metadata_path = recon_dir / "metadata.json"

        try:
            # Add src to import path (avoid shadowing by tools/ modules)
            src_dir = str(Path(__file__).parent.parent / "src")
            if src_dir not in sys.path:
                sys.path.insert(0, src_dir)
            
            import numpy as np
            from incremental_sfm import IncrementalSfM
            from image_quality_filter import compute_per_image_errors

            # Load SfM from refpoints_L.json
            sfm = IncrementalSfM(K=np.eye(3, dtype=np.float64))
            sfm.load_from_json(str(structure_to_load))
            self.qg_sfm = sfm

            # Reset manual marks on load
            self.qg_manual_status = {}
            self.qg_loaded_metadata = None

            # Compute per-image errors
            image_errors = compute_per_image_errors(self.qg_sfm)

            if len(image_errors) == 0:
                messagebox.showerror("Error", "No per-image errors computed (no registered cameras/observations?)")
                return

            # Convert to list of tuples for table: (image_id, mean, max, median)
            error_rows = []
            for img_id, stats in image_errors.items():
                error_rows.append(
                    (
                        img_id,
                        float(stats.get('mean_error_px', 0.0)),
                        float(stats.get('max_error_px', 0.0)),
                        float(stats.get('median_error_px', 0.0))
                    )
                )

            # Sort by the current criterion
            criterion = self.qg_criterion.get()
            criterion_idx = {'mean': 1, 'max': 2, 'median': 3}[criterion]
            self.qg_image_errors = sorted(error_rows, key=lambda x: x[criterion_idx], reverse=True)
            
            # Update display
            self.qg_update_table()
            self.qg_update_preview()

            # Optional: load metadata.json for extra info in the dialog (not required)
            extra = ""
            if metadata_path.exists():
                try:
                    with open(metadata_path, 'r') as f:
                        metadata = json.load(f)
                    if isinstance(metadata, dict):
                        self.qg_loaded_metadata = metadata

                        # Prefer Phase 3 metadata fields
                        tag_size = metadata.get('tag_size_mm', None)
                        layout_file = metadata.get('layout_file', None)
                        qa_passed = metadata.get('qa_passed', None)

                        extra_lines = []
                        if tag_size is not None:
                            extra_lines.append(f"  tag_size_mm: {tag_size}")
                        if layout_file:
                            extra_lines.append(f"  layout_file: {layout_file}")
                        if qa_passed is not None:
                            extra_lines.append(f"  qa_passed: {qa_passed}")

                        if extra_lines:
                            extra = "\n\nMetadata:\n" + "\n".join(extra_lines)
                except Exception:
                    extra = ""

            messagebox.showinfo(
                "Success",
                f"Loaded reconstruction from:\n{structure_to_load}\n\n"
                f"Cameras: {len(self.qg_sfm.cameras)}\n"
                f"3D points: {len(self.qg_sfm.points_3d)}"
                f"{extra}"
            )
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load reconstruction:\n{str(e)}")
            import traceback
            traceback.print_exc()
    
    def qg_update_table(self):
        """Update the image error table."""
        # Clear existing items
        for item in self.qg_tree.get_children():
            self.qg_tree.delete(item)
        
        if not self.qg_image_errors:
            return
        
        remove_set = self.qg_get_remove_set()
        
        # Add items
        for i, (image_name, mean_err, max_err, median_err) in enumerate(self.qg_image_errors):
            status = "REMOVE" if image_name in remove_set else "KEEP"
            tag = 'remove' if image_name in remove_set else 'keep'
            
            self.qg_tree.insert('', tk.END, values=(
                image_name,
                f"{mean_err:.3f}",
                f"{max_err:.3f}",
                f"{median_err:.3f}",
                status
            ), tags=(tag,))
    
    def qg_slider_changed(self, value):
        """Handle slider value change."""
        percentile = float(value)
        self.qg_percentile_label.config(text=f"{percentile:.1f}%")
        self.qg_update_preview()
    
    def qg_update_preview(self, event=None):
        """Update the preview statistics and table highlighting."""
        if not self.qg_image_errors:
            return
        
        # Re-sort by current criterion
        criterion = self.qg_criterion.get()
        criterion_idx = {'mean': 1, 'max': 2, 'median': 3}[criterion]
        self.qg_image_errors = sorted(self.qg_image_errors, key=lambda x: x[criterion_idx], reverse=True)
        
        # Update table
        self.qg_update_table()
        
        # Calculate statistics
        n_images = len(self.qg_image_errors)
        remove_set = self.qg_get_remove_set()
        n_remove = len(remove_set)
        n_keep = n_images - n_remove
        
        if n_remove == 0:
            stats_text = f"📊 {n_images} images total - No images will be removed"
            self.qg_stats_label.config(foreground='gray')
        else:
            # Calculate before/after errors
            criterion_values = [img[criterion_idx] for img in self.qg_image_errors]

            before_mean = sum(criterion_values) / len(criterion_values)
            after_values = [row[criterion_idx] for row in self.qg_image_errors if row[0] not in remove_set]
            after_mean = sum(after_values) / len(after_values) if after_values else 0
            
            improvement = (before_mean - after_mean) / before_mean * 100 if before_mean > 0 else 0
            
            worst_images = [img[0] for img in self.qg_image_errors if img[0] in remove_set]
            worst_list = ", ".join(worst_images[:3])
            if len(worst_images) > 3:
                worst_list += f" (+{len(worst_images)-3} more)"

            mode = "manual" if self.qg_manual_status else f"{self.qg_cutoff_percentile.get():.0f}%"
            stats_text = (f"📊 {n_images} images → Remove {n_remove} ({mode}) = {n_keep} remaining\n"
                         f"📉 {criterion.title()} error: {before_mean:.2f}px → {after_mean:.2f}px "
                         f"({improvement:+.1f}% improvement)\n"
                         f"🗑️ Worst: {worst_list}")
            self.qg_stats_label.config(foreground='black')
        
        self.qg_stats_label.config(text=stats_text)
    
    def qg_apply_filter(self):
        """Apply the quality gate filter and re-optimize."""
        if not self.qg_sfm:
            messagebox.showerror("Error", "Please load a reconstruction first!")
            return
        
        images_to_remove = self.qg_get_images_to_remove_ordered()
        n_remove = len(images_to_remove)
        
        if n_remove == 0:
            messagebox.showinfo("Info", "No images to remove at current threshold.")
            return
        
        # Confirm action
        result = messagebox.askyesno("Confirm", 
                                     f"Remove {n_remove} worst images and re-optimize?\n\n"
                                     "This will save filtered results to the same directory.")
        if not result:
            return
        
        try:
            # Add src to import path (avoid shadowing by tools/ modules)
            src_dir = str(Path(__file__).parent.parent / "src")
            if src_dir not in sys.path:
                sys.path.insert(0, src_dir)

            # Purge legacy tools/bundle_adjustment module if already imported
            mod = sys.modules.get('bundle_adjustment')
            if mod is not None:
                mod_file = getattr(mod, '__file__', '') or ''
                if mod_file.replace('\\', '/').endswith('/tools/bundle_adjustment.py'):
                    del sys.modules['bundle_adjustment']
            
            from image_quality_filter import apply_quality_gate_filter
            from bundle_adjustment import bundle_adjust_global
            from reconstruction_qa import run_full_qa
            
            # Apply filter
            sfm_filtered, filter_report = apply_quality_gate_filter(
                sfm=self.qg_sfm,
                remove_image_ids=images_to_remove,
                percentile=self.qg_cutoff_percentile.get(),
                criterion=self.qg_criterion.get(),
                verbose=True
            )

            if filter_report.get('status') != 'success':
                messagebox.showerror(
                    "Error",
                    f"Filtering did not run: {filter_report.get('reason', filter_report.get('status', 'unknown'))}"
                )
                return
            
            # Re-run bundle adjustment
            # Don't fix first camera since we removed images - coordinate frame is already established
            messagebox.showinfo("Info", "Re-running bundle adjustment...")
            
            sfm_refined, ba_info = bundle_adjust_global(
                sfm=sfm_filtered,
                fix_first_camera=False,  # Don't fix - frame already established from full BA
                loss_function='huber',
                loss_scale=1.0,
                verbose=1
            )
            
            # Save results
            output_dir = Path(self.qg_reconstruction_dir.get()) / "filtered"
            output_dir.mkdir(exist_ok=True)
            
            # Export structure (both names for compatibility)
            sfm_refined.export_to_json(str(output_dir / "refpoints_L.json"))
            sfm_refined.export_to_json(str(output_dir / "structure_L.json"))

            # Try to run final QA (uses layout metadata if present)
            apriltag_corners_3d = None
            expected_tag_edge_mm = None
            layout_file = None

            if isinstance(self.qg_loaded_metadata, dict):
                layout_file = self.qg_loaded_metadata.get('layout_file')
                expected_tag_edge_mm = self.qg_loaded_metadata.get('tag_size_mm')

            def _load_apriltag_corners(layout_path: str):
                import numpy as np
                with open(layout_path, 'r') as f:
                    data = json.load(f)
                ts = float(data.get('tag_size_mm', 0.0))
                centers = data.get('centers_mm', {})
                half = ts / 2.0
                template = np.array(
                    [[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]],
                    dtype=np.float64,
                )
                corners = {}
                for tid, c in centers.items():
                    cx, cy = float(c[0]), float(c[1])
                    corners[int(tid)] = template + np.array([cx, cy, 0.0], dtype=np.float64)
                return ts, corners

            if layout_file and Path(str(layout_file)).exists():
                try:
                    ts, corners = _load_apriltag_corners(str(layout_file))
                    apriltag_corners_3d = corners
                    if expected_tag_edge_mm is None:
                        expected_tag_edge_mm = ts
                except Exception:
                    apriltag_corners_3d = None

            if expected_tag_edge_mm is None:
                expected_tag_edge_mm = 8.8

            qa_report = run_full_qa(
                sfm_refined,
                apriltag_corners_3d=apriltag_corners_3d,
                expected_tag_edge_mm=float(expected_tag_edge_mm),
            )

            qa_data = {
                "overall_status": qa_report.overall_status.value,
                "passed": qa_report.passed(),
                "hard_failures": qa_report.hard_failures,
                "warnings": qa_report.warnings,
                "checks": [
                    {
                        "name": check.name,
                        "status": check.status.value,
                        "message": check.message,
                        "details": check.details,
                    }
                    for check in qa_report.checks
                ],
            }
            with open(output_dir / "qa_report.json", 'w') as f:
                json.dump(qa_data, f, indent=2)

            # Export minimal metadata for bookkeeping (GUI loads from refpoints_L.json)
            metadata = {
                'quality_gate': filter_report,
                'bundle_adjustment': ba_info,
                'source_dir': str(Path(self.qg_reconstruction_dir.get()).resolve()),
                'layout_file': layout_file,
                'tag_size_mm': expected_tag_edge_mm,
                'qa_passed': qa_report.passed(),
                'removed_images': images_to_remove,
            }

            with open(output_dir / "metadata.json", 'w') as f:
                json.dump(metadata, f, indent=2)
            
            messagebox.showinfo("Success", 
                               f"Filtered reconstruction saved to:\n{output_dir}\n\n"
                               f"Removed: {filter_report['removed']['n_images_removed']} images\n"
                               f"Remaining cameras: {filter_report['removed']['n_cameras_after']}\n"
                               f"Remaining points: {filter_report['removed']['n_points_after']}\n"
                               f"QA: {qa_data['overall_status']} (passed={qa_data['passed']})")
            
            # Reload to show new results
            self.qg_reconstruction_dir.set(str(output_dir))
            self.qg_load_reconstruction()
            
        except Exception as e:
            messagebox.showerror("Error", f"Filter application failed:\n{str(e)}")
            import traceback
            traceback.print_exc()
    
    def qg_export_stats(self):
        """Export image error statistics to CSV."""
        if not self.qg_image_errors:
            messagebox.showerror("Error", "No data to export!")
            return
        
        file = filedialog.asksaveasfilename(
            title="Save Error Statistics",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if not file:
            return
        
        try:
            import csv
            
            with open(file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Image', 'Mean_Error_px', 'Max_Error_px', 'Median_Error_px'])
                
                for image_name, mean_err, max_err, median_err in self.qg_image_errors:
                    writer.writerow([image_name, f"{mean_err:.6f}", f"{max_err:.6f}", f"{median_err:.6f}"])
            
            messagebox.showinfo("Success", f"Statistics exported to:\n{file}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Export failed:\n{str(e)}")
    
    def save_reconstruction_log(self):
        """Manually save reconstruction log to file."""
        if not self.recon_output.get():
            messagebox.showerror("Error", "No output directory specified!")
            return
        
        if not self.recon_accumulated_log:
            messagebox.showwarning("Warning", "Log is empty!")
            return
        
        try:
            self._save_log_to_file()
            messagebox.showinfo("Success", f"Log saved to:\n{Path(self.recon_output.get()) / 'reconstruction_log.txt'}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save log: {e}")
    
    def _save_log_to_file(self):
        """Internal method to save accumulated log to file."""
        from datetime import datetime
        
        output_dir = Path(self.recon_output.get())
        output_dir.mkdir(parents=True, exist_ok=True)
        
        log_file = output_dir / "reconstruction_log.txt"
        
        with open(log_file, 'w', encoding='utf-8') as f:
            f.write("="*80 + "\n")
            f.write("PHASE 3/4 RECONSTRUCTION LOG\n")
            f.write("="*80 + "\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Output Directory: {output_dir}\n")
            f.write(f"Layout Mode: {self.recon_layout_mode.get().upper()}\n")
            f.write(f"Tag Size: {self.recon_tag_size.get()} mm\n")
            f.write(f"Phase 4: {'ENABLED' if self.recon_phase4.get() else 'DISABLED'}\n")
            f.write("="*80 + "\n\n")
            
            for line in self.recon_accumulated_log:
                f.write(line + "\n")
        
        self.recon_log_text(f"Log saved to: {log_file}")
    
    def finish_reconstruction(self, success):
        """Finish reconstruction and re-enable controls."""
        self.is_running = False
        self.recon_progress.stop()
        self.recon_run_button.config(state=tk.NORMAL)
        
        # Auto-save log if enabled
        if self.recon_save_log.get() and self.recon_output.get():
            try:
                self._save_log_to_file()
            except Exception as e:
                self.recon_log_text(f"Warning: Could not auto-save log: {e}")
        
        if success:
            messagebox.showinfo("Success", "Reconstruction finished. Use the Quality Gate tab to review per-image errors and optionally re-optimize.")
        else:
            messagebox.showerror("Error", "Reconstruction failed. See the Reconstruction Log for details.")

    def qg_get_remove_set(self):
        """Compute the set of images currently marked for removal."""
        if self.qg_manual_status:
            return {img_id for img_id, remove in self.qg_manual_status.items() if remove}

        n_images = len(self.qg_image_errors)
        n_remove = max(0, int(n_images * self.qg_cutoff_percentile.get() / 100))
        return {img_id for (img_id, _, _, _) in self.qg_image_errors[:n_remove]}

    def qg_get_images_to_remove_ordered(self):
        """Return images marked for removal in worst-to-best order."""
        remove_set = self.qg_get_remove_set()
        return [row[0] for row in self.qg_image_errors if row[0] in remove_set]

    def qg_mark_selected(self, remove: bool):
        """Mark selected rows as REMOVE/KEEP (manual override)."""
        for item in self.qg_tree.selection():
            values = self.qg_tree.item(item, 'values')
            if not values:
                continue
            img_id = values[0]
            self.qg_manual_status[img_id] = bool(remove)
        self.qg_update_preview()

    def qg_reset_manual_marks(self):
        self.qg_manual_status = {}
        self.qg_update_preview()

    def qg_toggle_selected(self, event=None):
        """Double-click toggles selected image between REMOVE/KEEP."""
        item = self.qg_tree.identify_row(event.y) if event is not None else None
        if not item:
            return
        values = self.qg_tree.item(item, 'values')
        if not values:
            return
        img_id = values[0]
        current = self.qg_manual_status.get(img_id, None)
        if current is None:
            # Default toggle based on current computed status
            current_remove = img_id in self.qg_get_remove_set()
            self.qg_manual_status[img_id] = not current_remove
        else:
            self.qg_manual_status[img_id] = not current
        self.qg_update_preview()

    # ===== Phase 6: IOS Integration =====
    
    def create_phase6_tab(self, parent):
        """Create Phase 6 IOS Integration interface."""
        # Scrollable container so all Phase 6 fields remain accessible
        outer = ttk.Frame(parent)
        outer.grid(row=0, column=0, sticky="nsew")

        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=1)

        canvas = tk.Canvas(outer, highlightthickness=0)
        vscroll = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vscroll.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        vscroll.grid(row=0, column=1, sticky="ns")

        main_frame = ttk.Frame(canvas)
        main_frame.columnconfigure(0, weight=1)
        window_id = canvas.create_window((0, 0), window=main_frame, anchor="nw")

        def _on_frame_configure(event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _on_canvas_configure(event):
            # Keep inner frame width matched to canvas
            canvas.itemconfigure(window_id, width=event.width)

        main_frame.bind("<Configure>", _on_frame_configure)
        canvas.bind("<Configure>", _on_canvas_configure)

        def _bind_mousewheel(_event):
            canvas.bind_all("<MouseWheel>", _on_mousewheel)

        def _unbind_mousewheel(_event):
            canvas.unbind_all("<MouseWheel>")

        def _on_mousewheel(event):
            # Windows: event.delta is multiple of 120
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind("<Enter>", _bind_mousewheel)
        canvas.bind("<Leave>", _unbind_mousewheel)
        
        row = 0
        
        # Title and description
        title = ttk.Label(main_frame, text="Phase 6: IOS Integration", font=('Arial', 16, 'bold'))
        title.grid(row=row, column=0, pady=(0, 5), sticky=tk.W)
        row += 1
        
        desc = ttk.Label(main_frame, text="Align photogrammetry implants with IOS scan body data for CAD/CAM workflow.",
                        font=('Arial', 10))
        desc.grid(row=row, column=0, pady=(0, 5), sticky=tk.W)
        row += 1
        
        # Workflow overview
        workflow_text = "📋 Workflow: Extract implants → Convert IOS data → Compute alignment → Generate STLs → Export package"
        workflow = ttk.Label(main_frame, text=workflow_text, font=('Arial', 9), foreground='#555')
        workflow.grid(row=row, column=0, pady=(0, 15), sticky=tk.W)
        row += 1

        # Quick setup (explicit required file selection)
        quick_frame = ttk.LabelFrame(main_frame, text="⚡ Quick Setup (Required Files)", padding="10")
        quick_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        quick_frame.columnconfigure(1, weight=1)
        row += 1

        qs_row = 0
        ttk.Label(quick_frame, text="Implants_U.json (U-frame):").grid(row=qs_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(quick_frame, textvariable=self.p6_implants_u, width=50).grid(row=qs_row, column=1, sticky="we", padx=5)
        ttk.Button(quick_frame, text="Browse...", command=lambda: self.p6_implants_u.set(
            filedialog.askopenfilename(title="Select implants_U.json", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        )).grid(row=qs_row, column=2, padx=5)
        qs_row += 1

        ttk.Label(quick_frame, text="Implants_I.json (IOS I-frame):").grid(row=qs_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(quick_frame, textvariable=self.p6_implants_i, width=50).grid(row=qs_row, column=1, sticky="we", padx=5)
        ttk.Button(quick_frame, text="Browse...", command=lambda: self.p6_implants_i.set(
            filedialog.askopenfilename(title="Select implants_I.json", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        )).grid(row=qs_row, column=2, padx=5)
        qs_row += 1

        ttk.Label(quick_frame, text="Output T_I_from_U.json:").grid(row=qs_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(quick_frame, textvariable=self.p6_transform_file, width=50).grid(row=qs_row, column=1, sticky="we", padx=5)
        ttk.Button(quick_frame, text="Browse...", command=lambda: self.p6_transform_file.set(
            filedialog.asksaveasfilename(title="Save T_I_from_U.json", defaultextension=".json", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        )).grid(row=qs_row, column=2, padx=5)
        qs_row += 1

        ttk.Label(
            quick_frame,
            text="Tip: You can skip Step 1/2 if you already have these JSON files.",
            font=('Arial', 8),
            foreground='gray'
        ).grid(row=qs_row, column=0, columnspan=3, sticky=tk.W, padx=5, pady=(2, 0))
        
        # Step 1: Extract Implants from Photogrammetry
        step1_frame = ttk.LabelFrame(main_frame, text="📍 Step 1: Extract Implants from Photogrammetry", padding="15")
        step1_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        step1_frame.columnconfigure(1, weight=1)
        row += 1
        
        s1_row = 0
        
        # Helper tool section (prominent placement)
        helper_frame = ttk.LabelFrame(step1_frame, text="🔧 Helper Tool", padding="10")
        helper_frame.grid(row=s1_row, column=0, columnspan=3, sticky="we", pady=(0, 15))
        s1_row += 1
        
        helper_inner = ttk.Frame(helper_frame)
        helper_inner.pack(fill=tk.X)
        
        ttk.Label(helper_inner, text="Don't have refpoints_U.json? Convert from Phase 3 output:", 
                 font=('Arial', 9)).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(helper_inner, text="📄 Create from structure_L.json", 
                  command=self.p6_create_refpoints_u).pack(side=tk.LEFT)
        
        ttk.Label(step1_frame, text="Input refpoints_U.json:").grid(row=s1_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step1_frame, textvariable=self.p6_refpoints, width=50).grid(row=s1_row, column=1, sticky="we", padx=5)
        ttk.Button(step1_frame, text="Browse...", command=lambda: self.p6_refpoints.set(
            filedialog.askopenfilename(title="Select refpoints_U.json", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        )).grid(row=s1_row, column=2, padx=5)
        s1_row += 1
        
        ttk.Label(step1_frame, text="Output implants_U.json:").grid(row=s1_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step1_frame, textvariable=self.p6_implants_u, width=50).grid(row=s1_row, column=1, sticky="we", padx=5)
        ttk.Button(step1_frame, text="Browse...", command=lambda: self.p6_implants_u.set(
            filedialog.asksaveasfilename(title="Save implants_U.json", defaultextension=".json", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        )).grid(row=s1_row, column=2, padx=5)
        s1_row += 1
        
        ttk.Label(step1_frame, text="Tag Size (mm):").grid(row=s1_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Spinbox(step1_frame, from_=1.0, to=100.0, increment=0.1, textvariable=self.p6_tag_size, width=10).grid(row=s1_row, column=1, sticky=tk.W, padx=5)
        ttk.Button(step1_frame, text="Extract Implants", command=self.p6_extract_implants, style='Accent.TButton').grid(row=s1_row, column=2, padx=5)
        s1_row += 1
        
        # Step 2: Convert IOS Data
        step2_frame = ttk.LabelFrame(main_frame, text="📊 Step 2: Convert IOS Scan Body Data & Mapping", padding="15")
        step2_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        step2_frame.columnconfigure(1, weight=1)
        row += 1
        
        s2_row = 0
        ttk.Label(step2_frame, text="IOS Scan Body Data (.json / .csv):").grid(row=s2_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step2_frame, textvariable=self.p6_ios_file, width=50).grid(row=s2_row, column=1, sticky="we", padx=5)
        ttk.Button(step2_frame, text="Browse...", command=lambda: self.p6_ios_file.set(
            filedialog.askopenfilename(title="Select IOS Data", filetypes=[("JSON", "*.json"), ("CSV", "*.csv"), ("All Files", "*.*")])
        )).grid(row=s2_row, column=2, padx=5)
        s2_row += 1
        
        ttk.Label(step2_frame, text="Output implants_I.json:").grid(row=s2_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step2_frame, textvariable=self.p6_implants_i, width=50).grid(row=s2_row, column=1, sticky="we", padx=5)
        ttk.Button(step2_frame, text="Browse...", command=lambda: self.p6_implants_i.set(
            filedialog.asksaveasfilename(title="Save implants_I.json", defaultextension=".json", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        )).grid(row=s2_row, column=2, padx=5)
        s2_row += 1
        
        # Marker ID to Tooth Position Mapping
        ttk.Label(step2_frame, text="Marker IDs (comma-separated):").grid(row=s2_row, column=0, sticky=tk.W, pady=5, padx=5)
        marker_entry = ttk.Entry(step2_frame, textvariable=self.p6_marker_ids, width=25)
        marker_entry.grid(row=s2_row, column=1, sticky=tk.W, padx=5)
        ttk.Label(step2_frame, text="e.g., 100,101,102,103", font=('Arial', 8), foreground='gray').grid(row=s2_row, column=2, sticky=tk.W, padx=5)
        s2_row += 1
        
        # Mapping editor section (prominent placement)
        mapping_frame = ttk.LabelFrame(step2_frame, text="🎯 Correspondence Mapping", padding="10")
        mapping_frame.grid(row=s2_row, column=0, columnspan=3, sticky="we", pady=(10, 15))
        s2_row += 1
        
        mapping_inner = ttk.Frame(mapping_frame)
        mapping_inner.pack(fill=tk.X)
        
        ttk.Label(mapping_inner, text="Prevent alignment errors by explicitly mapping scan body positions to marker IDs:", 
                 font=('Arial', 9)).pack(anchor=tk.W, pady=(0, 5))
        ttk.Button(mapping_inner, text="🗺️ Open Mapping Editor", command=self.p6_open_mapping_editor).pack(anchor=tk.W)
        
        ttk.Button(step2_frame, text="Convert IOS Data", command=self.p6_convert_ios, style='Accent.TButton').grid(row=s2_row, column=1, pady=5)
        s2_row += 1
        
        # Step 3: Compute Alignment
        step3_frame = ttk.LabelFrame(main_frame, text="🔄 Step 3: Compute Alignment (T_I_from_U)", padding="15")
        step3_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        step3_frame.columnconfigure(1, weight=1)
        row += 1
        
        s3_row = 0
        ttk.Label(step3_frame, text="Output T_I_from_U.json:").grid(row=s3_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step3_frame, textvariable=self.p6_transform_file, width=50).grid(row=s3_row, column=1, sticky="we", padx=5)
        ttk.Button(step3_frame, text="Browse...", command=lambda: self.p6_transform_file.set(
            filedialog.asksaveasfilename(title="Save T_I_from_U.json", defaultextension=".json", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        )).grid(row=s3_row, column=2, padx=5)
        s3_row += 1
        
        ttk.Label(step3_frame, text="RMSE Threshold (mm):").grid(row=s3_row, column=0, sticky=tk.W, pady=5, padx=5)
        rmse_spin = ttk.Spinbox(step3_frame, from_=0.1, to=50.0, increment=0.1, textvariable=self.p6_rmse_threshold, width=10)
        rmse_spin.grid(row=s3_row, column=1, sticky=tk.W, padx=5)
        ttk.Label(step3_frame, text="Max acceptable alignment error", font=('Arial', 8), foreground='gray').grid(row=s3_row, column=2, sticky=tk.W, padx=5)
        s3_row += 1
        
        # Scale estimation option (prominent)
        scale_frame = ttk.Frame(step3_frame)
        scale_frame.grid(row=s3_row, column=0, columnspan=3, sticky="we", pady=(10, 5))
        s3_row += 1
        
        ttk.Checkbutton(scale_frame, text="⚖️ Allow scale estimation (Sim3 transform)", 
                       variable=self.p6_allow_scale, style='Accent.TCheckbutton').pack(anchor=tk.W)
        ttk.Label(scale_frame, text="  Enable when IOS and photogrammetry have different units or scales (~3-4× difference)", 
                 font=('Arial', 9), foreground='#555').pack(anchor=tk.W, padx=(20, 0))
        ttk.Label(scale_frame, text="  ⚠️  Leave unchecked if both systems use same scale (e.g., both in mm)", 
                 font=('Arial', 8), foreground='#888').pack(anchor=tk.W, padx=(20, 0), pady=(2, 0))
        
        ttk.Button(step3_frame, text="Compute Alignment", command=self.p6_compute_alignment, style='Accent.TButton').grid(row=s3_row, column=1, padx=5)
        s3_row += 1
        
        # Step 4: Generate Constellation STL
        step4_frame = ttk.LabelFrame(main_frame, text="🎨 Step 4: Generate Constellation STL Visualization", padding="15")
        step4_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        step4_frame.columnconfigure(1, weight=1)
        row += 1
        
        s4_row = 0

        # Optional scanbody STL template
        scanbody_frame = ttk.LabelFrame(step4_frame, text="Optional: Use Scanbody STL Template", padding="10")
        scanbody_frame.grid(row=s4_row, column=0, columnspan=3, sticky="we", pady=(0, 10), padx=5)
        scanbody_frame.columnconfigure(1, weight=1)
        s4_row += 1

        ttk.Checkbutton(
            scanbody_frame,
            text="Use scanbody STL instead of cylinders/cones",
            variable=self.p6_use_scanbody_stl
        ).grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 8))

        ttk.Label(scanbody_frame, text="Scanbody STL:").grid(row=1, column=0, sticky=tk.W, pady=3)
        ttk.Entry(scanbody_frame, textvariable=self.p6_scanbody_stl, width=50).grid(row=1, column=1, sticky="we", padx=5)
        ttk.Button(scanbody_frame, text="Browse...", command=lambda: self.p6_scanbody_stl.set(
            filedialog.askopenfilename(title="Select scanbody STL", filetypes=[("STL", "*.stl"), ("All Files", "*.*")])
        )).grid(row=1, column=2, padx=5)

        ttk.Label(scanbody_frame, text="Template axis:").grid(row=2, column=0, sticky=tk.W, pady=3)
        ttk.Combobox(scanbody_frame, textvariable=self.p6_scanbody_axis, values=['x', 'y', 'z', '-x', '-y', '-z'], width=6, state='readonly').grid(row=2, column=1, sticky=tk.W, padx=5)
        ttk.Label(scanbody_frame, text="(axis in STL that points along implant axis)", font=('Arial', 8), foreground='gray').grid(row=2, column=2, sticky=tk.W)

        ttk.Label(scanbody_frame, text="STL scale:").grid(row=3, column=0, sticky=tk.W, pady=3)
        ttk.Spinbox(scanbody_frame, from_=0.001, to=1000.0, increment=0.1, textvariable=self.p6_scanbody_scale, width=10).grid(row=3, column=1, sticky=tk.W, padx=5)
        ttk.Label(scanbody_frame, text="(use 1.0 if STL is already in mm)", font=('Arial', 8), foreground='gray').grid(row=3, column=2, sticky=tk.W)

        ttk.Label(scanbody_frame, text="Recenter:").grid(row=4, column=0, sticky=tk.W, pady=3)
        ttk.Combobox(scanbody_frame, textvariable=self.p6_scanbody_recenter, values=['none', 'bbox', 'centroid'], width=10, state='readonly').grid(row=4, column=1, sticky=tk.W, padx=5)
        ttk.Label(scanbody_frame, text="(if STL origin isn't at scanbody center)", font=('Arial', 8), foreground='gray').grid(row=4, column=2, sticky=tk.W)
        ttk.Label(step4_frame, text="U-Constellation Output:").grid(row=s4_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step4_frame, textvariable=self.p6_constellation_u, width=50).grid(row=s4_row, column=1, sticky="we", padx=5)
        ttk.Button(step4_frame, text="Browse...", command=lambda: self.p6_constellation_u.set(
            filedialog.asksaveasfilename(title="Save U_Constellation.stl", defaultextension=".stl", filetypes=[("STL", "*.stl"), ("All Files", "*.*")])
        )).grid(row=s4_row, column=2, padx=5)
        s4_row += 1
        
        ttk.Label(step4_frame, text="I-Constellation Output:").grid(row=s4_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step4_frame, textvariable=self.p6_constellation_i, width=50).grid(row=s4_row, column=1, sticky="we", padx=5)
        ttk.Button(step4_frame, text="Browse...", command=lambda: self.p6_constellation_i.set(
            filedialog.asksaveasfilename(title="Save I_Constellation.stl", defaultextension=".stl", filetypes=[("STL", "*.stl"), ("All Files", "*.*")])
        )).grid(row=s4_row, column=2, padx=5)
        s4_row += 1
        
        ttk.Button(step4_frame, text="Generate Both STLs", command=self.p6_generate_constellations, style='Accent.TButton').grid(row=s4_row, column=1, pady=5)
        s4_row += 1
        
        # Step 5: Export Package
        step5_frame = ttk.LabelFrame(main_frame, text="📦 Step 5: Export IOS Package", padding="15")
        step5_frame.grid(row=row, column=0, sticky="we", pady=(0, 10))
        step5_frame.columnconfigure(1, weight=1)
        row += 1
        
        s5_row = 0
        ttk.Label(step5_frame, text="Package Output Directory:").grid(row=s5_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step5_frame, textvariable=self.p6_package_dir, width=50).grid(row=s5_row, column=1, sticky="we", padx=5)
        ttk.Button(step5_frame, text="Browse...", command=lambda: self.p6_package_dir.set(
            filedialog.askdirectory(title="Select Package Output Directory")
        )).grid(row=s5_row, column=2, padx=5)
        s5_row += 1
        
        ttk.Label(step5_frame, text="Case Name:").grid(row=s5_row, column=0, sticky=tk.W, pady=5, padx=5)
        ttk.Entry(step5_frame, textvariable=self.p6_case_name, width=30).grid(row=s5_row, column=1, sticky=tk.W, padx=5)
        ttk.Button(step5_frame, text="Export Package", command=self.p6_export_package, style='Accent.TButton').grid(row=s5_row, column=2, padx=5)
        s5_row += 1
        
        # Status/Log
        status_frame = ttk.LabelFrame(main_frame, text="Status Log", padding="10")
        status_frame.grid(row=row, column=0, sticky="nsew", pady=(0, 10))
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(row, weight=1)
        row += 1
        
        self.p6_log = scrolledtext.ScrolledText(status_frame, height=10, width=80, wrap=tk.WORD)
        self.p6_log.grid(row=0, column=0, sticky="nsew")
        
    def p6_log_text(self, text):
        """Append text to Phase 6 log."""
        self.p6_log.insert(tk.END, text + "\n")
        self.p6_log.see(tk.END)
        self.root.update()
    
    def p6_open_mapping_editor(self):
        """Open mapping editor dialog."""
        # Create modal dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Scan Body to Marker ID Mapping")
        dialog.geometry("600x500")
        dialog.transient(self.root)
        dialog.grab_set()
        
        # Instructions
        ttk.Label(dialog, text="Scan Body to Marker ID Mapping", font=('Arial', 14, 'bold')).pack(pady=10)
        
        inst_frame = ttk.Frame(dialog, padding=10)
        inst_frame.pack(fill=tk.X)
        ttk.Label(inst_frame, text="Specify which scan body position corresponds to which AprilTag marker ID.",
                 wraplength=550).pack()
        ttk.Label(inst_frame, text="This prevents mismatches when IOS scan body order differs from marker order.",
                 wraplength=550, foreground='gray', font=('Arial', 9)).pack()
        
        # Mapping table
        table_frame = ttk.LabelFrame(dialog, text="Mapping Configuration", padding=10)
        table_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Get marker IDs
        marker_ids_str = self.p6_marker_ids.get().strip()
        if marker_ids_str:
            marker_ids = [int(x.strip()) for x in marker_ids_str.split(',')]
        else:
            marker_ids = [100, 101, 102, 103]

        marker_ids_choices = [str(m) for m in marker_ids]
        
        # Create mapping inputs
        mapping_entries = {}
        tooth_entries = {}
        
        ttk.Label(table_frame, text="Scan Body", font=('Arial', 10, 'bold')).grid(row=0, column=0, padx=10, pady=5)
        ttk.Label(table_frame, text="Marker ID", font=('Arial', 10, 'bold')).grid(row=0, column=1, padx=10, pady=5)
        ttk.Label(table_frame, text="Tooth # (optional)", font=('Arial', 10, 'bold')).grid(row=0, column=2, padx=10, pady=5)
        ttk.Label(table_frame, text="Position", font=('Arial', 10, 'bold')).grid(row=0, column=3, padx=10, pady=5)
        
        positions = ["Leftmost", "Second", "Third", "Rightmost"]
        
        for idx, marker_id in enumerate(marker_ids, start=1):
            sb_name = f"SB{idx}"
            
            # Scan body label
            ttk.Label(table_frame, text=sb_name, font=('Arial', 10)).grid(row=idx, column=0, padx=10, pady=5)
            
            # Marker ID combobox
            marker_var = tk.StringVar(value=str(marker_id))
            marker_combo = ttk.Combobox(table_frame, textvariable=marker_var, values=marker_ids_choices, 
                                       width=10, state='readonly')
            marker_combo.grid(row=idx, column=1, padx=10, pady=5)
            mapping_entries[sb_name] = marker_var
            
            # Restore previous mapping if exists
            if sb_name in self.p6_sb_marker_map:
                marker_var.set(str(self.p6_sb_marker_map[sb_name]))
            
            # Tooth number entry
            tooth_var = tk.StringVar()
            tooth_entry = ttk.Entry(table_frame, textvariable=tooth_var, width=10)
            tooth_entry.grid(row=idx, column=2, padx=10, pady=5)
            tooth_entries[sb_name] = tooth_var
            
            # Restore previous tooth number if exists
            if marker_id in self.p6_tooth_positions:
                tooth_var.set(self.p6_tooth_positions[marker_id])
            
            # Position indicator
            if idx <= len(positions):
                ttk.Label(table_frame, text=positions[idx-1], foreground='gray', 
                         font=('Arial', 9)).grid(row=idx, column=3, padx=10, pady=5)
        
        # Buttons
        button_frame = ttk.Frame(dialog)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        def save_mapping():
            # Save mapping
            self.p6_sb_marker_map.clear()
            self.p6_tooth_positions.clear()
            
            for sb_name, marker_var in mapping_entries.items():
                marker_id = int(marker_var.get())
                self.p6_sb_marker_map[sb_name] = marker_id
                
                # Save tooth number if provided
                tooth_num = tooth_entries[sb_name].get().strip()
                if tooth_num:
                    self.p6_tooth_positions[marker_id] = tooth_num
            
            # Update marker IDs string to match mapping order
            ordered_markers = [self.p6_sb_marker_map.get(f"SB{i+1}", marker_ids[i]) 
                             for i in range(len(marker_ids))]
            self.p6_marker_ids.set(','.join(map(str, ordered_markers)))
            
            self.p6_log_text("✓ Mapping saved:")
            for sb, marker in self.p6_sb_marker_map.items():
                tooth_info = f" (Tooth #{self.p6_tooth_positions[marker]})" if marker in self.p6_tooth_positions else ""
                self.p6_log_text(f"  {sb} → Marker {marker}{tooth_info}")
            
            dialog.destroy()
        
        def auto_detect():
            # Auto-detect from file if loaded
            messagebox.showinfo("Auto-Detect", 
                              "Auto-detect will analyze IOS file and photogrammetry data to suggest mapping.\n\n"
                              "This feature requires both files to be loaded first.")
        
        ttk.Button(button_frame, text="Save Mapping", command=save_mapping, style='Accent.TButton').pack(side=tk.RIGHT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side=tk.RIGHT, padx=5)
        ttk.Button(button_frame, text="Auto-Detect", command=auto_detect).pack(side=tk.LEFT, padx=5)
    
    def p6_create_refpoints_u(self):
        """Create refpoints_U.json from structure_L.json."""
        # Ask for structure_L.json
        structure_file = filedialog.askopenfilename(
            title="Select structure_L.json",
            filetypes=[("JSON", "*.json"), ("All Files", "*.*")]
        )
        
        if not structure_file:
            return
        
        # Ask for output location
        output_file = filedialog.asksaveasfilename(
            title="Save refpoints_U.json",
            defaultextension=".json",
            initialfile="refpoints_U.json",
            filetypes=[("JSON", "*.json"), ("All Files", "*.*")]
        )
        
        if not output_file:
            return
        
        self.p6_log_text("="*70)
        self.p6_log_text("Creating refpoints_U.json from structure_L.json...")
        
        try:
            import json
            
            # Load structure_L.json
            with open(structure_file, 'r') as f:
                structure = json.load(f)
            
            if 'points_3d' not in structure:
                raise ValueError("structure_L.json missing 'points_3d' field")
            
            # Convert to refpoints format
            refpoints = {
                "frame": "U",
                "units": "mm",
                "points": {}
            }
            
            for track_id, point_data in structure['points_3d'].items():
                # structure_L.json uses 'xyz' field for 3D coordinates
                if 'xyz' in point_data:
                    refpoints["points"][track_id] = point_data['xyz']
                elif 'position_mm' in point_data:
                    refpoints["points"][track_id] = point_data['position_mm']
                else:
                    raise ValueError(f"Point {track_id} missing coordinate data ('xyz' or 'position_mm')")
            
            # Add metadata
            refpoints["metadata"] = {
                "timestamp": datetime.now().isoformat() + "Z",
                "source": "structure_L.json conversion",
                "n_points": len(refpoints["points"])
            }
            
            # Save
            with open(output_file, 'w') as f:
                json.dump(refpoints, f, indent=2)
            
            # Update GUI
            self.p6_refpoints.set(output_file)
            
            self.p6_log_text(f"✓ Success: Created {output_file}")
            self.p6_log_text(f"  Converted {len(refpoints['points'])} points")
            messagebox.showinfo("Success", f"Created refpoints_U.json with {len(refpoints['points'])} points")
            
        except Exception as e:
            self.p6_log_text(f"✗ Error: {e}")
            messagebox.showerror("Error", str(e))
    
    def p6_extract_implants(self):
        """Extract implants from refpoints_U.json."""
        if not self.p6_refpoints.get() or not self.p6_implants_u.get():
            messagebox.showerror("Error", "Please specify input and output files.")
            return
        
        self.p6_log_text("="*70)
        self.p6_log_text("Extracting implants from photogrammetry...")
        
        def run():
            try:
                import sys
                sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
                from implant_extractor import extract_implants_from_refpoints
                
                extract_implants_from_refpoints(
                    self.p6_refpoints.get(),
                    self.p6_implants_u.get(),
                    self.p6_tag_size.get()
                )
                
                self.p6_log_text(f"✓ Success: Saved to {self.p6_implants_u.get()}")
                messagebox.showinfo("Success", "Implants extracted successfully!")
            except Exception as e:
                self.p6_log_text(f"✗ Error: {e}")
                messagebox.showerror("Error", str(e))
        
        threading.Thread(target=run, daemon=True).start()
    
    def p6_convert_ios(self):
        """Convert IOS scan body data to implants_I.json."""
        if not self.p6_ios_file.get() or not self.p6_implants_i.get():
            messagebox.showerror("Error", "Please specify input and output files.")
            return

        ios_path = self.p6_ios_file.get().strip()
        ios_ext = Path(ios_path).suffix.lower()

        marker_ids_str = self.p6_marker_ids.get().strip()
        # Marker IDs are optional for CSVs that already include tag IDs.
        # For JSON scanbody transforms we still require marker IDs.
        if ios_ext != '.csv' and not marker_ids_str:
            messagebox.showerror(
                "Error",
                "Please specify marker IDs (comma-separated).\n\n"
                "Tip: If you are using a CSV with tag IDs as the first/last column, marker IDs can be left blank."
            )
            return
        
        self.p6_log_text("="*70)
        self.p6_log_text("Converting IOS scan body data...")
        
        def run():
            try:
                import subprocess

                marker_ids = [x.strip() for x in marker_ids_str.split(',') if x.strip()] if marker_ids_str else []

                tools_dir = Path(__file__).parent

                if ios_ext == '.csv':
                    script = tools_dir / "convert_ios_centers.py"
                    cmd = [
                        sys.executable,
                        "-X", "utf8",
                        str(script),
                        "--input", ios_path,
                        "--output", self.p6_implants_i.get(),
                    ]
                    if marker_ids:
                        cmd += ["--marker-ids", *marker_ids]
                else:
                    script = tools_dir / "convert_ios_scanbody.py"
                    cmd = [
                        sys.executable,
                        "-X", "utf8",
                        str(script),
                        "--input", ios_path,
                        "--output", self.p6_implants_i.get(),
                        "--marker-ids", *marker_ids,
                        "--include-identity"
                    ]

                result = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
                
                if result.returncode == 0:
                    self.p6_log_text(result.stdout)
                    self.p6_log_text(f"✓ Success: Saved to {self.p6_implants_i.get()}")
                    messagebox.showinfo("Success", "IOS data converted successfully!")
                else:
                    self.p6_log_text(result.stdout)
                    self.p6_log_text(result.stderr)
                    messagebox.showerror("Error", "Conversion failed. See log for details.")
            except Exception as e:
                self.p6_log_text(f"✗ Error: {e}")
                messagebox.showerror("Error", str(e))
        
        threading.Thread(target=run, daemon=True).start()
    
    def p6_compute_alignment(self):
        """Compute T_I_from_U alignment."""
        # Auto-fill common defaults to reduce friction
        try:
            iu = (self.p6_implants_u.get() or "").strip()
            ii = (self.p6_implants_i.get() or "").strip()
            tf = (self.p6_transform_file.get() or "").strip()

            # Infer implants_U.json from refpoints_U.json folder
            if not iu:
                rp = (self.p6_refpoints.get() or "").strip()
                if rp and Path(rp).exists():
                    cand = Path(rp).parent / "implants_U.json"
                    if cand.exists():
                        self.p6_implants_u.set(str(cand))
                        iu = str(cand)

            # Infer implants_I.json from IOS input folder
            if not ii:
                ios_in = (self.p6_ios_file.get() or "").strip()
                if ios_in and Path(ios_in).exists():
                    cand = Path(ios_in).parent / "implants_I.json"
                    if cand.exists():
                        self.p6_implants_i.set(str(cand))
                        ii = str(cand)

            # Infer output transform path from implants_I folder
            if not tf and ii:
                cand = Path(ii).parent / "T_I_from_U.json"
                self.p6_transform_file.set(str(cand))
                tf = str(cand)
        except Exception:
            # If inference fails, fall back to explicit validation below
            pass

        missing = []
        if not (self.p6_implants_u.get() or "").strip():
            missing.append("implants_U.json (U-frame)")
        if not (self.p6_implants_i.get() or "").strip():
            missing.append("implants_I.json (IOS I-frame)")
        if not (self.p6_transform_file.get() or "").strip():
            missing.append("output T_I_from_U.json")

        if missing:
            messagebox.showerror(
                "Error",
                "Missing required inputs:\n\n- " + "\n- ".join(missing) + "\n\n"
                "Tip: Run Step 1 + Step 2 first, or browse to existing JSON files."
            )
            return
        
        self.p6_log_text("="*70)
        self.p6_log_text("Computing alignment...")
        
        def run():
            try:
                import subprocess
                
                cmd = [
                    sys.executable,
                    "-X", "utf8",
                    str(Path(__file__).parent / "solve_T_I_from_U.py"),
                    "--implants-u", self.p6_implants_u.get(),
                    "--implants-i", self.p6_implants_i.get(),
                    "--output", self.p6_transform_file.get(),
                    "--rmse-threshold", str(self.p6_rmse_threshold.get()),
                    "--export-transformed"
                ]
                
                # Add scale flag if enabled
                if self.p6_allow_scale.get():
                    cmd.append("--allow-scaling")
                    self.p6_log_text("Scale estimation enabled (Sim3 transform)")
                
                result = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
                
                self.p6_log_text(result.stdout)
                
                if result.returncode == 0:
                    self.p6_log_text(f"✓ Success: Transform saved to {self.p6_transform_file.get()}")
                    messagebox.showinfo("Success", "Alignment computed successfully!")
                else:
                    self.p6_log_text("⚠ Warning: RMSE exceeded threshold. Check alignment quality.")
                    if self.p6_allow_scale.get():
                        messagebox.showwarning("Warning", "Alignment completed but RMSE exceeds threshold even with scale estimation. Review the log.")
                    else:
                        response = messagebox.askyesno("Scale Mismatch?", 
                                                      "RMSE exceeds threshold. This may indicate a scale mismatch between IOS and photogrammetry.\n\n"
                                                      "Enable 'Allow scale estimation' and try again?")
                        if response:
                            self.p6_allow_scale.set(True)
            except Exception as e:
                self.p6_log_text(f"✗ Error: {e}")
                messagebox.showerror("Error", str(e))
        
        threading.Thread(target=run, daemon=True).start()
    
    def p6_generate_constellations(self):
        """Generate both U and I constellation STLs."""
        if not self.p6_implants_u.get() or not self.p6_implants_i.get():
            messagebox.showerror("Error", "Please specify implants_U and implants_I files.")
            return
        
        if not self.p6_constellation_u.get() or not self.p6_constellation_i.get():
            messagebox.showerror("Error", "Please specify output STL files.")
            return
        
        self.p6_log_text("="*70)
        self.p6_log_text("Generating constellation STLs...")
        
        def run():
            try:
                import subprocess

                def _auto_detect_scanbody_stl() -> str | None:
                    """Try to locate a scanbody STL in the same case directory as inputs/outputs."""
                    candidates = []
                    for raw in [
                        self.p6_ios_file.get(),
                        self.p6_implants_i.get(),
                        self.p6_implants_u.get(),
                        self.p6_transform_file.get(),
                        self.p6_constellation_i.get(),
                        self.p6_constellation_u.get(),
                    ]:
                        if not raw:
                            continue
                        try:
                            p = Path(raw).expanduser()
                            d = p if p.is_dir() else p.parent
                            if d and d.exists():
                                candidates.append(d)
                        except Exception:
                            continue

                    seen = set()
                    unique_dirs = []
                    for d in candidates:
                        key = str(d).lower()
                        if key in seen:
                            continue
                        seen.add(key)
                        unique_dirs.append(d)

                    preferred_names = [
                        "scanbody.stl",
                        "scan_body.stl",
                        "scanbody_template.stl",
                        "scanbody-library.stl",
                        "scanbody_library.stl",
                    ]

                    for d in unique_dirs:
                        for name in preferred_names:
                            p = d / name
                            if p.exists():
                                return str(p)
                        for p in sorted(d.glob("*scanbody*.stl")):
                            if p.exists():
                                return str(p)
                    return None

                scanbody_stl_path = self.p6_scanbody_stl.get().strip()
                if self.p6_use_scanbody_stl.get():
                    if scanbody_stl_path:
                        try:
                            sb = Path(scanbody_stl_path).expanduser()
                            if not sb.is_absolute():
                                sb = (Path.cwd() / sb).resolve()
                            scanbody_stl_path = str(sb)
                        except Exception:
                            pass

                    if not scanbody_stl_path or not Path(scanbody_stl_path).exists():
                        detected = _auto_detect_scanbody_stl()
                        if detected:
                            scanbody_stl_path = detected
                            self.p6_scanbody_stl.set(detected)
                            self.p6_log_text(f"Auto-detected scanbody STL: {detected}")
                        else:
                            raise Exception(
                                "Scanbody STL mode enabled but no scanbody STL found. "
                                "Place 'scanbody.stl' in your case directory (same folder as implants/IOS files) "
                                "or click Browse to select it."
                            )
                
                # Generate U constellation
                self.p6_log_text("Generating U-frame constellation...")
                cmd_u = [
                    sys.executable,
                    "-X", "utf8",
                    str(Path(__file__).parent / "generate_constellation.py"),
                    "--implants", self.p6_implants_u.get(),
                    "--output", self.p6_constellation_u.get()
                ]

                if self.p6_use_scanbody_stl.get() and scanbody_stl_path:
                    axis = (self.p6_scanbody_axis.get() or "").strip() or "z"
                    recenter = (self.p6_scanbody_recenter.get() or "").strip() or "none"
                    cmd_u += [
                        "--scanbody-stl", scanbody_stl_path,
                        "--scanbody-scale", str(self.p6_scanbody_scale.get()),
                        "--scanbody-recenter", recenter,
                    ]

                    # If axis begins with '-', argparse can misinterpret it as another flag.
                    if axis.startswith("-"):
                        cmd_u.append(f"--scanbody-axis={axis}")
                    else:
                        cmd_u += ["--scanbody-axis", axis]
                
                result_u = subprocess.run(cmd_u, capture_output=True, text=True, encoding='utf-8', errors='replace')
                self.p6_log_text(result_u.stdout)
                
                if result_u.returncode != 0:
                    raise Exception(f"U constellation failed: {result_u.stderr}")
                
                # Generate I constellation
                self.p6_log_text("Generating I-frame constellation...")
                cmd_i = [
                    sys.executable,
                    "-X", "utf8",
                    str(Path(__file__).parent / "generate_constellation.py"),
                    "--implants", self.p6_implants_i.get(),
                    "--output", self.p6_constellation_i.get()
                ]

                if self.p6_use_scanbody_stl.get() and scanbody_stl_path:
                    axis = (self.p6_scanbody_axis.get() or "").strip() or "z"
                    recenter = (self.p6_scanbody_recenter.get() or "").strip() or "none"
                    cmd_i += [
                        "--scanbody-stl", scanbody_stl_path,
                        "--scanbody-scale", str(self.p6_scanbody_scale.get()),
                        "--scanbody-recenter", recenter,
                    ]

                    if axis.startswith("-"):
                        cmd_i.append(f"--scanbody-axis={axis}")
                    else:
                        cmd_i += ["--scanbody-axis", axis]
                
                result_i = subprocess.run(cmd_i, capture_output=True, text=True, encoding='utf-8', errors='replace')
                self.p6_log_text(result_i.stdout)
                
                if result_i.returncode != 0:
                    raise Exception(f"I constellation failed: {result_i.stderr}")
                
                self.p6_log_text("✓ Success: Both constellations generated!")
                messagebox.showinfo("Success", "Constellation STLs generated successfully!")
            except Exception as e:
                self.p6_log_text(f"✗ Error: {e}")
                messagebox.showerror("Error", str(e))
        
        threading.Thread(target=run, daemon=True).start()
    
    def p6_export_package(self):
        """Export complete IOS package."""
        if not self.p6_package_dir.get():
            messagebox.showerror("Error", "Please specify output directory.")
            return
        
        # Check if transform file exists
        if not self.p6_transform_file.get() or not Path(self.p6_transform_file.get()).exists():
            messagebox.showerror("Error", "Please compute alignment first (Step 3).")
            return
        
        self.p6_log_text("="*70)
        self.p6_log_text("Exporting IOS package...")
        
        def run():
            try:
                import subprocess
                
                # Determine run directory from transform file
                run_dir = Path(self.p6_transform_file.get()).parent
                
                cmd = [
                    sys.executable,
                    str(Path(__file__).parent / "export_ios_package.py"),
                    "--run-dir", str(run_dir),
                    "--output", self.p6_package_dir.get(),
                    "--case-name", self.p6_case_name.get()
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True)
                
                self.p6_log_text(result.stdout)
                
                if result.returncode == 0:
                    self.p6_log_text(f"✓ Success: Package exported to {self.p6_package_dir.get()}")
                    messagebox.showinfo("Success", f"IOS package exported successfully!\n\nLocation: {self.p6_package_dir.get()}")
                else:
                    self.p6_log_text(result.stderr)
                    messagebox.showerror("Error", "Package export failed. See log for details.")
            except Exception as e:
                self.p6_log_text(f"✗ Error: {e}")
                messagebox.showerror("Error", str(e))
        
        threading.Thread(target=run, daemon=True).start()


class LogRedirector:
    """Redirect stdout to GUI log."""
    def __init__(self, log_func):
        self.log_func = log_func
        self.buffer = ""
    
    def write(self, text):
        if text and text != "\n":
            self.log_func(text.rstrip())
    
    def flush(self):
        pass


def main():
    root = tk.Tk()
    app = CalibrationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
