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
        self.recon_phase4 = tk.BooleanVar(value=False)
        self.recon_phase4_method = tk.StringVar(value="reference_plate")
        self.recon_ref_plate = tk.StringVar()
        
        # Quality Gate Variables
        self.qg_reconstruction_dir = tk.StringVar()
        self.qg_cutoff_percentile = tk.DoubleVar(value=10.0)
        self.qg_criterion = tk.StringVar(value="mean")
        self.qg_sfm = None
        self.qg_image_errors = []  # List of (image_name, mean, max, median) tuples
        self.qg_manual_status = {}  # image_id -> bool (True=REMOVE, False=KEEP)
        self.qg_loaded_metadata = None
        self.qg_loaded_structure_path = None
        
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
        
        self.calib_tab = calib_tab
        self.validation_tab = validation_tab
        self.reconstruction_tab = reconstruction_tab
        self.quality_gate_tab = quality_gate_tab

        self.notebook.add(calib_tab, text="Calibration")
        self.notebook.add(validation_tab, text="Validation (PnP + BA)")
        self.notebook.add(reconstruction_tab, text="Phase 3/4 Reconstruction")
        self.notebook.add(quality_gate_tab, text="Quality Gate")
        
        # Create calibration interface
        self.create_calibration_tab(calib_tab)
        
        # Create validation interface
        self.create_validation_tab(validation_tab)
        
        # Create reconstruction interface
        self.create_reconstruction_tab(reconstruction_tab)
        
        # Create quality gate interface
        self.create_quality_gate_tab(quality_gate_tab)
    
    def create_calibration_tab(self, parent):
        # Main container with padding
        main_frame = ttk.Frame(parent)
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
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
        
        ttk.Entry(main_frame, textvariable=self.image_folder, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_folder).grid(row=row, column=2)
        row += 1
        
        # Output File Section
        ttk.Label(main_frame, text="Output File:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.output_file, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_output).grid(row=row, column=2)
        row += 1
        
        # Board Configuration Section
        board_frame = ttk.LabelFrame(main_frame, text="ChArUco Board Configuration", padding="10")
        board_frame.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(15, 5))
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
        dict_combo.grid(row=board_row, column=1, columnspan=3, sticky=(tk.W, tk.E))
        
        # Calibration Parameters Section
        calib_frame = ttk.LabelFrame(main_frame, text="Calibration Parameters", padding="10")
        calib_frame.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
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
        self.progress.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        row += 1
        
        # Log output
        ttk.Label(main_frame, text="Calibration Log:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W)
        row += 1
        
        self.log_text = scrolledtext.ScrolledText(main_frame, height=15, width=90, wrap=tk.WORD, state=tk.DISABLED)
        self.log_text.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
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
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
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
        
        ttk.Entry(main_frame, textvariable=self.test_folder, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_test_folder).grid(row=row, column=2)
        row += 1
        
        # Calibration File
        ttk.Label(main_frame, text="Calibration File (JSON):", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.calib_file, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_calib_file).grid(row=row, column=2)
        row += 1
        
        # Layout File
        ttk.Label(main_frame, text="Tag Layout File (JSON):", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.layout_file, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_layout_file).grid(row=row, column=2)
        row += 1
        
        # Parameters Frame
        param_frame = ttk.LabelFrame(main_frame, text="Parameters", padding="10")
        param_frame.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(15, 5))
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
        ttk.Entry(param_frame, textvariable=self.validation_output, width=40).grid(row=param_row, column=1, sticky=(tk.W, tk.E), pady=5)
        
        # Control Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=row, column=0, columnspan=3, pady=(15, 10))
        row += 1
        
        self.val_run_button = ttk.Button(button_frame, text="Run Validation", command=self.run_validation, width=20)
        self.val_run_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(button_frame, text="Clear Log", command=self.clear_log, width=15).pack(side=tk.LEFT, padx=5)
        
        # Progress bar
        self.val_progress = ttk.Progressbar(main_frame, mode='indeterminate')
        self.val_progress.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        row += 1
        
        # Log output
        ttk.Label(main_frame, text="Validation Log:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W)
        row += 1
        
        self.val_log_text = scrolledtext.ScrolledText(main_frame, height=15, width=90, wrap=tk.WORD, state=tk.DISABLED)
        self.val_log_text.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
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
        main_frame = ttk.Frame(parent)
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        row = 0
        
        # Title
        title = ttk.Label(main_frame, text="Phase 3 & 4: Multi-View Reconstruction", font=('Arial', 16, 'bold'))
        title.grid(row=row, column=0, columnspan=3, pady=(0, 20))
        row += 1
        
        # Input Images Section
        ttk.Label(main_frame, text="Input Images:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=5)
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.recon_images, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_recon_images).grid(row=row, column=2)
        row += 1
        ttk.Label(main_frame, text="Glob pattern (e.g., data/*.TIF)", font=('Arial', 8)).grid(row=row, column=0, columnspan=2, sticky=tk.W)
        row += 1
        
        # Calibration File Section
        ttk.Label(main_frame, text="Calibration File:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.recon_calib, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_recon_calib).grid(row=row, column=2)
        row += 1
        
        # Output Directory Section
        ttk.Label(main_frame, text="Output Directory:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.recon_output, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_recon_output).grid(row=row, column=2)
        row += 1
        
        # Optional: Layout File
        ttk.Label(main_frame, text="Layout File (Optional):", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=(15, 5))
        row += 1
        
        ttk.Entry(main_frame, textvariable=self.recon_layout, width=60).grid(row=row, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=(0, 5))
        ttk.Button(main_frame, text="Browse...", command=self.browse_recon_layout).grid(row=row, column=2)
        row += 1
        ttk.Label(main_frame, text="For validation with known tag positions", font=('Arial', 8)).grid(row=row, column=0, columnspan=2, sticky=tk.W)
        row += 1
        
        # Parameters Frame
        params_frame = ttk.LabelFrame(main_frame, text="Parameters", padding="10")
        params_frame.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(15, 10))
        row += 1
        
        # Tag Size
        ttk.Label(params_frame, text="AprilTag Edge Length (mm):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(params_frame, textvariable=self.recon_tag_size, width=15).grid(row=0, column=1, sticky=tk.W, padx=5)
        
        # Phase 4 Options Frame
        phase4_frame = ttk.LabelFrame(main_frame, text="Phase 4: User Frame Transform", padding="10")
        phase4_frame.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(10, 10))
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
        self.ref_plate_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), padx=5)
        self.ref_plate_button = ttk.Button(phase4_frame, text="Browse...", command=self.browse_ref_plate, state=tk.DISABLED)
        self.ref_plate_button.grid(row=2, column=2, padx=5)
        
        # Progress and Log
        progress_frame = ttk.Frame(main_frame)
        progress_frame.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(10, 5))
        row += 1
        
        self.recon_progress = ttk.Progressbar(progress_frame, mode='indeterminate')
        self.recon_progress.pack(fill=tk.X, pady=5)
        
        # Log Output
        log_frame = ttk.LabelFrame(main_frame, text="Reconstruction Log", padding="5")
        log_frame.grid(row=row, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(5, 10))
        main_frame.rowconfigure(row, weight=1)
        row += 1
        
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
        
        # Clear log
        self.recon_log.config(state=tk.NORMAL)
        self.recon_log.delete(1.0, tk.END)
        self.recon_log.config(state=tk.DISABLED)
        
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
            import phase3_test_pipeline
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
                
                # Run Phase 3
                sfm, phase3_metadata = phase3_test_pipeline.run_phase3_pipeline(
                    image_paths=image_paths,
                    calib_file=self.recon_calib.get(),
                    output_dir=self.recon_output.get(),
                    layout_file=self.recon_layout.get() if self.recon_layout.get() else None,
                    tag_size_mm=self.recon_tag_size.get(),
                    verbose=True
                )

                # Always allow outlier analysis + re-opt even if QA failed.
                if not phase3_metadata.get("qa_passed", False):
                    self.recon_log_text("")
                    self.recon_log_text("⚠ Phase 3 QA did not pass. Use the Quality Gate tab to remove outlier images and re-run BA + QA.")

                # Kick the user into Quality Gate with this output directory.
                self.root.after(0, lambda: self.open_quality_gate(self.recon_output.get()))
                
                # Run Phase 4 if enabled
                if self.recon_phase4.get():
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
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
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
        load_frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        load_frame.columnconfigure(1, weight=1)
        row += 1
        
        ttk.Label(load_frame, text="Reconstruction Directory:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(load_frame, textvariable=self.qg_reconstruction_dir, width=60).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        ttk.Button(load_frame, text="Browse...", command=self.qg_browse_reconstruction).grid(row=0, column=2, padx=5)
        ttk.Button(load_frame, text="Load & Analyze", command=self.qg_load_reconstruction, style='Accent.TButton').grid(row=0, column=3, padx=5)
        
        # Filter Controls
        control_frame = ttk.LabelFrame(main_frame, text="Filter Settings", padding="10")
        control_frame.grid(row=row, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
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
        slider_frame.grid(row=2, column=0, columnspan=4, sticky=(tk.W, tk.E), pady=5)
        slider_frame.columnconfigure(0, weight=1)
        
        self.qg_slider = ttk.Scale(slider_frame, from_=0, to=50, orient=tk.HORIZONTAL, 
                                   variable=self.qg_cutoff_percentile, command=self.qg_slider_changed)
        self.qg_slider.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=5)
        
        ttk.Label(slider_frame, text="0%").grid(row=1, column=0, sticky=tk.W, padx=5)
        ttk.Label(slider_frame, text="50%").grid(row=1, column=0, sticky=tk.E, padx=5)
        
        # Statistics display
        stats_frame = ttk.Frame(control_frame)
        stats_frame.grid(row=3, column=0, columnspan=4, sticky=(tk.W, tk.E), pady=10)
        
        self.qg_stats_label = ttk.Label(stats_frame, text="Load reconstruction to see statistics", 
                                        font=('Arial', 9), foreground='gray')
        self.qg_stats_label.pack()
        
        # Image Error Table
        table_frame = ttk.LabelFrame(main_frame, text="Per-Image Errors (sorted worst to best)", padding="5")
        table_frame.grid(row=row, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
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
    
    def finish_reconstruction(self, success):
        """Finish reconstruction and re-enable controls."""
        self.is_running = False
        self.recon_progress.stop()
        self.recon_run_button.config(state=tk.NORMAL)
        
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
