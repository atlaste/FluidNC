#!/usr/bin/env python3
"""
PlatformIO post-build script to generate symbol map
Add to platformio.ini: extra_scripts = post:FluidNC/generate-symbols-pio.py
"""

Import("env")
import os
import sys

# Add the FluidNC directory to path so we can import the generator
sys.path.insert(0, os.path.join(env.get("PROJECT_DIR"), "FluidNC"))

try:
    from generate_symbols import extract_symbols
    
    def generate_symbols_callback(*args, **kwargs):
        """Called after ELF is built"""
        # Get the ELF file path
        elf_path = env.subst("$BUILD_DIR/${PROGNAME}.elf")
        output_path = env.subst("$BUILD_DIR/symbols.txt.gz")
        
        print("=" * 60)
        print("Generating symbol map for profiling...")
        print("=" * 60)
        
        result = extract_symbols(
            elf_path,
            output_path,
            compress=True,
            filter_text_only=True,
            filter_project_only=True
        )
        
        if result == 0:
            print(f"Symbol map generated: {output_path}")
            
            # Also copy to data directory if needed
            data_dir = os.path.join(env.get("PROJECT_DIR"), "FluidNC", "data")
            if os.path.exists(data_dir):
                import shutil
                data_symbols = os.path.join(data_dir, "symbols.txt.gz")
                shutil.copy(output_path, data_symbols)
                print(f"Symbol map copied to data directory")
        else:
            print("Warning: Failed to generate symbol map")
        
        print("=" * 60)
    
    # Register the callback
    env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", generate_symbols_callback)
    
except ImportError as e:
    print(f"Warning: Could not import symbol generator: {e}")
    print("Symbols will not be generated")

