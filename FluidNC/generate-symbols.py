#!/usr/bin/env python3
"""
Generate compressed symbol map from FluidNC ELF file
Filters to .text section only and excludes library symbols
"""

import subprocess
import sys
import gzip
import os
import re
from pathlib import Path

def get_toolchain_prefix(elf_path):
    """Detect the right toolchain based on ELF architecture"""
    # Try to determine from file command
    try:
        result = subprocess.run(['file', elf_path], capture_output=True, text=True)
        if 'xtensa' in result.stdout.lower():
            return 'xtensa-esp32s3-elf-' if 's3' in elf_path.lower() else 'xtensa-esp32-elf-'
        elif 'riscv' in result.stdout.lower():
            return 'riscv32-esp-elf-'
    except:
        pass
    
    # Default to esp32
    return 'xtensa-esp32-elf-'

def extract_symbols(elf_path, output_path=None, compress=True, filter_text_only=True, filter_project_only=True):
    """
    Extract symbols from ELF file
    
    Args:
        elf_path: Path to ELF file
        output_path: Output file path (default: symbols.txt or symbols.txt.gz)
        compress: Whether to gzip compress the output
        filter_text_only: Only include .text section symbols (code, not data)
        filter_project_only: Exclude ESP-IDF and library symbols
    """
    
    elf_path = Path(elf_path)
    if not elf_path.exists():
        print(f"Error: ELF file not found: {elf_path}")
        return 1
    
    # Determine toolchain
    toolchain_prefix = get_toolchain_prefix(str(elf_path))
    nm_tool = f"{toolchain_prefix}nm"
    
    # Check if nm tool exists
    try:
        subprocess.run([nm_tool, '--version'], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"Error: {nm_tool} not found in PATH")
        print("Make sure ESP-IDF toolchain is installed and in PATH")
        return 1
    
    print(f"Extracting symbols from {elf_path.name}...")
    
    # Run nm with options:
    # --numeric-sort: Sort by address
    # --print-size: Include symbol size
    # --defined-only: Only defined symbols (not external references)
    # -l: Include source file/line info if available
    cmd = [nm_tool, '--numeric-sort', '--print-size', '--defined-only', str(elf_path)]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running {nm_tool}: {e}")
        return 1
    
    symbols = []
    total_count = 0
    filtered_count = 0
    
    # Parse nm output
    # Format: address size type name
    # Example: 400d1234 00000010 T stepper_pulse_func
    for line in result.stdout.splitlines():
        total_count += 1
        parts = line.split()
        if len(parts) < 3:
            continue
        
        address = parts[0]
        size = parts[1] if len(parts) >= 4 else '0'
        symbol_type = parts[2] if len(parts) >= 3 else 'U'
        name = parts[3] if len(parts) >= 4 else parts[2]
        
        # Filter by section type (text section = code)
        if filter_text_only:
            # T = text section (code), t = local text
            if symbol_type not in ['T', 't', 'W', 'w']:  # W/w = weak symbols
                continue
        
        # Filter out ESP-IDF and library symbols
        if filter_project_only:
            # Skip common ESP-IDF prefixes
            if any(name.startswith(prefix) for prefix in [
                'esp_', 'rom_', 'ets_', 'spi_flash_', 'vTaskDelay',
                'xQueue', 'xTask', 'vQueue', 'pvPortMalloc',
                '__', '_Z',  # Compiler internals and mangled C++
                'std::', 'Arduino::'  # Standard library
            ]):
                continue
            
            # Skip if it looks like a library symbol (all lowercase with underscores)
            if re.match(r'^[a-z_]+$', name) and len(name) > 15:
                continue
        
        symbols.append(f"{address} {size} {symbol_type} {name}")
        filtered_count += 1
    
    print(f"Total symbols: {total_count}")
    print(f"Filtered symbols: {filtered_count}")
    
    # Determine output path
    if output_path is None:
        output_path = elf_path.with_suffix('.symbols.txt')
        if compress:
            output_path = Path(str(output_path) + '.gz')
    else:
        output_path = Path(output_path)
    
    # Write output
    content = '\n'.join(symbols) + '\n'
    
    if compress:
        print(f"Writing compressed symbols to {output_path}...")
        with gzip.open(output_path, 'wt', encoding='utf-8') as f:
            f.write(content)
    else:
        print(f"Writing symbols to {output_path}...")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
    
    # Print statistics
    uncompressed_size = len(content)
    compressed_size = os.path.getsize(output_path)
    
    print(f"Uncompressed size: {uncompressed_size:,} bytes ({uncompressed_size/1024:.1f} KB)")
    print(f"Output size: {compressed_size:,} bytes ({compressed_size/1024:.1f} KB)")
    if compress:
        ratio = (1 - compressed_size/uncompressed_size) * 100
        print(f"Compression ratio: {ratio:.1f}%")
    
    return 0

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Extract symbols from FluidNC ELF file')
    parser.add_argument('elf_file', help='Path to ELF file')
    parser.add_argument('-o', '--output', help='Output file path')
    parser.add_argument('--no-compress', action='store_true', help='Disable gzip compression')
    parser.add_argument('--all-sections', action='store_true', help='Include all sections (not just .text)')
    parser.add_argument('--all-symbols', action='store_true', help='Include library and ESP-IDF symbols')
    
    args = parser.parse_args()
    
    sys.exit(extract_symbols(
        args.elf_file,
        args.output,
        compress=not args.no_compress,
        filter_text_only=not args.all_sections,
        filter_project_only=not args.all_symbols
    ))

