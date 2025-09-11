#!/usr/bin/env python3
"""
Extract FPGA statistics from Quartus reports
"""
import sys
import re
import os

def extract_stats(stats_file):
    """Extract key FPGA statistics with proper parsing"""
    try:
        with open(stats_file, 'r') as f:
            content = f.read()
        
        # Define regex patterns with specific capture groups for used/total/percent
        # Updated to handle Quartus report format with semicolons
        patterns = {
            'LOGIC_ALMS': r'Logic utilization[^;]*;\s*([0-9,]+)\s*/\s*([0-9,]+)\s*\(\s*(\d+)\s*%\s*\)',
            'MEMORY_BITS': r'Total block memory bits[^;]*;\s*([0-9,]+)\s*/\s*([0-9,]+)\s*\(\s*(\d+)\s*%\s*\)',
            'DSP_BLOCKS': r'Total DSP Blocks[^;]*;\s*([0-9,]+)\s*/\s*([0-9,]+)\s*\(\s*(\d+)\s*%\s*\)',
            'PLLS': r'Total PLLs[^;]*;\s*([0-9,]+)\s*/\s*([0-9,]+)\s*\(\s*(\d+)\s*%\s*\)'
        }
        
        stats = {}
        for key, pattern in patterns.items():
            match = re.search(pattern, content)
            if match:
                used, total, percent = match.groups()
                stats[key] = f"{used} / {total} ({percent}%)"
            else:
                stats[key] = "N/A"
        
        return stats
        
    except Exception as e:
        print(f"Error extracting stats: {e}", file=sys.stderr)
        return {
            'LOGIC_ALMS': 'N/A',
            'MEMORY_BITS': 'N/A', 
            'DSP_BLOCKS': 'N/A',
            'PLLS': 'N/A'
        }

def main():
    stats_file = sys.argv[1] if len(sys.argv) > 1 else 'src/fpga/fpga_stats.txt'
    
    if not os.path.exists(stats_file):
        print(f"Stats file not found: {stats_file}", file=sys.stderr)
        sys.exit(1)
    
    stats = extract_stats(stats_file)
    
    # Output as shell variables (properly quoted for eval)
    for key, value in stats.items():
        print(f'{key}="{value}"')

if __name__ == '__main__':
    main()