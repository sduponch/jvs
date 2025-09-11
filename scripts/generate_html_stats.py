#!/usr/bin/env python3
"""
Generate HTML page with FPGA statistics using proper templating
"""
import os
import sys
import re
import subprocess
from pathlib import Path
from datetime import datetime

def extract_fpga_stats(stats_file):
    """Extract key FPGA statistics from Quartus reports"""
    try:
        with open(stats_file, 'r') as f:
            content = f.read()
        
        # Define regex patterns for resource utilization
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
        
        return stats, content
        
    except Exception as e:
        print(f"Error extracting stats: {e}", file=sys.stderr)
        return {
            'LOGIC_ALMS': 'N/A',
            'MEMORY_BITS': 'N/A', 
            'DSP_BLOCKS': 'N/A',
            'PLLS': 'N/A'
        }, ""

def get_file_list(package_dir):
    """Get list of files in package directory"""
    files = []
    if os.path.exists(package_dir):
        for root, dirs, filenames in os.walk(package_dir):
            for filename in filenames:
                rel_path = os.path.relpath(os.path.join(root, filename), package_dir)
                files.append(rel_path)
    return sorted(files)

def generate_html(build_info, stats, detailed_stats, file_list, output_file):
    """Generate HTML using external template file"""
    
    # Create stats cards HTML
    stats_cards = f"""<div class="stat-card"><div class="stat-value">ALMs</div><div class="stat-label">{stats['LOGIC_ALMS']}</div></div>
    <div class="stat-card"><div class="stat-value">RAM</div><div class="stat-label">{stats['MEMORY_BITS']}</div></div>
    <div class="stat-card"><div class="stat-value">DSP</div><div class="stat-label">{stats['DSP_BLOCKS']}</div></div>
    <div class="stat-card"><div class="stat-value">PLL</div><div class="stat-label">{stats['PLLS']}</div></div>"""
    
    # Create file list HTML
    file_list_html = "<ul>"
    for file in file_list:
        file_list_html += f"<li>{file}</li>"
    file_list_html += "</ul>"
    
    # Escape detailed stats for HTML
    detailed_stats_escaped = detailed_stats.replace('<', '&lt;').replace('>', '&gt;')
    
    # Load template file
    template_path = Path(__file__).parent / 'template.html'
    try:
        with open(template_path, 'r') as f:
            template_content = f.read()
    except FileNotFoundError:
        print(f"Template file not found: {template_path}", file=sys.stderr)
        return
    
    # Replace placeholders
    html_content = template_content.replace('{{BUILD_NUMBER}}', build_info['build_number'])
    html_content = html_content.replace('{{COMMIT_SHA}}', build_info['commit_sha'])
    html_content = html_content.replace('{{BUILD_DATE}}', build_info['build_date'])
    html_content = html_content.replace('{{BRANCH_NAME}}', build_info['branch_name'])
    html_content = html_content.replace('{{STATS_CARDS}}', stats_cards)
    html_content = html_content.replace('{{DETAILED_STATS}}', detailed_stats_escaped)
    html_content = html_content.replace('{{FILE_LIST}}', file_list_html)

    # Write HTML file
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        f.write(html_content)

def main():
    """Main function"""
    # Get build info from environment variables
    build_info = {
        'build_number': os.getenv('BUILD_NUMBER', 'Unknown'),
        'commit_sha': os.getenv('COMMIT_SHA', 'Unknown'),
        'build_date': os.getenv('BUILD_DATE', datetime.now().strftime('%Y-%m-%d %H:%M:%S UTC')),
        'branch_name': os.getenv('BRANCH_NAME', 'Unknown')
    }
    
    # Paths
    stats_file = 'src/fpga/fpga_stats.txt'
    output_file = 'package/index.html'
    package_dir = 'package'
    
    print("🌐 Generating HTML page with Python...")
    
    # Extract FPGA stats
    if os.path.exists(stats_file):
        print("📊 Processing FPGA statistics...")
        stats, detailed_stats = extract_fpga_stats(stats_file)
    else:
        print(f"⚠️ Stats file not found: {stats_file}")
        stats = {'LOGIC_ALMS': 'N/A', 'MEMORY_BITS': 'N/A', 'DSP_BLOCKS': 'N/A', 'PLLS': 'N/A'}
        detailed_stats = "No statistics available"
    
    # Get file list
    file_list = get_file_list(package_dir)
    
    # Generate HTML
    generate_html(build_info, stats, detailed_stats, file_list, output_file)
    
    print(f"✅ HTML generated: {output_file}")
    print(f"📊 FPGA Stats: ALMs={stats['LOGIC_ALMS']}, RAM={stats['MEMORY_BITS']}, DSP={stats['DSP_BLOCKS']}, PLL={stats['PLLS']}")

if __name__ == '__main__':
    main()