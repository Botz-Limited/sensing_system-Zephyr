#!/usr/bin/env python3
"""
Extract all Mermaid diagrams from markdown files and convert them to images
"""
import re
import os
import subprocess
from pathlib import Path

def extract_mermaid_diagrams(markdown_file):
    """Extract all mermaid code blocks from a markdown file"""
    with open(markdown_file, 'r') as f:
        content = f.read()
    
    # Find all mermaid blocks
    pattern = r'```mermaid\n(.*?)\n```'
    diagrams = re.findall(pattern, content, re.DOTALL)
    
    return diagrams

def save_diagram(diagram_content, output_path):
    """Save diagram content to a .mmd file"""
    with open(output_path, 'w') as f:
        f.write(diagram_content)

def convert_to_image(mmd_file, output_format='png'):
    """Convert .mmd file to image using mermaid-cli"""
    output_file = mmd_file.replace('.mmd', f'.{output_format}')
    cmd = ['mmdc', '-i', mmd_file, '-o', output_file, '-t', 'default', '-b', 'white', '-p', '/home/ee/sensing_fw/docs/puppeteer-config.json']
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ Converted: {output_file}")
            return output_file
        else:
            print(f"✗ Error converting {mmd_file}: {result.stderr}")
            return None
    except Exception as e:
        print(f"✗ Error: {e}")
        return None

def process_markdown_file(md_file, output_dir):
    """Process a single markdown file"""
    print(f"\nProcessing: {md_file}")
    
    # Create output directory for this file
    base_name = Path(md_file).stem
    file_output_dir = os.path.join(output_dir, base_name)
    os.makedirs(file_output_dir, exist_ok=True)
    
    # Extract diagrams
    diagrams = extract_mermaid_diagrams(md_file)
    
    if not diagrams:
        print(f"  No Mermaid diagrams found in {md_file}")
        return []
    
    print(f"  Found {len(diagrams)} Mermaid diagram(s)")
    
    converted_files = []
    
    for i, diagram in enumerate(diagrams):
        # Save as .mmd file
        mmd_file = os.path.join(file_output_dir, f'diagram_{i+1}.mmd')
        save_diagram(diagram, mmd_file)
        
        # Convert to PNG
        png_file = convert_to_image(mmd_file)
        if png_file:
            converted_files.append(png_file)
        
        # Also create SVG for better quality
        svg_file = convert_to_image(mmd_file, 'svg')
        if svg_file:
            converted_files.append(svg_file)
    
    return converted_files

def main():
    """Main function"""
    # Create output directory
    output_dir = '/home/ee/sensing_fw/docs/mermaid_images'
    os.makedirs(output_dir, exist_ok=True)
    
    # Find all markdown files with Mermaid diagrams
    docs_dir = '/home/ee/sensing_fw/docs'
    markdown_files = [
        'Bluetooth_GATT_Specification.md',
        'Sensor_Logging_Specification.md',
        'D2D_Communication_Complete_Guide.md',
        'FOTA_Complete_Guide.md'
    ]
    
    all_converted = []
    
    for md_file in markdown_files:
        full_path = os.path.join(docs_dir, md_file)
        if os.path.exists(full_path):
            converted = process_markdown_file(full_path, output_dir)
            all_converted.extend(converted)
    
    print(f"\n{'='*50}")
    print(f"Total diagrams converted: {len(all_converted)//2}")  # Divide by 2 because we create both PNG and SVG
    print(f"Output directory: {output_dir}")
    print(f"{'='*50}")

if __name__ == '__main__':
    main()