#!/usr/bin/env python3
"""
Update markdown documents to replace Mermaid blocks with image references
"""
import re
import os
from pathlib import Path

def replace_mermaid_with_images(markdown_file, image_dir):
    """Replace mermaid code blocks with image references"""
    with open(markdown_file, 'r') as f:
        content = f.read()
    
    # Get base name for image directory
    base_name = Path(markdown_file).stem
    file_image_dir = os.path.join(image_dir, base_name)
    
    # Find all mermaid blocks
    pattern = r'```mermaid\n(.*?)\n```'
    matches = list(re.finditer(pattern, content, re.DOTALL))
    
    if not matches:
        print(f"No Mermaid blocks found in {markdown_file}")
        return False
    
    # Replace from end to beginning to maintain positions
    modified_content = content
    for i, match in enumerate(reversed(matches)):
        diagram_num = len(matches) - i
        
        # Check if images exist
        png_path = os.path.join(file_image_dir, f'diagram_{diagram_num}.png')
        svg_path = os.path.join(file_image_dir, f'diagram_{diagram_num}.svg')
        
        if os.path.exists(png_path):
            # Create relative path from docs directory
            rel_png_path = os.path.relpath(png_path, os.path.dirname(markdown_file))
            
            # Replace mermaid block with image
            replacement = f'![Diagram {diagram_num}]({rel_png_path})'
            
            # Add note about original being Mermaid
            replacement += f'\n<!-- Original diagram was Mermaid format - see {rel_png_path.replace(".png", ".mmd")} -->'
            
            modified_content = modified_content[:match.start()] + replacement + modified_content[match.end():]
            print(f"  ✓ Replaced diagram {diagram_num} with image")
        else:
            print(f"  ✗ Image not found for diagram {diagram_num}")
    
    # Save modified content
    output_file = markdown_file.replace('.md', '_with_images.md')
    with open(output_file, 'w') as f:
        f.write(modified_content)
    
    print(f"  → Saved to: {output_file}")
    return True

def main():
    """Main function"""
    docs_dir = '/home/ee/sensing_fw/docs'
    image_dir = '/home/ee/sensing_fw/docs/mermaid_images'
    
    # Documents to update
    markdown_files = [
        'Bluetooth_GATT_Specification.md',
        'Sensor_Logging_Specification.md',
        'D2D_Communication_Complete_Guide.md',
        'FOTA_Complete_Guide.md'
    ]
    
    print("Updating documents with images...")
    print("=" * 50)
    
    for md_file in markdown_files:
        full_path = os.path.join(docs_dir, md_file)
        if os.path.exists(full_path):
            print(f"\nProcessing: {md_file}")
            replace_mermaid_with_images(full_path, image_dir)
    
    print("\n" + "=" * 50)
    print("Update complete!")
    print("\nTo use the updated documents:")
    print("1. Review the *_with_images.md files")
    print("2. If satisfied, rename them to replace the originals")
    print("3. The images are in the mermaid_images/ directory")

if __name__ == '__main__':
    main()