#!/usr/bin/env python3
"""
Create Coda-optimized documentation with responsive images
"""
import re
import os

def create_coda_optimized_markdown(input_file, output_file):
    """Create markdown with Coda-friendly image formatting"""
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Pattern to match image references
    pattern = r'!\[([^\]]*)\]\(([^)]+)\)'
    
    def replace_image(match):
        alt_text = match.group(1)
        image_url = match.group(2)
        
        # Skip if already HTML
        if image_url.startswith('<'):
            return match.group(0)
        
        # Create Coda-friendly image with click-to-expand
        # Using HTML that Coda understands
        return f'''<div style="max-width: 100%; overflow: hidden;">
<img src="{image_url}" alt="{alt_text}" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
{alt_text} (Click to view full size)
</p>'''
    
    # Replace all images
    modified_content = re.sub(pattern, replace_image, content)
    
    # Add header with instructions
    header = """<!-- CODA OPTIMIZED VERSION -->
<!-- Images are set to fit within Coda's column width -->
<!-- Click any image to view full size in new tab -->

"""
    
    modified_content = header + modified_content
    
    with open(output_file, 'w') as f:
        f.write(modified_content)
    
    print(f"✓ Created: {output_file}")

def main():
    """Process all GitHub-ready documents"""
    docs = [
        'Bluetooth_GATT_Specification_github.md',
        'Sensor_Logging_Specification_github.md',
        'D2D_Communication_Complete_Guide_github.md',
        'FOTA_Complete_Guide_github.md'
    ]
    
    print("Creating Coda-optimized documentation...")
    print("=" * 50)
    
    for doc in docs:
        if os.path.exists(doc):
            output = doc.replace('_github.md', '_coda.md')
            create_coda_optimized_markdown(doc, output)
    
    print("\n" + "=" * 50)
    print("✓ Complete! Use the *_coda.md files for better image display")
    print("\nThese files have:")
    print("- Images that fit within Coda's columns")
    print("- Click-to-expand functionality")
    print("- Captions indicating full-size viewing")

if __name__ == '__main__':
    main()