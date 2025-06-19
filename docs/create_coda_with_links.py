#!/usr/bin/env python3
"""
Create Coda-friendly docs with external image links
"""
import re
import os

def create_coda_with_links(input_file, output_file):
    """Create markdown with links to full-size images"""
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Pattern to match image references
    pattern = r'!\[([^\]]*)\]\(([^)]+)\)'
    
    def replace_image(match):
        alt_text = match.group(1)
        image_url = match.group(2)
        
        # Skip if already processed
        if '<!-- Original diagram' in image_url:
            return match.group(0)
        
        # Create a thumbnail with link to full size
        return f'''**{alt_text}**
[📊 View Full Size Diagram]({image_url})

*Note: Click the link above to see the full diagram in a new tab*
'''
    
    # Replace all images
    modified_content = re.sub(pattern, replace_image, content)
    
    # Add header
    header = """<!-- CODA VERSION WITH EXTERNAL LINKS -->
<!-- Since Coda crops wide images, this version provides links to view full-size diagrams -->

"""
    
    modified_content = header + modified_content
    
    with open(output_file, 'w') as f:
        f.write(modified_content)
    
    print(f"✓ Created: {output_file}")

def main():
    """Process all documents"""
    docs = [
        'Bluetooth_GATT_Specification_github.md',
        'Sensor_Logging_Specification_github.md',
        'D2D_Communication_Complete_Guide_github.md',
        'FOTA_Complete_Guide_github.md'
    ]
    
    print("Creating Coda docs with external links...")
    print("=" * 50)
    
    for doc in docs:
        if os.path.exists(doc):
            output = doc.replace('_github.md', '_coda_links.md')
            create_coda_with_links(doc, output)
    
    print("\n" + "=" * 50)
    print("✓ Complete! Use *_coda_links.md files")
    print("\nThese replace images with clickable links to full-size versions")

if __name__ == '__main__':
    main()