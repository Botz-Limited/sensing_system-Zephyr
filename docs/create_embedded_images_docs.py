#!/usr/bin/env python3
"""
Create markdown documents with base64 embedded images for Coda compatibility
"""
import re
import os
import base64
from pathlib import Path

def image_to_base64(image_path):
    """Convert image to base64 data URI"""
    with open(image_path, 'rb') as f:
        image_data = f.read()
    
    # Determine MIME type
    ext = os.path.splitext(image_path)[1].lower()
    mime_type = 'image/png' if ext == '.png' else 'image/svg+xml'
    
    # Create data URI
    base64_data = base64.b64encode(image_data).decode('utf-8')
    return f"data:{mime_type};base64,{base64_data}"

def embed_images_in_markdown(markdown_file):
    """Replace image references with base64 embedded images"""
    with open(markdown_file, 'r') as f:
        content = f.read()
    
    # Find all image references
    # Pattern matches: ![alt text](path)
    pattern = r'!\[([^\]]*)\]\(([^)]+)\)'
    
    def replace_image(match):
        alt_text = match.group(1)
        image_path = match.group(2)
        
        # Skip if already a data URI or URL
        if image_path.startswith('data:') or image_path.startswith('http'):
            return match.group(0)
        
        # Construct full path
        full_path = os.path.join(os.path.dirname(markdown_file), image_path)
        
        if os.path.exists(full_path):
            print(f"  ✓ Embedding: {image_path}")
            data_uri = image_to_base64(full_path)
            
            # For Coda, we'll use HTML img tag which works better
            return f'<img src="{data_uri}" alt="{alt_text}" style="max-width: 100%;">'
        else:
            print(f"  ✗ Not found: {image_path}")
            return match.group(0)
    
    # Replace all image references
    modified_content = re.sub(pattern, replace_image, content)
    
    # Save with new name
    output_file = markdown_file.replace('_with_images.md', '_embedded.md')
    with open(output_file, 'w') as f:
        f.write(modified_content)
    
    print(f"  → Saved to: {output_file}")
    return output_file

def create_coda_ready_html(markdown_files):
    """Create a single HTML file with all content for easy Coda import"""
    html_content = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Sensing Firmware Documentation</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            line-height: 1.6;
            max-width: 900px;
            margin: 0 auto;
            padding: 20px;
        }
        h1 { color: #333; border-bottom: 3px solid #007AFF; padding-bottom: 10px; }
        h2 { color: #555; margin-top: 30px; }
        h3 { color: #666; }
        code {
            background: #f4f4f4;
            padding: 2px 5px;
            border-radius: 3px;
            font-family: 'Courier New', monospace;
        }
        pre {
            background: #f4f4f4;
            padding: 15px;
            border-radius: 5px;
            overflow-x: auto;
        }
        table {
            border-collapse: collapse;
            width: 100%;
            margin: 20px 0;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 8px;
            text-align: left;
        }
        th {
            background-color: #f2f2f2;
            font-weight: bold;
        }
        img {
            max-width: 100%;
            height: auto;
            display: block;
            margin: 20px auto;
            border: 1px solid #ddd;
            border-radius: 5px;
        }
        .toc {
            background: #f9f9f9;
            border: 1px solid #ddd;
            padding: 20px;
            margin: 20px 0;
            border-radius: 5px;
        }
        .toc h2 {
            margin-top: 0;
        }
        .toc ul {
            list-style-type: none;
            padding-left: 20px;
        }
        .toc a {
            color: #007AFF;
            text-decoration: none;
        }
        .toc a:hover {
            text-decoration: underline;
        }
        .document-section {
            margin-top: 50px;
            padding-top: 30px;
            border-top: 2px solid #eee;
        }
    </style>
</head>
<body>
    <h1>Sensing Firmware Documentation</h1>
    
    <div class="toc">
        <h2>Table of Contents</h2>
        <ul>
            <li><a href="#bluetooth-gatt">Bluetooth GATT Specification</a></li>
            <li><a href="#sensor-logging">Sensor Logging Specification</a></li>
            <li><a href="#d2d-communication">D2D Communication Guide</a></li>
            <li><a href="#fota-guide">FOTA Complete Guide</a></li>
        </ul>
    </div>
"""
    
    # Convert markdown to basic HTML (simplified)
    def markdown_to_html(content):
        # Headers
        content = re.sub(r'^### (.+)$', r'<h3>\1</h3>', content, flags=re.MULTILINE)
        content = re.sub(r'^## (.+)$', r'<h2>\1</h2>', content, flags=re.MULTILINE)
        content = re.sub(r'^# (.+)$', r'<h1>\1</h1>', content, flags=re.MULTILINE)
        
        # Code blocks
        content = re.sub(r'```(\w+)?\n(.*?)\n```', r'<pre><code>\2</code></pre>', content, flags=re.DOTALL)
        
        # Inline code
        content = re.sub(r'`([^`]+)`', r'<code>\1</code>', content)
        
        # Bold
        content = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', content)
        
        # Tables (basic support)
        lines = content.split('\n')
        in_table = False
        new_lines = []
        
        for line in lines:
            if '|' in line and not in_table:
                in_table = True
                new_lines.append('<table>')
            elif in_table and '|' not in line:
                in_table = False
                new_lines.append('</table>')
                new_lines.append(line)
            elif in_table:
                cells = [cell.strip() for cell in line.split('|')[1:-1]]
                if all('-' in cell for cell in cells):
                    continue  # Skip separator line
                row_tag = 'th' if new_lines[-1] == '<table>' else 'td'
                row = '<tr>' + ''.join(f'<{row_tag}>{cell}</{row_tag}>' for cell in cells) + '</tr>'
                new_lines.append(row)
            else:
                new_lines.append(line)
        
        return '\n'.join(new_lines)
    
    # Add each document
    doc_sections = [
        ('bluetooth-gatt', 'Bluetooth GATT Specification', 'Bluetooth_GATT_Specification_embedded.md'),
        ('sensor-logging', 'Sensor Logging Specification', 'Sensor_Logging_Specification_embedded.md'),
        ('d2d-communication', 'D2D Communication Guide', 'D2D_Communication_Complete_Guide_embedded.md'),
        ('fota-guide', 'FOTA Complete Guide', 'FOTA_Complete_Guide_embedded.md')
    ]
    
    for section_id, title, filename in doc_sections:
        if os.path.exists(filename):
            with open(filename, 'r') as f:
                content = f.read()
            
            html_content += f'''
    <div class="document-section" id="{section_id}">
        {markdown_to_html(content)}
    </div>
'''
    
    html_content += """
</body>
</html>
"""
    
    with open('sensing_firmware_docs_for_coda.html', 'w') as f:
        f.write(html_content)
    
    print("\n✓ Created: sensing_firmware_docs_for_coda.html")
    print("  This file can be opened in a browser and content copied to Coda")

def main():
    """Main function"""
    docs_dir = '/home/ee/sensing_fw/docs'
    os.chdir(docs_dir)
    
    # Documents to process
    markdown_files = [
        'Bluetooth_GATT_Specification_with_images.md',
        'Sensor_Logging_Specification_with_images.md',
        'D2D_Communication_Complete_Guide_with_images.md',
        'FOTA_Complete_Guide_with_images.md'
    ]
    
    print("Creating documents with embedded images for Coda...")
    print("=" * 50)
    
    embedded_files = []
    
    for md_file in markdown_files:
        if os.path.exists(md_file):
            print(f"\nProcessing: {md_file}")
            embedded_file = embed_images_in_markdown(md_file)
            embedded_files.append(embedded_file)
    
    # Create combined HTML file
    print("\nCreating combined HTML file...")
    create_coda_ready_html(embedded_files)
    
    print("\n" + "=" * 50)
    print("✓ Embedded image documents created!")
    print("\nFor Coda:")
    print("1. Open any *_embedded.md file in a markdown viewer")
    print("2. Copy all content and paste into Coda")
    print("   OR")
    print("3. Open sensing_firmware_docs_for_coda.html in a browser")
    print("4. Copy sections and paste into Coda")
    print("\nThe images are now embedded and will display correctly!")

if __name__ == '__main__':
    main()