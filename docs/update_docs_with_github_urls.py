#!/usr/bin/env python3
"""
Update markdown documents to use GitHub-hosted images
"""
import re
import os

# Configuration - UPDATE THESE VALUES
GITHUB_USERNAME = "Botz-Limited"   # Your GitHub organization
REPO_NAME = "sensing_system-Zephyr"  # Your repository name
BRANCH = "Nrf5340-framework"       # Your current branch

def update_image_urls(markdown_file, base_url):
    """Replace local image paths with GitHub URLs"""
    with open(markdown_file, 'r') as f:
        content = f.read()
    
    # Base URL for raw GitHub content
    github_base = f"https://raw.githubusercontent.com/{GITHUB_USERNAME}/{REPO_NAME}/{BRANCH}/docs"
    
    # Pattern to match image references
    # Matches: ![alt text](path)
    pattern = r'!\[([^\]]*)\]\(([^)]+)\)'
    
    def replace_image_url(match):
        alt_text = match.group(1)
        image_path = match.group(2)
        
        # Skip if already a URL
        if image_path.startswith('http'):
            return match.group(0)
        
        # Skip HTML comments
        if '<!--' in image_path:
            return match.group(0)
        
        # Convert relative path to GitHub URL
        if image_path.startswith('mermaid_images/'):
            new_url = f"{github_base}/{image_path}"
            print(f"  ✓ Updated: {image_path}")
            print(f"    → {new_url}")
            return f"![{alt_text}]({new_url})"
        
        return match.group(0)
    
    # Replace all image references
    modified_content = re.sub(pattern, replace_image_url, content)
    
    # Save with new name
    output_file = markdown_file.replace('.md', '_github.md')
    with open(output_file, 'w') as f:
        f.write(modified_content)
    
    print(f"  → Saved to: {output_file}")
    return output_file

def create_github_pages_index():
    """Create an index.html for GitHub Pages"""
    html_content = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Sensing Firmware Documentation Images</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            background: #f5f5f5;
        }
        h1, h2 { color: #333; }
        .image-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            gap: 20px;
            margin: 20px 0;
        }
        .image-card {
            background: white;
            border-radius: 8px;
            padding: 15px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .image-card img {
            width: 100%;
            height: auto;
            border: 1px solid #ddd;
            border-radius: 4px;
        }
        .image-card h3 {
            margin: 10px 0 5px 0;
            font-size: 16px;
        }
        .image-card code {
            display: block;
            background: #f4f4f4;
            padding: 5px;
            border-radius: 3px;
            font-size: 12px;
            word-break: break-all;
            margin: 5px 0;
        }
        .copy-btn {
            background: #007AFF;
            color: white;
            border: none;
            padding: 5px 10px;
            border-radius: 3px;
            cursor: pointer;
            font-size: 12px;
        }
        .copy-btn:hover {
            background: #0051D5;
        }
    </style>
</head>
<body>
    <h1>Sensing Firmware Documentation Images</h1>
    <p>These images are hosted on GitHub Pages for use in documentation.</p>
"""
    
    # Add sections for each document
    docs = [
        ('Bluetooth_GATT_Specification', 7),
        ('Sensor_Logging_Specification', 11),
        ('D2D_Communication_Complete_Guide', 8),
        ('FOTA_Complete_Guide', 6)
    ]
    
    for doc_name, count in docs:
        html_content += f"\n<h2>{doc_name.replace('_', ' ')}</h2>\n<div class='image-grid'>\n"
        
        for i in range(1, count + 1):
            img_url = f"https://{GITHUB_USERNAME}.github.io/{REPO_NAME}/mermaid_images/{doc_name}/diagram_{i}.png"
            raw_url = f"https://raw.githubusercontent.com/{GITHUB_USERNAME}/{REPO_NAME}/{BRANCH}/docs/mermaid_images/{doc_name}/diagram_{i}.png"
            
            html_content += f"""
    <div class="image-card">
        <img src="mermaid_images/{doc_name}/diagram_{i}.png" alt="Diagram {i}">
        <h3>Diagram {i}</h3>
        <code id="url-{doc_name}-{i}">{raw_url}</code>
        <button class="copy-btn" onclick="copyToClipboard('url-{doc_name}-{i}')">Copy URL</button>
    </div>
"""
        
        html_content += "</div>\n"
    
    html_content += """
<script>
function copyToClipboard(id) {
    const element = document.getElementById(id);
    const text = element.textContent;
    navigator.clipboard.writeText(text).then(() => {
        const btn = element.nextElementSibling;
        btn.textContent = 'Copied!';
        setTimeout(() => { btn.textContent = 'Copy URL'; }, 2000);
    });
}
</script>
</body>
</html>
"""
    
    with open('index.html', 'w') as f:
        f.write(html_content)
    
    print("\n✓ Created index.html for GitHub Pages")

def main():
    """Main function"""
    print("=" * 60)
    print("GitHub Image Hosting Setup")
    print("=" * 60)
    
    if GITHUB_USERNAME == "YOUR_USERNAME":
        print("\n⚠️  Please update the configuration in this script:")
        print("   - GITHUB_USERNAME: Your GitHub username")
        print("   - REPO_NAME: Your repository name")
        print("   - BRANCH: Your default branch (main or master)")
        print("\nEdit this file and run again.")
        return
    
    print(f"\nConfiguration:")
    print(f"  GitHub Username: {GITHUB_USERNAME}")
    print(f"  Repository: {REPO_NAME}")
    print(f"  Branch: {BRANCH}")
    
    # Update documents
    docs_to_update = [
        'Bluetooth_GATT_Specification.md',
        'Sensor_Logging_Specification.md',
        'D2D_Communication_Complete_Guide.md',
        'FOTA_Complete_Guide.md'
    ]
    
    print("\nUpdating documents with GitHub URLs...")
    for doc in docs_to_update:
        if os.path.exists(doc):
            print(f"\nProcessing: {doc}")
            update_image_urls(doc, "")
    
    # Create GitHub Pages index
    create_github_pages_index()
    
    print("\n" + "=" * 60)
    print("✓ Setup Complete!")
    print("\nNext steps:")
    print("1. Update the configuration in this script with your GitHub details")
    print("2. Commit and push to GitHub:")
    print("   git add -A")
    print("   git commit -m 'Add documentation with images'")
    print("   git push origin main")
    print("3. Enable GitHub Pages in repository settings")
    print("4. Use the *_github.md files for Coda")
    print("\nYour images will be available at:")
    print(f"https://{GITHUB_USERNAME}.github.io/{REPO_NAME}/")

if __name__ == '__main__':
    main()