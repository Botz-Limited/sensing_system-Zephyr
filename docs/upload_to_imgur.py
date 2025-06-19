#!/usr/bin/env python3
"""
Upload images to Imgur and update markdown documents
No account required - uses anonymous upload
"""
import os
import base64
import json
import requests
import re
from pathlib import Path

# Imgur anonymous upload endpoint
IMGUR_UPLOAD_URL = "https://api.imgur.com/3/image"
IMGUR_CLIENT_ID = "546c25a59c58ad7"  # Public client ID for anonymous uploads

def upload_to_imgur(image_path):
    """Upload an image to Imgur anonymously"""
    with open(image_path, 'rb') as f:
        image_data = f.read()
    
    # Convert to base64
    b64_image = base64.b64encode(image_data).decode('utf-8')
    
    # Prepare headers and data
    headers = {
        'Authorization': f'Client-ID {IMGUR_CLIENT_ID}'
    }
    
    data = {
        'image': b64_image,
        'type': 'base64',
        'name': os.path.basename(image_path),
        'title': f'Sensing FW - {os.path.basename(image_path)}'
    }
    
    try:
        response = requests.post(IMGUR_UPLOAD_URL, headers=headers, data=data)
        if response.status_code == 200:
            result = response.json()
            if result['success']:
                return result['data']['link']
            else:
                print(f"  ✗ Upload failed: {result.get('data', {}).get('error', 'Unknown error')}")
                return None
        else:
            print(f"  ✗ HTTP error {response.status_code}")
            return None
    except Exception as e:
        print(f"  ✗ Error uploading: {e}")
        return None

def update_markdown_with_imgur_urls(markdown_file, url_mapping):
    """Update markdown file with Imgur URLs"""
    with open(markdown_file, 'r') as f:
        content = f.read()
    
    # Pattern to match image references
    pattern = r'!\[([^\]]*)\]\(([^)]+)\)'
    
    def replace_image_url(match):
        alt_text = match.group(1)
        image_path = match.group(2)
        
        # Skip if already a URL
        if image_path.startswith('http'):
            return match.group(0)
        
        # Get the filename from the path
        filename = os.path.basename(image_path)
        
        # Look for the Imgur URL in our mapping
        for local_path, imgur_url in url_mapping.items():
            if filename in local_path:
                print(f"  ✓ Replaced: {filename} → {imgur_url}")
                return f"![{alt_text}]({imgur_url})"
        
        return match.group(0)
    
    # Replace all image references
    modified_content = re.sub(pattern, replace_image_url, content)
    
    # Save with new name
    output_file = markdown_file.replace('.md', '_imgur.md')
    with open(output_file, 'w') as f:
        f.write(modified_content)
    
    print(f"  → Saved to: {output_file}")

def save_url_mapping(url_mapping):
    """Save URL mapping for future reference"""
    with open('imgur_urls.json', 'w') as f:
        json.dump(url_mapping, f, indent=2)
    print("\n✓ URL mapping saved to imgur_urls.json")

def main():
    """Main function"""
    print("=" * 60)
    print("Imgur Image Upload for Documentation")
    print("=" * 60)
    print("\nThis will upload images anonymously to Imgur.")
    print("Images will be public but not listed on Imgur's gallery.")
    
    response = input("\nContinue? (y/n): ")
    if response.lower() != 'y':
        print("Cancelled.")
        return
    
    # Find all PNG images
    image_dir = "mermaid_images"
    images = []
    for root, dirs, files in os.walk(image_dir):
        for file in files:
            if file.endswith('.png'):
                images.append(os.path.join(root, file))
    
    print(f"\nFound {len(images)} images to upload.")
    
    # Upload images
    url_mapping = {}
    print("\nUploading images to Imgur...")
    
    for i, image_path in enumerate(images, 1):
        print(f"\n[{i}/{len(images)}] Uploading: {image_path}")
        imgur_url = upload_to_imgur(image_path)
        
        if imgur_url:
            url_mapping[image_path] = imgur_url
            print(f"  ✓ Success: {imgur_url}")
        else:
            print(f"  ✗ Failed to upload {image_path}")
    
    # Save mapping
    save_url_mapping(url_mapping)
    
    # Update markdown files
    print("\nUpdating markdown files...")
    docs_to_update = [
        'Bluetooth_GATT_Specification_with_images.md',
        'Sensor_Logging_Specification_with_images.md',
        'D2D_Communication_Complete_Guide_with_images.md',
        'FOTA_Complete_Guide_with_images.md'
    ]
    
    for doc in docs_to_update:
        if os.path.exists(doc):
            print(f"\nProcessing: {doc}")
            update_markdown_with_imgur_urls(doc, url_mapping)
    
    print("\n" + "=" * 60)
    print("✓ Upload Complete!")
    print(f"\nSuccessfully uploaded {len(url_mapping)} images to Imgur")
    print("\nThe *_imgur.md files now contain Imgur URLs that will work in Coda.")
    print("\nNote: Imgur URLs are permanent but anonymous uploads may be")
    print("removed if they violate Imgur's terms or are not viewed for 6 months.")

if __name__ == '__main__':
    main()