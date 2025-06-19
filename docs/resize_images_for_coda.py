#!/usr/bin/env python3
"""
Resize images for better display in Coda
"""
import os
from PIL import Image
import shutil

def resize_image(input_path, output_path, max_width=800):
    """Resize image to fit within max_width while maintaining aspect ratio"""
    try:
        with Image.open(input_path) as img:
            # Only resize if image is wider than max_width
            if img.width > max_width:
                # Calculate new height maintaining aspect ratio
                ratio = max_width / img.width
                new_height = int(img.height * ratio)
                
                # Resize image
                resized = img.resize((max_width, new_height), Image.Resampling.LANCZOS)
                resized.save(output_path, optimize=True)
                print(f"  ✓ Resized: {img.width}x{img.height} → {max_width}x{new_height}")
                return True
            else:
                # Copy as-is if already small enough
                shutil.copy2(input_path, output_path)
                print(f"  ✓ Copied: {img.width}x{img.height} (already optimal)")
                return True
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False

def main():
    """Main function"""
    print("Creating Coda-optimized images...")
    print("=" * 50)
    
    # Create output directory
    output_dir = "mermaid_images_coda"
    os.makedirs(output_dir, exist_ok=True)
    
    # Process all PNG images
    source_dir = "mermaid_images"
    
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            if file.endswith('.png'):
                # Create relative path
                rel_path = os.path.relpath(root, source_dir)
                output_subdir = os.path.join(output_dir, rel_path)
                os.makedirs(output_subdir, exist_ok=True)
                
                # Resize image
                input_file = os.path.join(root, file)
                output_file = os.path.join(output_subdir, file)
                
                print(f"\nProcessing: {input_file}")
                resize_image(input_file, output_file, max_width=800)
    
    print("\n" + "=" * 50)
    print("✓ Complete! Resized images are in 'mermaid_images_coda/'")
    print("\nTo use these smaller images:")
    print("1. Upload to GitHub")
    print("2. Update the markdown files to use the new paths")

if __name__ == '__main__':
    main()