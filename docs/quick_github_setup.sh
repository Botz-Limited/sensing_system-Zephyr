#!/bin/bash
# Quick GitHub setup script for hosting documentation images

echo "==================================="
echo "GitHub Pages Setup for Documentation"
echo "==================================="

# Check if git is initialized
if [ ! -d .git ]; then
    echo "Initializing git repository..."
    git init
fi

# Check if we're in the docs directory
if [ ! -f "Bluetooth_GATT_Specification.md" ]; then
    echo "Error: Please run this script from the docs directory"
    exit 1
fi

# Get GitHub username
echo -n "Enter your GitHub username: "
read GITHUB_USERNAME

echo -n "Enter your repository name (default: sensing_fw): "
read REPO_NAME
REPO_NAME=${REPO_NAME:-sensing_fw}

# Create .gitignore if it doesn't exist
if [ ! -f ../.gitignore ]; then
    echo "Creating .gitignore..."
    cat > ../.gitignore << EOF
# Build files
build/
*.o
*.a
*.so

# IDE files
.vscode/
.idea/

# OS files
.DS_Store
Thumbs.db

# Python
__pycache__/
*.pyc

# Temporary files
*.tmp
*.swp
*~

# But include documentation images
!docs/mermaid_images/
EOF
fi

# Update the Python script with actual values
echo "Updating configuration..."
sed -i.bak "s/YOUR_USERNAME/$GITHUB_USERNAME/g" update_docs_with_github_urls.py
sed -i.bak "s/sensing_fw/$REPO_NAME/g" update_docs_with_github_urls.py

# Run the Python script to create GitHub-ready files
echo "Creating GitHub-ready documentation..."
python3 update_docs_with_github_urls.py

# Stage files
echo "Staging files for commit..."
cd ..
git add docs/mermaid_images
git add docs/*.md
git add docs/index.html
git add .gitignore

# Commit
echo "Creating commit..."
git commit -m "Add documentation with images for GitHub Pages"

# Instructions for next steps
echo ""
echo "==================================="
echo "✓ Local setup complete!"
echo ""
echo "Next steps:"
echo "1. Create a new repository on GitHub:"
echo "   https://github.com/new"
echo "   Name: $REPO_NAME"
echo ""
echo "2. Add the remote and push:"
echo "   git remote add origin https://github.com/$GITHUB_USERNAME/$REPO_NAME.git"
echo "   git branch -M main"
echo "   git push -u origin main"
echo ""
echo "3. Enable GitHub Pages:"
echo "   - Go to: https://github.com/$GITHUB_USERNAME/$REPO_NAME/settings/pages"
echo "   - Source: Deploy from a branch"
echo "   - Branch: main"
echo "   - Folder: /docs"
echo "   - Click Save"
echo ""
echo "4. Your images will be available at:"
echo "   https://$GITHUB_USERNAME.github.io/$REPO_NAME/"
echo ""
echo "5. Use the *_github.md files in Coda"
echo "==================================="