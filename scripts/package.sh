#!/bin/bash
# Package Analog Pocket Core

set -e

echo "📦 Creating Analog Pocket core package..."

# Create the core directory structure
mkdir -p package/Cores/RndMnkIII.JVS_Debugger

# Copy dist folder if it exists
if [ -d "dist" ]; then
  echo "📁 Copying dist folder..."
  cp -r dist/* package/
else
  echo "⚠️ No dist folder found, creating basic structure"
  mkdir -p package/Cores/RndMnkIII.JVS_Debugger
fi

# Copy the RBF_R file to the core directory
echo "📄 Copying jvs_debug.rbf_r to core directory..."
cp src/fpga/output_files/jvs_debug.rbf_r package/Cores/RndMnkIII.JVS_Debugger/

echo "✅ Package structure created:"
find package -type f | head -20

# Create complete package ZIP
echo "📦 Creating complete package ZIP..."
cd package
zip -r ../jvs_debugger_complete_package.zip .
cd ..
mv jvs_debugger_complete_package.zip package/

echo "✅ Package created successfully"