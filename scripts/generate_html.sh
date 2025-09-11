#!/bin/bash
# Generate HTML from template

set -e

TEMPLATE_FILE="${SCRIPTS_DIR:-scripts}/index.html.template"
OUTPUT_FILE="package/index.html"
STATS_FILE="src/fpga/fpga_stats.txt"

echo "🌐 Generating HTML page..."

# Create output directory
mkdir -p $(dirname "$OUTPUT_FILE")

# Start with template
cp "$TEMPLATE_FILE" "$OUTPUT_FILE"

# Replace build info
sed -i "s/{{BUILD_NUMBER}}/${BUILD_NUMBER:-Unknown}/g" "$OUTPUT_FILE"
sed -i "s/{{COMMIT_SHA}}/${COMMIT_SHA:-Unknown}/g" "$OUTPUT_FILE"
sed -i "s/{{BUILD_DATE}}/${BUILD_DATE:-Unknown}/g" "$OUTPUT_FILE"
sed -i "s/{{BRANCH_NAME}}/${BRANCH_NAME:-Unknown}/g" "$OUTPUT_FILE"

# Generate FPGA stats
FPGA_STATS=""
if [ -f "$STATS_FILE" ]; then
    echo "📊 Processing FPGA statistics..."
    # Extract key stats using dedicated script
    eval $(${SCRIPTS_DIR:-scripts}/extract_fpga_stats.py "$STATS_FILE")
    
    # Create summary cards
    FPGA_STATS="<div class=\"stat-card\"><div class=\"stat-value\">ALMs</div><div class=\"stat-label\">$LOGIC_ALMS</div></div>"
    FPGA_STATS="$FPGA_STATS<div class=\"stat-card\"><div class=\"stat-value\">RAM</div><div class=\"stat-label\">$MEMORY_BITS</div></div>"
    FPGA_STATS="$FPGA_STATS<div class=\"stat-card\"><div class=\"stat-value\">DSP</div><div class=\"stat-label\">$DSP_BLOCKS</div></div>"
    FPGA_STATS="$FPGA_STATS<div class=\"stat-card\"><div class=\"stat-value\">PLL</div><div class=\"stat-label\">$PLLS</div></div>"
    
    # Create detailed stats (separate)
    FPGA_STATS_DETAILED=$(cat "$STATS_FILE" | sed 's/</\&lt;/g' | sed 's/>/\&gt;/g')
fi

# Replace FPGA stats using Python to avoid sed issues with special characters
python3 -c "
import sys
with open('$OUTPUT_FILE', 'r') as f:
    content = f.read()
content = content.replace('{{FPGA_STATS}}', '''$FPGA_STATS''')
content = content.replace('{{FPGA_STATS_DETAILED}}', '''$FPGA_STATS_DETAILED''')
with open('$OUTPUT_FILE', 'w') as f:
    f.write(content)
"

# Generate file list
FILE_LIST=""
if [ -d "package" ]; then
    FILE_LIST="<ul>"
    while IFS= read -r file; do
        FILE_LIST="$FILE_LIST<li>$file</li>"
    done < <(find package -type f | sed 's|package/||' | sort)
    FILE_LIST="$FILE_LIST</ul>"
fi

# Replace file list
python3 -c "
import sys
with open('$OUTPUT_FILE', 'r') as f:
    content = f.read()
content = content.replace('{{FILE_LIST}}', '''$FILE_LIST''')
with open('$OUTPUT_FILE', 'w') as f:
    f.write(content)
"

echo "✅ HTML generated: $OUTPUT_FILE"