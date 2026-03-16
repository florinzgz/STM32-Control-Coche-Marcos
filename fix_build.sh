#!/bin/bash
# fix_build.sh — Fix circular-dependency build errors in STM32CubeIDE
#
# Run this script AFTER STM32CubeMX code generation (which may overwrite
# .cproject with broken element IDs), or whenever you see build errors like:
#
#   make: Circular *.o <- *.o dependency dropped.
#   Building file:
#   arm-none-eabi-gcc "" ...
#
# Root cause: when a .cproject element's "id" attribute equals its
# "superClass" attribute, Eclipse CDT confuses the instance with the
# template and generates subdir.mk rules where .o files depend on
# themselves instead of the corresponding .c source file.
#
# This script:
#   1. Removes the stale Debug/ build directory
#   2. Scans .cproject for id==superClass conflicts and fixes them
#   3. Scans for duplicate element IDs among managed-build elements
#   4. Fixes the assembler inputType superClass if needed

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "🔧 STM32CubeIDE Build Fix Script"
echo "================================="
echo ""

FIXED=0

# ---- Step 1: Clean stale Debug directory ----
if [ -d "Debug" ]; then
    echo "🧹 Removing stale Debug/ directory..."
    rm -rf Debug
    FIXED=1
fi

# ---- Step 2: Fix .cproject element IDs ----
if [ ! -f ".cproject" ]; then
    echo "❌ .cproject not found in $(pwd)"
    exit 1
fi

echo "🔍 Checking .cproject for id==superClass conflicts..."

CONFLICTS=0
COUNTER=0
# Process lines that contain both id="..." and superClass="..."
# The grep output is captured once via process substitution; each sed
# call modifies the file on disk so changes accumulate correctly.
while IFS= read -r line; do
    # Extract id value
    id_val=$(echo "$line" | sed -n 's/.*[[:space:]]id="\([^"]*\)".*/\1/p')
    # Extract superClass value
    sc_val=$(echo "$line" | sed -n 's/.*superClass="\([^"]*\)".*/\1/p')

    if [ -n "$id_val" ] && [ -n "$sc_val" ] && [ "$id_val" = "$sc_val" ]; then
        # Generate a unique numeric suffix combining timestamp, PID, and counter
        COUNTER=$((COUNTER + 1))
        suffix="$(awk -v seed="$(($(date +%s) * $$ + COUNTER * 97))" 'BEGIN{srand(seed); printf "%09d", int(rand()*1000000000)}')"
        new_id="${id_val}.${suffix}"
        echo "   ⚠️  Fixing: ${id_val}"
        echo "            → ${new_id}"
        # Replace exactly the id="<value>" token (dots escaped for regex).
        # Each CDT element has a distinct id, so this is a single-match replace.
        escaped_id=$(echo "$id_val" | sed 's/\./\\./g')
        sed "s|id=\"${escaped_id}\"|id=\"${new_id}\"|" .cproject > .cproject.tmp \
            && mv .cproject.tmp .cproject
        CONFLICTS=$((CONFLICTS + 1))
        FIXED=1
    fi
done < <(grep -E 'id="[^"]*".*superClass="[^"]*"|superClass="[^"]*".*id="[^"]*"' .cproject)

if [ "$CONFLICTS" -eq 0 ]; then
    echo "   ✅ No id==superClass conflicts found."
else
    echo "   ✅ Fixed $CONFLICTS conflict(s)."
fi

# ---- Step 3: Fix duplicate element IDs among managed-build elements ----
# Only check elements that have a superClass attribute (tool, option,
# inputType, builder, targetPlatform, toolChain).  The cconfiguration,
# storageModule, and configuration elements intentionally share the
# same ID for the same build configuration — that is normal.
echo "🔍 Checking .cproject for duplicate element IDs..."

DUP_IDS=$(grep -E 'superClass="[^"]*"' .cproject 2>/dev/null \
    | sed -n 's/.*id="\([^"]*\)".*/\1/p' \
    | sort | uniq -d)
DUP_FIXED=0

if [ -n "$DUP_IDS" ]; then
    while IFS= read -r dup_id; do
        escaped_id=$(echo "$dup_id" | sed 's/\./\\./g')
        # Count occurrences
        COUNT=$(grep -c "id=\"$dup_id\"" .cproject 2>/dev/null || echo 0)
        if [ "$COUNT" -gt 1 ]; then
            echo "   ⚠️  Duplicate id: $dup_id (${COUNT}x)"
            # Replace all but the first occurrence with unique IDs
            SKIP=1
            while [ "$SKIP" -lt "$COUNT" ]; do
                COUNTER=$((COUNTER + 1))
                # Seed uses timestamp, PID, and counter to ensure uniqueness
                suffix="$(awk -v seed="$(($(date +%s) * $$ + COUNTER * 131))" \
                    'BEGIN{srand(seed); printf "%09d", int(rand()*1000000000)}')"
                new_id="${dup_id}.${suffix}"
                # Use awk to replace only the Nth occurrence
                awk -v old="$dup_id" -v new="$new_id" -v skip="$SKIP" '
                {
                    n = gsub("id=\"" old "\"", "id=\"" old "\"")
                    if (n > 0) {
                        count += n
                        if (count > skip) {
                            sub("id=\"" old "\"", "id=\"" new "\"")
                        }
                    }
                    print
                }' .cproject > .cproject.tmp && mv .cproject.tmp .cproject
                DUP_FIXED=$((DUP_FIXED + 1))
                SKIP=$((SKIP + 1))
            done
        fi
    done <<< "$DUP_IDS"
    FIXED=1
fi

if [ "$DUP_FIXED" -eq 0 ] && [ -z "$DUP_IDS" ]; then
    echo "   ✅ No duplicate element IDs found."
fi

# ---- Step 4: Check assembler inputType superClass ----
if grep -q 'superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.assembler.input.s"' .cproject 2>/dev/null; then
    echo "   ⚠️  Fixing assembler inputType superClass (.input.s → .input)"
    sed 's|superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.assembler.input.s"|superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.assembler.input"|' \
        .cproject > .cproject.tmp && mv .cproject.tmp .cproject
    FIXED=1
fi

# ---- Summary ----
echo ""
if [ "$FIXED" -eq 0 ]; then
    echo "✅ No issues found. Your project should build correctly."
else
    echo "✅ Fixes applied. Next steps:"
    echo "   1. In STM32CubeIDE: right-click project → Refresh (F5)"
    echo "   2. Project → Clean... → select project → Clean"
    echo "   3. Build the project (Ctrl+B)"
fi
echo ""
