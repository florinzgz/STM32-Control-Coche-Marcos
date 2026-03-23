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
#   5. Strips trailing empty Defaults fields (|| "") that cause GCC "" errors

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

# ---- Step 5: Fix trailing empty Defaults field ----
# CubeMX migration sometimes leaves a trailing "|| " in the Defaults option
# value, producing an empty field that CubeIDE passes to GCC as "".
if grep -q 'name="Defaults".*|| *" *valueType=' .cproject 2>/dev/null; then
    echo "   ⚠️  Fixing trailing empty Defaults field (|| \"\")"
    sed '/name="Defaults"/s/ *|| *" *valueType=/" valueType=/' .cproject > .cproject.tmp \
        && mv .cproject.tmp .cproject
    FIXED=1
fi

# ---- Step 6: Fix .mxproject duplicate entries, file type mismatches,
#              trailing semicolons, empty entries, and bare list lines ----
# CubeMX migration sometimes corrupts .mxproject with:
#   - duplicated SourceFiles, LibFiles, CDefines entries
#   - .c files incorrectly listed in LibFiles (should be .h only)
#   - .h files incorrectly listed in SourceFiles (should be .c only)
#   - trailing semicolons (;;;) that produce empty source entries
#   - bare "HeaderFiles=;" or "SourceFiles=;" lines that conflict with indexed entries
#   - repeated section blocks
# CubeIDE uses these lists to generate subdir.mk build rules; any of
# these issues cause circular .o deps and empty GCC arguments (gcc "").
if [ -f ".mxproject" ]; then
    echo "🔍 Checking .mxproject for corruption..."

    _PY=""
    for _p in python3 python py; do
        if command -v "$_p" >/dev/null 2>&1; then _PY="$_p"; break; fi
    done

    if [ -n "$_PY" ]; then
        MX_FIXED=$("$_PY" -c "
import sys

fixed = 0
lines = open('.mxproject', 'r').readlines()
out = []
seen_sections = set()
skip_section = False

for line in lines:
    stripped = line.rstrip('\n\r')

    # --- Remove duplicate INI section headers ---
    if stripped.startswith('[') and stripped.endswith(']'):
        if stripped in seen_sections:
            sys.stderr.write('   ⚠️  Removing duplicate section: ' + stripped + '\n')
            fixed += 1
            skip_section = True
            continue
        seen_sections.add(stripped)
        skip_section = False
        out.append(line)
        continue
    if skip_section:
        continue

    # --- Remove bare empty list lines like 'HeaderFiles=;' or 'SourceFiles=;' ---
    if stripped in ('HeaderFiles=;', 'SourceFiles=;', 'HeaderFiles=', 'SourceFiles='):
        sys.stderr.write('   ⚠️  Removing bare empty line: ' + stripped + '\n')
        fixed += 1
        continue

    # --- Fix LibFiles: deduplicate, remove .c files, strip trailing semicolons ---
    if stripped.startswith('LibFiles='):
        value = stripped[len('LibFiles='):]
        items = [x for x in value.split(';') if x.strip()]
        # Remove .c files (they belong in SourceFiles, not LibFiles)
        c_files = [x for x in items if x.strip().endswith('.c')]
        if c_files:
            for cf in c_files:
                sys.stderr.write('   ⚠️  Removing .c file from LibFiles: ' + cf + '\n')
            items = [x for x in items if not x.strip().endswith('.c')]
            fixed += 1
        # Deduplicate
        seen = set(); unique = []
        for item in items:
            if item not in seen:
                seen.add(item); unique.append(item)
        if len(unique) != len(items):
            sys.stderr.write('   ⚠️  Deduplicating LibFiles\n')
            fixed += 1
        out.append('LibFiles=' + ';'.join(sorted(unique)) + '\n')
        continue

    # --- Fix SourceFiles (non-indexed): deduplicate, remove .h files, strip trailing semicolons ---
    if stripped.startswith('SourceFiles=') and not stripped.startswith('SourceFiles#'):
        value = stripped[len('SourceFiles='):]
        items = [x for x in value.split(';') if x.strip()]
        # Remove .h files (they belong in LibFiles, not SourceFiles)
        h_files = [x for x in items if x.strip().endswith('.h')]
        if h_files:
            for hf in h_files:
                sys.stderr.write('   ⚠️  Removing .h file from SourceFiles: ' + hf + '\n')
            items = [x for x in items if not x.strip().endswith('.h')]
            fixed += 1
        # Deduplicate
        seen = set(); unique = []
        for item in items:
            if item not in seen:
                seen.add(item); unique.append(item)
        if len(unique) != len(items):
            sys.stderr.write('   ⚠️  Deduplicating SourceFiles\n')
            fixed += 1
        out.append('SourceFiles=' + ';'.join(sorted(unique)) + '\n')
        continue

    # --- Fix CDefines: deduplicate ---
    if stripped.startswith('CDefines='):
        value = stripped[len('CDefines='):]
        items = [x for x in value.split(';') if x.strip()]
        seen = set(); unique = []
        for item in items:
            if item not in seen:
                seen.add(item); unique.append(item)
        if len(unique) != len(items):
            sys.stderr.write('   ⚠️  Deduplicating CDefines\n')
            fixed += 1
        out.append('CDefines=' + ';'.join(unique) + '\n')
        continue

    # --- Fix HeaderPath: deduplicate ---
    if stripped.startswith('HeaderPath=') and not stripped.startswith('HeaderPath#'):
        value = stripped[len('HeaderPath='):]
        items = [x for x in value.split(';') if x.strip()]
        seen = set(); unique = []
        for item in items:
            if item not in seen:
                seen.add(item); unique.append(item)
        if len(unique) != len(items):
            sys.stderr.write('   ⚠️  Deduplicating HeaderPath\n')
            fixed += 1
        out.append('HeaderPath=' + ';'.join(unique) + '\n')
        continue

    out.append(line)

open('.mxproject', 'w').writelines(out)
print(fixed)
" 2>&1)
        MX_COUNT=$(echo "$MX_FIXED" | tail -1)
        echo "$MX_FIXED" | head -n -1
    else
        # Fallback: use awk for basic deduplication
        MX_COUNT=0
        for KEY in SourceFiles LibFiles CDefines HeaderPath; do
            if grep -q "^${KEY}=" .mxproject 2>/dev/null; then
                BEFORE=$(grep "^${KEY}=" .mxproject | wc -c)
                awk -v key="$KEY" '
                BEGIN { prefix = key "=" }
                $0 ~ "^" prefix {
                    val = substr($0, length(prefix)+1)
                    n = split(val, items, ";")
                    delete seen; out = ""
                    for (i = 1; i <= n; i++) {
                        if (items[i] != "" && !seen[items[i]]++) {
                            out = out items[i] ";"
                        }
                    }
                    print prefix out
                    next
                }
                { print }
                ' .mxproject > .mxproject.tmp && mv .mxproject.tmp .mxproject
                AFTER=$(grep "^${KEY}=" .mxproject | wc -c)
                if [ "$BEFORE" -ne "$AFTER" ]; then
                    echo "   ⚠️  Deduplicating ${KEY} in .mxproject"
                    MX_COUNT=$((MX_COUNT + 1))
                fi
            fi
        done
    fi

    case "$MX_COUNT" in
        ''|*[!0-9]*) MX_COUNT=0 ;;
    esac

    if [ "$MX_COUNT" -eq 0 ]; then
        echo "   ✅ No .mxproject corruption found."
    else
        echo "   ✅ Fixed ${MX_COUNT} issue(s) in .mxproject."
        FIXED=1
    fi
else
    echo "ℹ️  No .mxproject file found (will be created by CubeMX code generation)."
fi

# ---- Step 7: Detect and fix unexpected STM32/ folder structure ----
# Some CubeMX versions generate an unexpected intermediate directory:
#   STM32/Drivers/  instead of  Drivers/
# This breaks all include paths and source entries in .cproject.
if [ -d "STM32" ]; then
    echo "🔍 Detected unexpected STM32/ directory..."
    if [ -d "STM32/Drivers" ]; then
        echo "   ⚠️  Found STM32/Drivers/ — moving to Drivers/"
        if [ -d "Drivers" ]; then
            echo "   ⚠️  Merging STM32/Drivers/ into existing Drivers/"
            cp -rn STM32/Drivers/* Drivers/ 2>/dev/null || true
        else
            mv STM32/Drivers Drivers
        fi
        FIXED=1
    fi
    if [ -d "STM32/Core" ]; then
        echo "   ⚠️  Found STM32/Core/ — moving to Core/"
        if [ -d "Core" ]; then
            echo "   ⚠️  Merging STM32/Core/ into existing Core/"
            cp -rn STM32/Core/* Core/ 2>/dev/null || true
        else
            mv STM32/Core Core
        fi
        FIXED=1
    fi
    # Remove STM32/ if now empty
    rmdir STM32 2>/dev/null && echo "   🧹 Removed empty STM32/ directory" || true
else
    echo "✅ No unexpected STM32/ directory found."
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
