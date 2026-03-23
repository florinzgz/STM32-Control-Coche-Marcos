#!/bin/bash
# validate_cproject.sh — Comprehensive health check for .cproject
#
# Validates the STM32CubeIDE .cproject file for common corruption
# patterns that cause circular-dependency build errors.  Run this
# script after every CubeMX code generation to verify the file is
# intact before building.
#
# Checks performed:
#   1. File existence and XML well-formedness
#   2. Elements where id == superClass  (causes circular .o deps)
#   3. Duplicate element IDs  (different elements sharing the same id)
#   4. Wrong assembler inputType superClass  (.input.s instead of .input)
#   5. Required tool elements present  (compiler, linker, assembler)
#   6. Stale Debug/ directory with circular .o self-dependencies
#   7. Trailing empty Defaults field  (|| "" causes GCC "" argument)
#
# Exit codes:
#   0  — all checks passed
#   1  — one or more issues detected (see output for details)
#   2  — .cproject file not found

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

CPROJECT=".cproject"
ERRORS=0
WARNINGS=0

echo "🔍 STM32CubeIDE .cproject Validation"
echo "======================================"
echo ""

# ---- Check 0: File exists ----
if [ ! -f "$CPROJECT" ]; then
    echo "❌ FAIL: $CPROJECT not found in $(pwd)"
    echo ""
    echo "   You can safely regenerate it:"
    echo "   1. Open the .ioc file in STM32CubeMX"
    echo "   2. Click 'Generate Code' (Alt+K)"
    echo "   CubeMX will create a fresh .cproject from the .ioc settings."
    exit 2
fi

# ---- Check 1: XML well-formedness ----
echo "1. XML well-formedness"
if command -v xmllint >/dev/null 2>&1; then
    if xmllint --noout "$CPROJECT" 2>/dev/null; then
        echo "   ✅ PASS — XML is well-formed."
    else
        echo "   ❌ FAIL — XML is malformed."
        echo "   Recommendation: delete .cproject and regenerate from .ioc"
        ERRORS=$((ERRORS + 1))
    fi
elif _PY=$(command -v python3 || command -v python || command -v py) 2>/dev/null && "$_PY" -c "import xml.etree.ElementTree" 2>/dev/null; then
    # Fallback: use Python standard-library XML parser (python3, python, or py)
    if "$_PY" -c "import xml.etree.ElementTree as ET; ET.parse('$CPROJECT')" 2>/dev/null; then
        echo "   ✅ PASS — XML is well-formed ($(basename "$_PY"))."
    else
        echo "   ❌ FAIL — XML is malformed ($(basename "$_PY"))."
        echo "   Recommendation: delete .cproject and regenerate from .ioc"
        ERRORS=$((ERRORS + 1))
    fi
elif _PS=$(command -v powershell.exe || command -v pwsh) 2>/dev/null; then
    # Fallback: use PowerShell XML parser (always available on Windows/MINGW64)
    if "$_PS" -NoProfile -Command "[xml](Get-Content '$CPROJECT')" >/dev/null 2>&1; then
        echo "   ✅ PASS — XML is well-formed ($(basename "$_PS"))."
    else
        echo "   ❌ FAIL — XML is malformed ($(basename "$_PS"))."
        echo "   Recommendation: delete .cproject and regenerate from .ioc"
        ERRORS=$((ERRORS + 1))
    fi
else
    # Fallback: basic check for matching tags
    open_tags=$(grep -co '<[a-zA-Z]' "$CPROJECT" 2>/dev/null || echo 0)
    close_tags=$(grep -co '</[a-zA-Z]' "$CPROJECT" 2>/dev/null || echo 0)
    self_close=$(grep -co '/>' "$CPROJECT" 2>/dev/null || echo 0)
    if [ "$open_tags" -gt 0 ]; then
        echo "   ⚠️  SKIP — xmllint not available; basic tag check:"
        echo "      Open tags: $open_tags, Close tags: $close_tags, Self-closing: $self_close"
        WARNINGS=$((WARNINGS + 1))
    else
        echo "   ❌ FAIL — file appears empty or not XML."
        ERRORS=$((ERRORS + 1))
    fi
fi

# ---- Check 2: id == superClass conflicts ----
echo ""
echo "2. id == superClass conflicts"
ID_SC_CONFLICTS=0
while IFS= read -r line; do
    id_val=$(echo "$line" | sed -n 's/.*[[:space:]]id="\([^"]*\)".*/\1/p')
    sc_val=$(echo "$line" | sed -n 's/.*superClass="\([^"]*\)".*/\1/p')
    if [ -n "$id_val" ] && [ -n "$sc_val" ] && [ "$id_val" = "$sc_val" ]; then
        echo "   ❌ FAIL — id == superClass: $id_val"
        ID_SC_CONFLICTS=$((ID_SC_CONFLICTS + 1))
    fi
done < <(grep -E 'id="[^"]*".*superClass="[^"]*"|superClass="[^"]*".*id="[^"]*"' "$CPROJECT" 2>/dev/null)

if [ "$ID_SC_CONFLICTS" -eq 0 ]; then
    echo "   ✅ PASS — no id==superClass conflicts."
else
    echo "   Found $ID_SC_CONFLICTS conflict(s). Run fix_build.sh to repair."
    ERRORS=$((ERRORS + ID_SC_CONFLICTS))
fi

# ---- Check 3: Duplicate element IDs among managed-build elements ----
# Only check elements that have a superClass attribute (tool, option,
# inputType, builder, targetPlatform, toolChain).  The cconfiguration,
# storageModule, and configuration elements intentionally share the
# same ID for the same build configuration — that is normal.
echo ""
echo "3. Duplicate element IDs"
DUP_IDS=$(grep -E 'superClass="[^"]*"' "$CPROJECT" 2>/dev/null \
    | sed -n 's/.*id="\([^"]*\)".*/\1/p' \
    | sort | uniq -d)
if [ -z "$DUP_IDS" ]; then
    echo "   ✅ PASS — all element IDs are unique."
else
    DUP_COUNT=$(echo "$DUP_IDS" | wc -l)
    echo "   ❌ FAIL — $DUP_COUNT duplicate ID(s) found:"
    echo "$DUP_IDS" | while IFS= read -r dup; do
        echo "      $dup"
    done
    echo "   Run fix_build.sh to repair."
    ERRORS=$((ERRORS + DUP_COUNT))
fi

# ---- Check 4: Wrong assembler inputType superClass ----
echo ""
echo "4. Assembler inputType superClass"
if grep -q 'superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.assembler.input.s"' "$CPROJECT" 2>/dev/null; then
    echo "   ❌ FAIL — uses .input.s instead of .input"
    echo "   Run fix_build.sh to repair."
    ERRORS=$((ERRORS + 1))
else
    echo "   ✅ PASS — assembler inputType is correct."
fi

# ---- Check 5: Required tool elements ----
echo ""
echo "5. Required tool elements"
MISSING_TOOLS=0

check_tool() {
    local label="$1"
    local pattern="$2"
    if grep -q "$pattern" "$CPROJECT" 2>/dev/null; then
        echo "   ✅ $label"
    else
        echo "   ❌ MISSING: $label"
        MISSING_TOOLS=$((MISSING_TOOLS + 1))
    fi
}

check_tool "MCU GCC Assembler"  'managedbuild.tool.assembler'
check_tool "MCU GCC Compiler"   'managedbuild.tool.c.compiler'
check_tool "MCU G++ Compiler"   'managedbuild.tool.cpp.compiler'
check_tool "MCU GCC Linker"     'managedbuild.tool.c.linker'
check_tool "MCU G++ Linker"     'managedbuild.tool.cpp.linker'
check_tool "Linker script ref"  'STM32G474RETX_FLASH.ld'
check_tool "Debug configuration" 'name="Debug"'

if [ "$MISSING_TOOLS" -gt 0 ]; then
    echo "   $MISSING_TOOLS required element(s) missing."
    echo "   Recommendation: delete .cproject and regenerate from .ioc"
    ERRORS=$((ERRORS + MISSING_TOOLS))
fi

# ---- Check 6: Stale Debug/ directory ----
echo ""
echo "6. Stale Debug/ directory"
if [ -d "Debug" ]; then
    STALE=$(find Debug -name 'subdir.mk' -exec awk '
        /\.o[[:space:]]*:/ {
            t = $0; sub(/:.*$/, "", t); gsub(/^[[:space:]]+|[[:space:]]+$/, "", t)
            p = $0; sub(/^[^:]*:/, "", p)
            if (t ~ /\.o$/ && index(p, t) > 0) { print FILENAME; exit }
        }' {} + 2>/dev/null | head -1)
    if [ -n "$STALE" ]; then
        echo "   ❌ FAIL — circular .o self-dependencies in $STALE"
        echo "   Run fix_build.sh or delete Debug/ and rebuild."
        ERRORS=$((ERRORS + 1))
    else
        echo "   ✅ PASS — Debug/ exists with no circular deps detected."
    fi
else
    echo "   ✅ PASS — no Debug/ directory (will be regenerated on build)."
fi

# ---- Check 7: Trailing empty Defaults field ----
# CubeMX migration sometimes appends an empty field ("|| ") to the Defaults
# option value.  CubeIDE emits the empty field as "" on the GCC command line,
# causing "linker input file not found" build errors.
echo ""
echo "7. Trailing empty Defaults field"
if grep -q 'name="Defaults".*|| *" *valueType=' "$CPROJECT" 2>/dev/null; then
    echo "   ❌ FAIL — Defaults option has trailing empty field (|| \"\")"
    echo "   This causes arm-none-eabi-gcc to receive an empty \"\" argument."
    echo "   Run fix_build.sh to repair."
    ERRORS=$((ERRORS + 1))
else
    echo "   ✅ PASS — no trailing empty Defaults field."
fi

# ---- Check 8: .mxproject duplicate entries ----
echo ""
echo "8. .mxproject duplicate entries"
if [ -f ".mxproject" ]; then
    MX_DUPS=0
    for KEY in SourceFiles LibFiles CDefines HeaderPath; do
        if grep -q "^${KEY}=" .mxproject 2>/dev/null; then
            LINE=$(grep "^${KEY}=" .mxproject)
            VALUE="${LINE#*=}"
            TOTAL=$(echo "$VALUE" | tr ';' '\n' | awk 'NF' | wc -l)
            UNIQUE=$(echo "$VALUE" | tr ';' '\n' | awk 'NF' | sort -u | wc -l)
            if [ "$TOTAL" -ne "$UNIQUE" ]; then
                DUP_N=$((TOTAL - UNIQUE))
                echo "   ❌ FAIL — ${KEY} has $DUP_N duplicate(s) ($TOTAL entries, $UNIQUE unique)"
                MX_DUPS=$((MX_DUPS + DUP_N))
            fi
        fi
    done
    if [ "$MX_DUPS" -eq 0 ]; then
        echo "   ✅ PASS — no duplicate entries in .mxproject."
    else
        echo "   Run fix_build.sh to deduplicate."
        ERRORS=$((ERRORS + 1))
    fi
else
    echo "   ⚠️  SKIP — .mxproject not found (will be created by CubeMX)."
    WARNINGS=$((WARNINGS + 1))
fi

# ---- Check 9: .mxproject file type validation ----
echo ""
echo "9. .mxproject file type validation"
if [ -f ".mxproject" ]; then
    MX_TYPE_ERR=0
    # Check for .c files in LibFiles (should only contain .h)
    if grep -q "^LibFiles=" .mxproject 2>/dev/null; then
        C_IN_LIB=$(grep "^LibFiles=" .mxproject | tr ';' '\n' | grep '\.c$' | wc -l)
        if [ "$C_IN_LIB" -gt 0 ]; then
            echo "   ❌ FAIL — LibFiles contains $C_IN_LIB .c file(s) (should be .h only)"
            MX_TYPE_ERR=$((MX_TYPE_ERR + C_IN_LIB))
        fi
    fi
    # Check for .h files in SourceFiles (should only contain .c)
    if grep -q "^SourceFiles=" .mxproject 2>/dev/null; then
        H_IN_SRC=$(grep "^SourceFiles=" .mxproject | grep -v "^SourceFiles#" | tr ';' '\n' | grep '\.h$' | wc -l)
        if [ "$H_IN_SRC" -gt 0 ]; then
            echo "   ❌ FAIL — SourceFiles contains $H_IN_SRC .h file(s) (should be .c only)"
            MX_TYPE_ERR=$((MX_TYPE_ERR + H_IN_SRC))
        fi
    fi
    # Check for trailing semicolons producing empty entries
    TRAILING=$(grep -cE '^(SourceFiles|LibFiles|CDefines|HeaderPath)=.*;;' .mxproject 2>/dev/null) || TRAILING=0
    if [ "$TRAILING" -gt 0 ]; then
        echo "   ❌ FAIL — $TRAILING line(s) with trailing/double semicolons (empty entries)"
        MX_TYPE_ERR=$((MX_TYPE_ERR + TRAILING))
    fi
    # Check for bare empty list lines
    BARE=$(grep -cE '^(HeaderFiles|SourceFiles)=;?$' .mxproject 2>/dev/null) || BARE=0
    if [ "$BARE" -gt 0 ]; then
        echo "   ❌ FAIL — $BARE bare empty list line(s) (e.g. HeaderFiles=;)"
        MX_TYPE_ERR=$((MX_TYPE_ERR + BARE))
    fi
    if [ "$MX_TYPE_ERR" -eq 0 ]; then
        echo "   ✅ PASS — .mxproject file types and format are correct."
    else
        echo "   Run fix_build.sh to repair."
        ERRORS=$((ERRORS + 1))
    fi
else
    echo "   ⚠️  SKIP — .mxproject not found."
    WARNINGS=$((WARNINGS + 1))
fi

# ---- Check 10: Unexpected STM32/ folder structure ----
echo ""
echo "10. Folder structure (STM32/ directory)"
IOC_FILE=$(find . -maxdepth 1 -name '*.ioc' -print -quit)
CHECK10_OK=1
if [ -n "$IOC_FILE" ] && grep -q 'ProjectManager\.UnderRoot=false' "$IOC_FILE"; then
    echo "   ❌ FAIL — ProjectManager.UnderRoot=false in $IOC_FILE."
    echo "   This setting causes CubeMX to generate code into an STM32/ subdirectory."
    echo "   Change to ProjectManager.UnderRoot=true or run fix_build.sh."
    ERRORS=$((ERRORS + 1))
    CHECK10_OK=0
fi
if [ -d "STM32" ]; then
    echo "   ❌ FAIL — unexpected STM32/ directory detected."
    if [ -d "STM32/Drivers" ]; then
        echo "      Found STM32/Drivers/ — should be Drivers/ at project root."
    fi
    if [ -d "STM32/Core" ]; then
        echo "      Found STM32/Core/ — should be Core/ at project root."
    fi
    echo "   This typically happens when CubeMX uses a non-standard output path."
    echo "   Run fix_build.sh to move contents to the correct location."
    ERRORS=$((ERRORS + 1))
    CHECK10_OK=0
fi
if [ "$CHECK10_OK" -eq 1 ]; then
    echo "   ✅ PASS — no unexpected STM32/ directory."
fi

# ---- Check 11: test_*.c files excluded from .cproject sourceEntries ----
echo ""
echo "11. Test file exclusions (test_*.c in Core/Src)"
if [ -d "Core/Src" ]; then
    TESTS_ON_DISK=$(find Core/Src -maxdepth 1 -name 'test_*.c' -printf '%f\n' | sort)
    if [ -n "$TESTS_ON_DISK" ]; then
        CUR_EXCL=$(sed -n 's/.*<entry excluding="\([^"]*\)".*name="Core\/Src".*/\1/p' "$CPROJECT")
        # Build set of currently excluded files once for efficient lookup
        CUR_SET=$(echo "$CUR_EXCL" | tr '|' '\n')
        MISSING_TESTS=0
        while IFS= read -r tf; do
            if ! echo "$CUR_SET" | grep -qFx "$tf"; then
                echo "   ❌ FAIL — $tf not excluded from Core/Src sourceEntry"
                MISSING_TESTS=$((MISSING_TESTS + 1))
            fi
        done <<< "$TESTS_ON_DISK"
        if [ "$MISSING_TESTS" -eq 0 ]; then
            echo "   ✅ PASS — all test_*.c files are excluded."
        else
            echo "   $MISSING_TESTS test file(s) not excluded. Run fix_build.sh to repair."
            ERRORS=$((ERRORS + MISSING_TESTS))
        fi
    else
        echo "   ✅ PASS — no test_*.c files found in Core/Src."
    fi
else
    echo "   ⚠️  SKIP — Core/Src directory not found."
    WARNINGS=$((WARNINGS + 1))
fi

# ---- Summary ----
echo ""
echo "======================================"
if [ "$ERRORS" -eq 0 ] && [ "$WARNINGS" -eq 0 ]; then
    echo "✅ All checks passed. Your .cproject is healthy."
    echo ""
    echo "If you still have build errors, try:"
    echo "  1. Delete the Debug/ folder"
    echo "  2. Project → Clean → Clean all"
    echo "  3. Rebuild (Ctrl+B)"
elif [ "$ERRORS" -eq 0 ]; then
    echo "⚠️  $WARNINGS warning(s), 0 errors. The .cproject is likely OK."
else
    echo "❌ $ERRORS error(s) detected."
    echo ""
    echo "To fix automatically:"
    echo "  bash fix_build.sh"
    echo ""
    echo "If fix_build.sh doesn't resolve the issues, you can safely"
    echo "delete .cproject and regenerate it:"
    echo "  1. Delete .cproject"
    echo "  2. Open the .ioc file in STM32CubeMX"
    echo "  3. Click 'Generate Code' (Alt+K)"
    echo "  4. Run: bash fix_build.sh"
    echo "  5. Rebuild in STM32CubeIDE"
fi
echo ""

[ "$ERRORS" -eq 0 ] && exit 0 || exit 1
