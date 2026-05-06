#!/bin/bash
# Download USGS Digital Elevation Model tiles covering Mars Desert Research
# Station (MDRS), Hanksville, Utah — the URC competition site.
#
# MDRS coordinates: ~38.4060 N, 110.7920 W
# We grab a 4-tile region from USGS National Map at 1/3 arc-second (~10 m) resolution.
#
# Usage:
#   bash download_mdrs_dem.sh
#
# Output: ~/deimos_data/mdrs_dem/*.tif

set -euo pipefail

OUT_DIR="${HOME}/deimos_data/mdrs_dem"
mkdir -p "$OUT_DIR"

# USGS 3DEP 1/3 arc-second tiles. n39w112 covers MDRS region.
# Update tile names if competition zone is at a different coordinate.
TILES=(
    "n39w111"
    "n39w112"
    "n38w111"
    "n38w112"
)

BASE_URL="https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/13/TIFF/historical"

for tile in "${TILES[@]}"; do
    echo "=== Fetching $tile ==="
    # Find the latest version of the tile via S3 listing
    curl -sf "https://prd-tnm.s3.amazonaws.com/?prefix=StagedProducts/Elevation/13/TIFF/historical/${tile}/" \
        | grep -oE "${tile}/USGS_13_${tile}_[0-9]+\.tif" \
        | sort -u \
        | tail -1 \
        | while read -r path; do
            url="https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/13/TIFF/historical/${path}"
            outfile="${OUT_DIR}/$(basename "$path")"
            if [ -f "$outfile" ]; then
                echo "  [skip] $outfile already present"
            else
                echo "  fetch $url"
                curl -fL "$url" -o "$outfile"
            fi
        done
done

echo
echo "DEM tiles in $OUT_DIR:"
ls -la "$OUT_DIR"/*.tif
