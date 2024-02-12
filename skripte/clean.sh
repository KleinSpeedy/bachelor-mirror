#!/bin/sh

set +eux

echo "Deleting artifacts"

rm -r $PWD/svg-inkscape
find . -type f -name "*.aux" -delete \
    -o -name "*.lo?" -delete \
    -o -name "*.bcf" -delete \
    -o -name "*.run.xml" -delete \
    -o -name "*.toc" -delete \
    -o -name "*.out" -delete
