#!/bin/sh

set +eux

echo "Deleting artifacts"

if [ -d $PWD/svg-inkscape ]
then
    rm -r $PWD/svg-inkscape
fi

find . -type f -name "*.aux" -delete \
    -o -name "*.lo?" -delete \
    -o -name "*.bcf" -delete \
    -o -name "*.run.xml" -delete \
    -o -name "*.toc" -delete \
    -o -name "*.out" -delete \
    -o -name "*.bbl" -delete \
    -o -name "*.blg" -delete
