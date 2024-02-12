#!/bin/sh

set +eux

# Check all commands exist
for cmd in lualatex inkscape date
do
    if ! command -v "$cmd" >/dev/null 2>&1
    then
        echo "Requires $cmd command!"
        exit 1
    fi
done


TOC=$(date +"%Y_%m_%d")
NAME=Jonas_Schulze_Bachelorarbeit_$TOC

echo "Start creation of $NAME at `pwd`"

lualatex --halt-on-error --shell-escape Bachelorarbeit.tex
lualatex Bachelorarbeit.tex

# Rename pdf
if [ -f Bachelorarbeit.pdf ]; then
    mv Bachelorarbeit.pdf $NAME.pdf
else
    echo "Could not find output pdf, exiting"
    exit 1
fi

# Cleanup afterwards on success
$PWD/skripte/clean.sh
