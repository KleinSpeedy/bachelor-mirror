#!/bin/sh

#
# Build latex document and convert to pdf
#

set +eux

# Check all commands exist
for cmd in lualatex inkscape date biber
do
    if ! command -v ${cmd} >/dev/null 2>&1
    then
        echo "Requires $cmd command!"
        exit 1
    fi
done

TOC=$(date +"%Y_%m_%d")
NAME=Jonas_Schulze_Bachelorarbeit_$TOC

help()
{
    echo "Usage of $(basename $0) [-h] [-n]"
    echo "\t" "-h" "\t" "prints usage"
    echo "\t" "-n" "\t" "do not delete artifacts after building document"
    echo "\t" "no arguments builds project and deletes artifacts after build"
}

build_pdf()
{
    echo "Start creation of $NAME at `pwd`"
    lualatex --shell-escape Bachelorarbeit.tex
    if [ -f $PWD/Bachelorarbeit.bcf ]; then
        biber $PWD/Bachelorarbeit.bcf
    else
        echo "Could not run biber, did not create Bachelorarbeit.bcf file!"
        exit 1
    fi
    lualatex Bachelorarbeit.tex
}

rename_pdf()
{
    # Rename pdf
    if [ -f Bachelorarbeit.pdf ]; then
        mv Bachelorarbeit.pdf $NAME.pdf
    else
        echo "Could not find output pdf, exiting"
        exit 1
    fi
}

while getopts ":hn" opt; do
    case $opt in
        h) # display usage
            help
            exit;;
        n) # dont delete artifacts
            build_pdf
            rename_pdf
            exit;;
        ?) # invalid arguments
            echo "Invalid argument: ${opt}"
            help
            exit;;
    esac
done

build_pdf
rename_pdf
# Cleanup afterwards on success
$PWD/skripte/clean.sh
