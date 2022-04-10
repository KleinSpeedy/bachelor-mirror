echo `pwd`
echo "xelatex geht auch"
lualatex --shell-escape MusterAbschlussarbeit.tex
lualatex  MusterAbschlussarbeit.tex
rm *.aux
rm *.lo?
rm *.out
rm *.toc

