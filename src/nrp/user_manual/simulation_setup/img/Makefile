IMAGES=$(patsubst %.tex,%.png,$(wildcard *.tex))
PDF_IMAGES=$(patsubst %.pdf,%.png,$(wildcard *.pdf))

%.pdf: %.tex
	pdflatex $< -output-directory=$@

%.png: %.pdf
	convert -flatten -density 200 -trim -quality 100 $< $@

images: $(IMAGES)
	rm -r -f *.pdf *.aux *.log

images_pdf: $(PDF_IMAGES)
