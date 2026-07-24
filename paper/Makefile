PAPER = main

all: $(PAPER).pdf

$(PAPER).pdf: $(PAPER).tex references.bib sections/*.tex
	latexmk -pdf -interaction=nonstopmode $(PAPER).tex

clean:
	latexmk -C $(PAPER).tex
	rm -f sections/*.aux

.PHONY: all clean
