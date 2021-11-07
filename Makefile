.PHONY: all docs build clean help


all: docs


# generate build folders
build:
	mkdir -p build/docs

docs: build
	echo "$(dirname "$1")"
	pip3 install virtualenv || return -1
	python3 -m virtualenv venv --system-site-packages || return -1
	# Don't use the source command since not all shells support it.
	. venv/bin/activate;\
	pip3 install -r requirements-py3.txt;\
	sphinx-build -v -b html doc build/docs/html

clean:
	rm -rf build
	rm -rf venv

help:
	@echo "Available Targets:"
	@echo "... all"
	@echo "... docs"