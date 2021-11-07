.PHONY: all docs build clean help


all: docs


# generate build folders
build:
	mkdir -p build/docs

docs: build
	echo "$(dirname "$1")"
	pip3 install virtualenv || return -1
	if [ ! -d venv ]; then\
		python3 -m virtualenv venv --system-site-packages || return -1;\
	fi
	# Don't use the source command since not all shells support it.
	. venv/bin/activate || return -1
	pip3 install -r requirements-py3.txt || return -1
	sphinx-build -b html doc build/docs/html

clean:
	rm -rf build

help:
	@echo "Available Targets:"
	@echo "... all"
	@echo "... docs"