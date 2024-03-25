.PHONY: all docs build clean version_test help


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
	pip3 install pdm
	pdm export -o requirements.txt --without-hashes;\
	pip3 install sphinx==5.0 rst2pdf==0.98;\
	sphinx-build -v -b html doc build/docs/html

clean:
	rm -rf build
	rm -rf venv

.venv: pyproject.toml
	pdm install --dev --no-editable

version_test: .venv
	# Check if it is possible to bump version
	# e.g. all versions in files are the same and eq
	# to version in .bumpversion.cfg
	pdm run bump2version --dry-run --allow-dirty --verbose major

help:
	@echo "Available Targets:"
	@echo "... all"
	@echo "... docs"
	@echo "... clean"
	@echo "... version_test"
