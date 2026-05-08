.PHONY: install run debug lint lint-strict clean

MAP = maps/medium/02_circular_loop.txt

install:
	pip install flake8 mypy --break-system-packages

run:
	python3 main.py $(MAP)

debug:
	python3 -m pdb main.py $(MAP)

lint:
	flake8 .
	mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports \
		--disallow-untyped-defs --check-untyped-defs

lint-strict:
	flake8 .
	mypy . --strict

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name .mypy_cache -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true
