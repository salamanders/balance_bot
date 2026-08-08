.PHONY: install lint format run check test typecheck

install:
	uv sync

lint:
	uv run ruff check src tests
	uv run mypy src

typecheck:
	uv run mypy src

format:
	uv run ruff format src tests

test:
	uv run pytest tests/

check: lint test
	uv run ruff format --check src tests

run:
	uv run balance-bot --allow-mocks

