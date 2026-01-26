.PHONY: install lint format run check

install:
	uv sync

lint:
	uv run ruff check src

format:
	uv run ruff format src

check: lint
	uv run ruff format --check src

run:
	MOCK_HARDWARE=1 uv run balance-bot
