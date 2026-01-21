.PHONY: help setup dev prod migrate seed test clean logs restart

help:
	@echo "AI-Native Robotics Textbook - Development Commands"
	@echo ""
	@echo "Setup & Initialization:"
	@echo "  make setup      - Initial setup (copy .env, build images)"
	@echo "  make migrate    - Run database migrations"
	@echo "  make seed       - Generate sample data"
	@echo ""
	@echo "Development:"
	@echo "  make dev        - Start development environment"
	@echo "  make dev-build  - Rebuild and start development"
	@echo "  make logs       - View all service logs"
	@echo "  make logs-api   - View backend logs"
	@echo "  make logs-web   - View frontend logs"
	@echo "  make restart    - Restart all services"
	@echo "  make stop       - Stop all services"
	@echo ""
	@echo "Testing:"
	@echo "  make test       - Run full test suite"
	@echo "  make test-api   - Run backend tests only"
	@echo "  make test-web   - Run frontend tests only"
	@echo "  make test-lab   - Run lab environment tests"
	@echo ""
	@echo "Production:"
	@echo "  make prod       - Start production environment"
	@echo "  make prod-build - Build production images"
	@echo ""
	@echo "Utilities:"
	@echo "  make clean      - Remove containers and volumes"
	@echo "  make clean-all  - Remove everything (including images)"
	@echo "  make shell-api  - Open backend shell"
	@echo "  make shell-db   - Open database shell"
	@echo "  make health     - Check service health"

setup:
	@echo "Setting up development environment..."
	@if not exist .env copy .env.example .env
	@echo "Please edit .env and add your API keys!"
	docker-compose -f docker-compose.dev.yml build
	@echo "Setup complete! Run 'make dev' to start."

dev:
	@echo "Starting development environment..."
	docker-compose -f docker-compose.dev.yml up -d
	@echo "Services starting... Check 'make logs' for status"
	@echo ""
	@echo "Access points:"
	@echo "  Frontend: http://localhost:3000"
	@echo "  Backend:  http://localhost:8000/docs"
	@echo "  PgAdmin:  http://localhost:5050"

dev-build:
	@echo "Rebuilding development environment..."
	docker-compose -f docker-compose.dev.yml up -d --build

stop:
	@echo "Stopping services..."
	docker-compose -f docker-compose.dev.yml down

restart:
	@echo "Restarting services..."
	docker-compose -f docker-compose.dev.yml restart

migrate:
	@echo "Running database migrations..."
	docker-compose -f docker-compose.dev.yml exec backend alembic upgrade head
	@echo "Migrations complete!"

seed:
	@echo "Generating sample data..."
	docker-compose -f docker-compose.dev.yml exec backend python scripts/generate_sample_data.py
	@echo "Sample data created!"

test:
	@echo "Running full test suite..."
	docker-compose -f docker-compose.dev.yml exec backend python scripts/test_fullstack.py

test-api:
	@echo "Running backend tests..."
	docker-compose -f docker-compose.dev.yml exec backend pytest

test-web:
	@echo "Running frontend tests..."
	cd frontend && npm test

test-lab:
	@echo "Running lab tests..."
	docker-compose -f docker-compose.dev.yml exec backend bash specs/1-book-curriculum/chapters/chapter-1/lab/test_lab.sh

logs:
	docker-compose -f docker-compose.dev.yml logs -f

logs-api:
	docker-compose -f docker-compose.dev.yml logs -f backend

logs-web:
	docker-compose -f docker-compose.dev.yml logs -f frontend

shell-api:
	docker-compose -f docker-compose.dev.yml exec backend bash

shell-db:
	docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -d textbook_db

health:
	@echo "Checking service health..."
	@curl -s http://localhost:8000/health || echo "Backend: DOWN"
	@curl -s http://localhost:3000 || echo "Frontend: DOWN"
	@curl -s http://localhost:6333/health || echo "Qdrant: DOWN"
	docker-compose -f docker-compose.dev.yml ps

prod:
	@echo "Starting production environment..."
	docker-compose up -d
	@echo "Production services running!"

prod-build:
	@echo "Building production images..."
	docker-compose build
	@echo "Production images built!"

clean:
	@echo "Removing containers and volumes..."
	docker-compose -f docker-compose.dev.yml down -v
	@echo "Cleanup complete!"

clean-all:
	@echo "Removing everything (containers, volumes, images)..."
	docker-compose -f docker-compose.dev.yml down -v --rmi all
	@echo "Complete cleanup done!"
