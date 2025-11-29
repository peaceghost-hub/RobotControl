#!/bin/bash
# Setup script for dashboard virtual environment

echo "Setting up Python virtual environment for dashboard..."

# Check if python3 is available
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 is not installed"
    exit 1
fi

# Create virtual environment
echo "Creating virtual environment..."
python3 -m venv dashboard_env

# Activate virtual environment
echo "Activating virtual environment..."
source dashboard_env/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install requirements
echo "Installing requirements..."
pip install -r requirements.txt

echo ""
echo "Virtual environment setup complete!"
echo "To activate the environment, run:"
echo "  source dashboard_env/bin/activate"
echo ""
echo "To run the dashboard:"
echo "  source dashboard_env/bin/activate"
echo "  python app.py"
echo ""
echo "To deactivate:"
echo "  deactivate"