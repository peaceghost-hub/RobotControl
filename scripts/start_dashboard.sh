#!/bin/bash
# Start the web dashboard server

echo "Starting Robot Dashboard Server..."

cd "$(dirname "$0")/../dashboard"

# Check if virtual environment exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Check if dependencies are installed
if ! python3 -c "import flask" 2>/dev/null; then
    echo "Installing dependencies..."
    pip3 install -r requirements.txt
fi

# Initialize database if it doesn't exist
if [ ! -f "../database/robot_monitor.db" ]; then
    echo "Initializing database..."
    python3 ../database/init_db.py
fi

# Start server
echo "Dashboard starting on http://0.0.0.0:5000"
python3 app.py
