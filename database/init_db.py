"""
Initialize Database
Creates database tables for the dashboard
"""

import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dashboard.app import app, db

def init_database():
    """Initialize database tables"""
    with app.app_context():
        # Create all tables
        db.create_all()
        print("✓ Database tables created successfully")
        
        # Print table information
        print("\nCreated tables:")
        for table in db.metadata.tables.keys():
            print(f"  - {table}")

if __name__ == '__main__':
    init_database()
