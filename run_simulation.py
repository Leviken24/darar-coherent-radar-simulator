"""
Launcher for DARAR Distributed Coherent Radar Digital Twin.

Starts the FastAPI simulation engine backend and hosts the 3D interactive engineering environment.
"""

import os
import sys
import webbrowser
import uvicorn

if __name__ == "__main__":
    backend_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "backend")
    sys.path.insert(0, backend_dir)

    port = 8000
    host = "127.0.0.1"
    url = f"http://{host}:{port}"

    print("=" * 70)
    print("  DARAR DISTRIBUTED COHERENT RADAR SIMULATION ENVIRONMENT")
    print("  Based on: 'Distributed coherent detection method for rotating array radar'")
    print("  Journal of Systems Engineering and Electronics, Vol. 37, No. 5, Oct 2026")
    print("=" * 70)
    print(f"\n[+] Starting FastAPI Simulation Server at: {url}")
    print("[+] 3D Digital Twin & Signal Lab available directly in your browser.")
    print("[+] Press Ctrl+C to stop the simulation server.\n")

    uvicorn.run("main:app", app_dir=backend_dir, host=host, port=port, reload=False, log_level="info")
