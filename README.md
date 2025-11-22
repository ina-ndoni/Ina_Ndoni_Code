# Smart Kitchen Robot Planner — Debugging & Run Instructions

This project contains `smart_kitchen_robot_planner.py`, a small planner that runs BFS and A* over a tiny state space and writes two graph images: `bfs_graph.png` and `astar_graph.png`.

Use this README to set up the environment, reproduce results, and debug the script step-by-step.

Prerequisites
- macOS with Python 3 installed (recommended: 3.8+).
- A terminal using zsh (your default shell).

Quick setup (recommended)
1. Create and activate a virtual environment in the project directory:

```zsh
python3 -m venv .venv
source .venv/bin/activate
```

2. Install runtime dependencies(if you don't have them already):

```zsh
.venv/bin/python3 -m pip install --upgrade pip setuptools wheel
.venv/bin/python3 -m pip install networkx matplotlib
```

Run the script

```zsh
.venv/bin/python3 smart_kitchen_robot_planner.py
```

This should print summary info for BFS and A* and create `bfs_graph.png` and `astar_graph.png` in the same folder (if images are already on the folder from previous debugging, they don't get recreated).

Windows instructions (PowerShell and cmd)
-------------------------------------

If you're on Windows, use the following steps. These examples show PowerShell then cmd variants.

1) Create a virtual environment

PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

cmd.exe:

```cmd
python -m venv .venv
.\.venv\Scripts\activate.bat
```

2) Install dependencies

PowerShell / cmd (after activating the venv):

```powershell
python -m pip install --upgrade pip setuptools wheel
python -m pip install networkx matplotlib
```

3) Run the script

PowerShell / cmd:

```powershell
python smart_kitchen_robot_planner.py
```

This will print the BFS / A* summaries and create `bfs_graph.png` and `astar_graph.png` in the project folder(if images are already on the folder from previous debugging, they don't get recreated).

Debugging on Windows
--------------------

- Use the traceback: when an exception occurs, copy the full traceback and note the file and line number shown.
- Interactive debugger (pdb / ipdb):
	- Run under pdb:

```powershell
python -m pdb smart_kitchen_robot_planner.py
```

	- Install and run ipdb for a nicer shell:

```powershell
python -m pip install ipdb
python -m ipdb smart_kitchen_robot_planner.py
```

- Programmatic breakpoints: add `import pdb; pdb.set_trace()` (or `import ipdb; ipdb.set_trace()`) in the file where you want to inspect variables.

- VS Code debugger on Windows:
	1. Open the project folder in VS Code.
	2. Install the Python extension if you haven't already.
	3. Select the interpreter (Command Palette -> "Python: Select Interpreter") and pick the `.venv\Scripts\python.exe` interpreter.
	4. Open `smart_kitchen_robot_planner.py`, set breakpoints by clicking the gutter, then Run > Start Debugging (F5). You can inspect variables, step, and evaluate expressions in the Debug Console.

Notes
-----
- If PowerShell blocks running scripts, you may need to change the execution policy (PowerShell opened as Admin):

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

	Only change the policy if you understand the security implications; the venv activation script may be blocked by strict policies.

- If you don't want to create a venv, you can install dependencies globally (not recommended) with `python -m pip install networkx matplotlib` and run `python smart_kitchen_robot_planner.py`.

---

